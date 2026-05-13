#!/usr/bin/env python3
"""
Lightweight planar pose graph:
  - Nodes: keyframe poses in `map` (from /keyframe_map/keyframes).
  - Edges: odometry-like constraints between consecutive keyframes.
  - Loop edges: /keyframe_map/loop_closure_pair [i, j] → identity relative constraint.

**Optimization (no g2o):** variables are ``(x, y, yaw)`` for poses 1..N-1; pose 0 is fixed.
Stacked SE(2) edge residuals (weighted) are minimized with **``scipy.optimize.least_squares``**
(method ``trf``). Depends on ``python3-scipy`` (declared in ``package.xml``).

Publishes /pose_graph/corrected_keyframes (nav_msgs/Path).

Optional **map → odom** correction (`publish_map_odom_tf`): after each successful solve, sets
`T_map_odom = T(corrected_last) * inv(T(raw_last))` (SE2) and republishes it periodically so the
TF tree absorbs drift at the map–odom boundary. **Disable** `launch_slam`'s static `map→odom`
when this is enabled (bringup does that automatically).
"""
from __future__ import annotations

import math
from typing import List, Set, Tuple

import numpy as np
import numpy.linalg as la
import rclpy
from builtin_interfaces.msg import Time as StampMsg
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Int32MultiArray
from tf2_ros import TransformBroadcaster
from tf_transformations import euler_from_quaternion, quaternion_from_euler

try:
    from scipy.optimize import least_squares

    _HAVE_SCIPY = True
except ImportError:
    _HAVE_SCIPY = False

from keyframe_scan_map.pose_graph_se2 import (
    T_from_xyw,
    odom_measurement,
    residual_between,
)


def path_to_xyw(path: Path) -> np.ndarray:
    rows: List[List[float]] = []
    for ps in path.poses:
        q = ps.pose.orientation
        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        rows.append([ps.pose.position.x, ps.pose.position.y, yaw])
    if not rows:
        return np.zeros((0, 3), dtype=np.float64)
    return np.array(rows, dtype=np.float64)


def xyw_to_path(poses: np.ndarray, frame_id: str, stamp) -> Path:
    out = Path()
    out.header.stamp = stamp
    out.header.frame_id = frame_id
    for i in range(poses.shape[0]):
        x, y, yaw = float(poses[i, 0]), float(poses[i, 1]), float(poses[i, 2])
        ps = PoseStamped()
        ps.header = out.header
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = 0.0
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)
        ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        out.poses.append(ps)
    return out


class PoseGraphNode(Node):
    def __init__(self) -> None:
        super().__init__('pose_graph_node')

        self.declare_parameter('keyframes_topic', '/keyframe_map/keyframes')
        self.declare_parameter('loop_pair_topic', '/keyframe_map/loop_closure_pair')
        self.declare_parameter('output_path_topic', '/pose_graph/corrected_keyframes')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('weight_odom', 12.0)
        self.declare_parameter('weight_loop', 55.0)
        self.declare_parameter('max_graph_nodes', 180)
        self.declare_parameter('max_loop_edges', 40)
        self.declare_parameter('publish_map_odom_tf', False)
        self.declare_parameter('map_odom_tf_period_sec', 0.1)
        # SE(2) low-pass on map→odom: 1.0 = use each graph solve as-is; **0.1–0.35** eases TF jumps / map skew
        # when new keyframes or loop edges re-optimize the chain.
        self.declare_parameter('map_odom_tf_smooth_alpha', 1.0)
        # Stamp map→odom with latest /ekf/odom time so tf2 matches lidar-timed odom→base_link.
        self.declare_parameter('odom_stamp_topic', '/ekf/odom')
        # Ignore /ekf/odom samples whose header stamp is this far behind node clock (stale DDS / replay).
        self.declare_parameter('odom_stamp_max_past_sec', 25.0)
        # Ignore stamps this far in the future vs clock (bad clock sync).
        self.declare_parameter('odom_stamp_max_future_sec', 2.0)

        self._kf_topic = self.get_parameter('keyframes_topic').value
        self._pair_topic = self.get_parameter('loop_pair_topic').value
        self._out_topic = self.get_parameter('output_path_topic').value
        self._map_frame = self.get_parameter('map_frame').value
        self._odom_frame = self.get_parameter('odom_frame').value
        self._w_odom = math.sqrt(float(self.get_parameter('weight_odom').value))
        self._w_loop = math.sqrt(float(self.get_parameter('weight_loop').value))
        self._max_n = max(3, int(self.get_parameter('max_graph_nodes').value))
        self._max_loops = int(self.get_parameter('max_loop_edges').value)
        self._pub_tf = bool(self.get_parameter('publish_map_odom_tf').value)
        self._tf_period = float(self.get_parameter('map_odom_tf_period_sec').value)
        self._map_odom_smooth_alpha = float(
            self.get_parameter('map_odom_tf_smooth_alpha').value
        )
        self._map_odom_smooth_have_prior = False

        self._P_init: np.ndarray = np.zeros((0, 3), dtype=np.float64)
        self._loop_edges: List[Tuple[int, int]] = []
        self._loop_seen: Set[Tuple[int, int]] = set()
        self._T_map_odom = np.eye(3, dtype=np.float64)
        self._tf_broadcaster: TransformBroadcaster | None = None
        self._last_odom_stamp: StampMsg | None = None
        self._odom_stamp_max_past = max(
            0.5, float(self.get_parameter('odom_stamp_max_past_sec').value)
        )
        self._odom_stamp_max_future = max(
            0.05, float(self.get_parameter('odom_stamp_max_future_sec').value)
        )
        self._warned_stale_odom_stamp = False
        self._odom_stamp_topic = ''

        self.create_subscription(Path, self._kf_topic, self._on_path, 10)
        self.create_subscription(Int32MultiArray, self._pair_topic, self._on_loop_pair, 20)
        self._pub = self.create_publisher(Path, self._out_topic, 10)

        if self._pub_tf:
            self._tf_broadcaster = TransformBroadcaster(self)
            period = max(0.05, self._tf_period)
            self.create_timer(period, self._publish_map_odom_tf)
            stamp_topic = str(self.get_parameter('odom_stamp_topic').value).strip()
            self._odom_stamp_topic = stamp_topic
            if stamp_topic:
                self.create_subscription(Odometry, stamp_topic, self._on_odom_stamp, 10)

        if not _HAVE_SCIPY:
            self.get_logger().warn(
                'python3-scipy not available; pose_graph_node will echo keyframes without '
                'optimization. Install rosdep: python3-scipy'
            )

        self.get_logger().info(
            f'pose_graph: keyframes={self._kf_topic} -> {self._out_topic} '
            f'publish_map_odom_tf={self._pub_tf}'
            + (
                f' map_odom_smooth_alpha={self._map_odom_smooth_alpha:g}'
                if self._pub_tf and self._map_odom_smooth_alpha < 1.0 - 1e-9
                else ''
            )
        )

    def _on_odom_stamp(self, msg: Odometry) -> None:
        """Latch latest *reasonable* /ekf/odom stamp for map→odom TF (avoids TF_OLD_DATA from stale DDS)."""
        st = msg.header.stamp
        t = Time.from_msg(st)
        now = self.get_clock().now()
        age_past = (now - t).nanoseconds * 1e-9
        age_future = (t - now).nanoseconds * 1e-9
        if age_future > self._odom_stamp_max_future:
            return
        if age_past > self._odom_stamp_max_past:
            if not self._warned_stale_odom_stamp:
                self.get_logger().warn(
                    f'Ignoring {self._odom_stamp_topic!r} stamps '
                    f'> {self._odom_stamp_max_past:.1f}s behind clock (stale interleaved messages / '
                    f'another host on ROS_DOMAIN_ID). map→odom uses wall time until fresh odom.',
                    throttle_duration_sec=30.0,
                )
                self._warned_stale_odom_stamp = True
            return
        if self._last_odom_stamp is None:
            self._last_odom_stamp = st
            return
        if t >= Time.from_msg(self._last_odom_stamp):
            self._last_odom_stamp = st

    def _publish_map_odom_tf(self) -> None:
        if self._tf_broadcaster is None:
            return
        T = self._T_map_odom
        yaw = math.atan2(float(T[1, 0]), float(T[0, 0]))
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)
        msg = TransformStamped()
        # Match EKF odometry stamp so map→odom chains with odom→base_link at LiDAR/IMU times.
        if self._last_odom_stamp is not None:
            msg.header.stamp = self._last_odom_stamp
        else:
            msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._map_frame
        msg.child_frame_id = self._odom_frame
        msg.transform.translation.x = float(T[0, 2])
        msg.transform.translation.y = float(T[1, 2])
        msg.transform.translation.z = 0.0
        msg.transform.rotation.x = float(qx)
        msg.transform.rotation.y = float(qy)
        msg.transform.rotation.z = float(qz)
        msg.transform.rotation.w = float(qw)
        self._tf_broadcaster.sendTransform(msg)

    def _on_loop_pair(self, msg: Int32MultiArray) -> None:
        if len(msg.data) < 2:
            return
        a, b = int(msg.data[0]), int(msg.data[1])
        if a == b:
            return
        lo, hi = (a, b) if a < b else (b, a)
        key = (lo, hi)
        if key in self._loop_seen:
            return
        self._loop_seen.add(key)
        self._loop_edges.append((a, b))
        if len(self._loop_edges) > self._max_loops:
            self._loop_edges = self._loop_edges[-self._max_loops :]
            self._loop_seen = {
                (min(x, y), max(x, y)) for x, y in self._loop_edges
            }
        self.get_logger().info(f'Pose graph: added loop edge ({a}, {b})')
        self._optimize_and_publish()

    def _on_path(self, msg: Path) -> None:
        self._P_init = path_to_xyw(msg)
        self._optimize_and_publish()

    def _optimize_and_publish(self) -> None:
        if self._P_init.shape[0] < 2:
            return

        n_all = self._P_init.shape[0]
        if n_all > self._max_n:
            self.get_logger().warn(
                f'Keyframes {n_all} > max_graph_nodes {self._max_n}; skipping pose graph',
                throttle_duration_sec=8.0,
            )
            self._pub.publish(
                xyw_to_path(
                    self._P_init,
                    self._map_frame,
                    self.get_clock().now().to_msg(),
                )
            )
            return

        P0 = self._P_init.copy()
        n = P0.shape[0]
        z_odom: List[np.ndarray] = []
        for i in range(n - 1):
            z_odom.append(odom_measurement(P0, i, i + 1))

        I3 = np.eye(3, dtype=np.float64)

        if not _HAVE_SCIPY:
            self._pub.publish(
                xyw_to_path(P0, self._map_frame, self.get_clock().now().to_msg())
            )
            return

        x0 = P0[1:, :].copy().ravel()

        def fun(x_flat: np.ndarray) -> np.ndarray:  # residual vector for scipy.least_squares
            p = np.zeros((n, 3), dtype=np.float64)
            p[0, :] = P0[0, :]
            p[1:, :] = x_flat.reshape(n - 1, 3)
            blocks: List[np.ndarray] = []
            for i in range(n - 1):
                pred = odom_measurement(p, i, i + 1)
                blocks.append(self._w_odom * residual_between(pred, z_odom[i]))
            for a, b in self._loop_edges:
                if a < 0 or b < 0 or a >= n or b >= n:
                    continue
                pred = odom_measurement(p, a, b)
                blocks.append(self._w_loop * residual_between(pred, I3))
            if not blocks:
                return np.zeros(0, dtype=np.float64)
            return np.concatenate(blocks)

        res = least_squares(  # trust-region reflective; dense Jacobian from finite diff
            fun,
            x0,
            method='trf',
            max_nfev=min(400, 40 * n + 80 * len(self._loop_edges)),
            ftol=1e-6,
            xtol=1e-6,
        )
        P_opt = np.zeros((n, 3), dtype=np.float64)
        P_opt[0, :] = P0[0, :]
        P_opt[1:, :] = res.x.reshape(n - 1, 3)
        for k in range(n):
            P_opt[k, 2] = float(
                math.atan2(math.sin(P_opt[k, 2]), math.cos(P_opt[k, 2]))
            )

        self._pub.publish(
            xyw_to_path(P_opt, self._map_frame, self.get_clock().now().to_msg())
        )

        if self._pub_tf:
            T_new = T_from_xyw(P_opt[-1]) @ la.inv(T_from_xyw(P0[-1]))
            a = max(0.0, min(1.0, float(self._map_odom_smooth_alpha)))
            if not self._map_odom_smooth_have_prior or a >= 1.0 - 1e-12:
                self._T_map_odom = T_new
                self._map_odom_smooth_have_prior = True
            else:
                T_old = self._T_map_odom
                tx_o, ty_o = float(T_old[0, 2]), float(T_old[1, 2])
                tx_n, ty_n = float(T_new[0, 2]), float(T_new[1, 2])
                yaw_o = math.atan2(float(T_old[1, 0]), float(T_old[0, 0]))
                yaw_n = math.atan2(float(T_new[1, 0]), float(T_new[0, 0]))
                dyaw = math.atan2(
                    math.sin(yaw_n - yaw_o), math.cos(yaw_n - yaw_o)
                )
                yaw_s = yaw_o + a * dyaw
                cs, sn = math.cos(yaw_s), math.sin(yaw_s)
                Ts = np.eye(3, dtype=np.float64)
                Ts[0, 0], Ts[0, 1] = cs, -sn
                Ts[1, 0], Ts[1, 1] = sn, cs
                Ts[0, 2] = (1.0 - a) * tx_o + a * tx_n
                Ts[1, 2] = (1.0 - a) * ty_o + a * ty_n
                self._T_map_odom = Ts


def main() -> None:
    rclpy.init()
    node = PoseGraphNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
