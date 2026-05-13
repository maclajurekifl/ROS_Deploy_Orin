#!/usr/bin/env python3
"""
Relay FAST-LIO /Odometry (camera_init -> body) onto /lidar/odom (odom -> base_link)
with the same pose/twist/covariance so localisation_ekf + frame checks stay unchanged.

Assumes odom is aligned with FAST-LIO world (camera_init) and base_link with body
for this workspace (see lio_bringup README).

Optional ``body_to_base_yaw_deg`` (default 0): planar rotation from FAST-LIO **body** to robot
``base_link`` (REP-103 +X forward). When RViz ``/ekf/path`` +X points **aft** while the robot
drives **forward**, try **180.0** — Livox mount yaw is separate (``livox_extrinsic_yaw_deg``).

Optional ``publish_tf`` (default false): broadcast ``odom`` -> ``base_link`` from each relayed
message so one node owns that TF (use with ``ekf_node`` ``publish_tf``: false in LIO mode).

Optional ``sync_tf_cloud_topic``: when set and ``publish_tf`` is true, also broadcast the same
transform with each point-cloud ``header.stamp`` so ``tf2`` / message_filters can interpolate at
LiDAR time (last odometry pose held forward until the next /Odometry).
"""
from __future__ import annotations

import math
from copy import deepcopy
from typing import Optional

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from tf2_ros import TransformBroadcaster
from tf_transformations import euler_from_quaternion, quaternion_from_euler


def _wrap_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _rotate_twist_body_to_base(tw, yaw_rad: float) -> None:
    """Twist is expressed in child frame; rotate linear + angular from body to base (planar Z)."""
    if abs(yaw_rad) < 1e-9:
        return
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    lx, ly, lz = tw.linear.x, tw.linear.y, tw.linear.z
    tw.linear.x = c * lx - s * ly
    tw.linear.y = s * lx + c * ly
    tw.linear.z = lz
    ax, ay, az = tw.angular.x, tw.angular.y, tw.angular.z
    tw.angular.x = c * ax - s * ay
    tw.angular.y = s * ax + c * ay
    tw.angular.z = az


def _apply_body_to_base_yaw(msg: Odometry, yaw_rad: float) -> Odometry:
    """Return a copy with pose + twist adjusted by fixed yaw about Z (body -> base_link)."""
    out = deepcopy(msg)
    q = out.pose.pose.orientation
    roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
    yaw2 = _wrap_angle(float(yaw) + float(yaw_rad))
    qx, qy, qz, qw = quaternion_from_euler(float(roll), float(pitch), float(yaw2))
    out.pose.pose.orientation.x = float(qx)
    out.pose.pose.orientation.y = float(qy)
    out.pose.pose.orientation.z = float(qz)
    out.pose.pose.orientation.w = float(qw)
    _rotate_twist_body_to_base(out.twist.twist, yaw_rad)
    return out


class LioOdomRelayNode(Node):
    def __init__(self) -> None:
        super().__init__('lio_odom_relay_node')
        self.declare_parameter('in_topic', '/Odometry')
        self.declare_parameter('out_topic', '/lidar/odom')
        self.declare_parameter('out_frame_id', 'odom')
        self.declare_parameter('out_child_frame_id', 'base_link')
        self.declare_parameter('publish_tf', False)
        self.declare_parameter('sync_tf_cloud_topic', '')
        self.declare_parameter('body_to_base_yaw_deg', 0.0)

        in_t = self.get_parameter('in_topic').value
        out_t = self.get_parameter('out_topic').value
        self._frame = str(self.get_parameter('out_frame_id').value)
        self._child = str(self.get_parameter('out_child_frame_id').value)
        self._publish_tf = bool(self.get_parameter('publish_tf').value)
        sync_topic = str(self.get_parameter('sync_tf_cloud_topic').value).strip()
        self._sync_cloud = bool(self._publish_tf and sync_topic)
        self._yaw_fix = math.radians(float(self.get_parameter('body_to_base_yaw_deg').value))
        self._last_pose: Optional[Odometry] = None
        self._tf_broadcaster = TransformBroadcaster(self) if self._publish_tf else None

        qos = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
        )
        cloud_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Odometry, in_t, self._cb, qos)
        self._pub = self.create_publisher(Odometry, out_t, qos)
        if self._sync_cloud:
            self.create_subscription(PointCloud2, sync_topic, self._on_cloud, cloud_qos)
        extra = ''
        if abs(self._yaw_fix) > 1e-9:
            extra = f'; body→base yaw fix {math.degrees(self._yaw_fix):.1f}deg'
        self.get_logger().info(
            f'lio_odom_relay: {in_t} (camera_init/body) -> {out_t} ({self._frame} -> {self._child})'
            + (f'; TF {self._frame}->{self._child}' if self._publish_tf else '')
            + (f'; TF sync cloud={sync_topic!r}' if self._sync_cloud else '')
            + extra
        )

    def _send_tf(self, stamp, pose: Odometry) -> None:
        if self._tf_broadcaster is None:
            return
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self._frame
        t.child_frame_id = self._child
        t.transform.translation.x = pose.pose.pose.position.x
        t.transform.translation.y = pose.pose.pose.position.y
        t.transform.translation.z = pose.pose.pose.position.z
        t.transform.rotation = pose.pose.pose.orientation
        self._tf_broadcaster.sendTransform(t)

    def _cb(self, msg: Odometry) -> None:
        out = _apply_body_to_base_yaw(msg, self._yaw_fix) if abs(self._yaw_fix) > 1e-9 else msg
        self._last_pose = out
        out2 = Odometry()
        out2.header.stamp = out.header.stamp
        out2.header.frame_id = self._frame
        out2.child_frame_id = self._child
        out2.pose = out.pose
        out2.twist = out.twist
        self._pub.publish(out2)
        if self._tf_broadcaster is not None:
            if not self._sync_cloud:
                self._send_tf(out2.header.stamp, out2)

    def _on_cloud(self, msg: PointCloud2) -> None:
        if self._tf_broadcaster is None or not self._sync_cloud or self._last_pose is None:
            return
        self._send_tf(msg.header.stamp, self._last_pose)


def main() -> None:
    rclpy.init()
    node = LioOdomRelayNode()
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
