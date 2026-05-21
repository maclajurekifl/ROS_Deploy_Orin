#!/usr/bin/env python3

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformException, TransformListener


def _trans_norm(t) -> float:
    tr = t.transform.translation
    return float(math.hypot(tr.x, tr.y))


class NdtEkfTimeDiagnose(Node):
    def __init__(self) -> None:
        super().__init__('ndt_ekf_time_diagnose')
        self.declare_parameter('cloud_topic', '/livox/lidar')
        self.declare_parameter('delta_topic', '/lidar/relative_motion')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('log_every', 8)
        self.declare_parameter('tf_timeout_sec', 0.75)

        self._odom = str(self.get_parameter('odom_frame').value).strip()
        self._base = str(self.get_parameter('base_frame').value).strip()
        self._every = max(1, int(self.get_parameter('log_every').value))
        self._tf_to = float(self.get_parameter('tf_timeout_sec').value)
        self._to = Duration(seconds=self._tf_to)

        self._tfbuf = Buffer(cache_time=Duration(seconds=120.0))
        self._tflis = TransformListener(self._tfbuf, self, spin_thread=True)

        self._last_delta: TwistStamped | None = None
        dt = str(self.get_parameter('delta_topic').value).strip()
        self.create_subscription(TwistStamped, dt, self._on_delta, 20)

        ct = str(self.get_parameter('cloud_topic').value).strip()
        self.create_subscription(PointCloud2, ct, self._on_cloud, qos_profile_sensor_data)

        self._n = 0
        self.get_logger().info(
            f'ndt_ekf_time_diagnose: cloud={ct!r} delta={dt!r} '
            f'lookup {self._odom!r}->{self._base!r} at cloud stamp every {self._every} msgs'
        )

    def _on_delta(self, msg: TwistStamped) -> None:
        self._last_delta = msg

    def _on_cloud(self, msg: PointCloud2) -> None:
        self._n += 1
        if self._n % self._every != 0:
            return
        st = msg.header.stamp
        if int(st.sec) == 0 and int(st.nanosec) == 0:
            self.get_logger().warn('cloud stamp is zero — skip')
            return
        t_cloud = Time.from_msg(st)

        at_cloud = 'FAIL'
        at_cloud_err = ''
        n_cloud = float('nan')
        try:
            tr = self._tfbuf.lookup_transform(self._odom, self._base, t_cloud, timeout=self._to)
            at_cloud = 'OK'
            n_cloud = _trans_norm(tr)
        except TransformException as e:
            at_cloud_err = str(e).replace('\n', ' ')

        latest = 'FAIL'
        n_latest = float('nan')
        latest_err = ''
        try:
            tr2 = self._tfbuf.lookup_transform(self._odom, self._base, Time(), timeout=self._to)
            latest = 'OK'
            n_latest = _trans_norm(tr2)
        except TransformException as e:
            latest_err = str(e).replace('\n', ' ')

        if self._last_delta is None:
            rel = 'last_rel_motion=n/a (no /lidar/relative_motion yet)'
        else:
            dx = float(self._last_delta.twist.linear.x)
            dy = float(self._last_delta.twist.linear.y)
            dyaw = float(self._last_delta.twist.angular.z)
            rel = f'last_rel_motion dx={dx:.4f} dy={dy:.4f} dtheta={dyaw:.4f}'

        self.get_logger().info(
            f'cloud#{self._n} stamp={st.sec}.{st.nanosec:09d} | '
            f'TF_at_cloud={at_cloud} xy={n_cloud:.3f}m'
            + (f' err={at_cloud_err[:120]}' if at_cloud_err else '')
            + f' | TF_latest={latest} xy={n_latest:.3f}m'
            + (f' err={latest_err[:80]}' if latest_err else '')
            + f' | {rel}'
        )


def main() -> None:
    rclpy.init()
    node = NdtEkfTimeDiagnose()
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
