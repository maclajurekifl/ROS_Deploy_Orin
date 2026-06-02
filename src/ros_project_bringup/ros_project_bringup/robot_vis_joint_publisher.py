#!/usr/bin/env python3
"""Publish wheel/steer joint states from odometry for RViz robot_model display."""
from __future__ import annotations

import math
from typing import Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState


def _stamp_sec(msg: Odometry) -> float:
    t = msg.header.stamp
    return float(t.sec) + float(t.nanosec) * 1e-9


class RobotVisJointPublisher(Node):
    def __init__(self) -> None:
        super().__init__('robot_vis_joint_publisher')
        self.declare_parameter('odom_topic', '/ekf/odom')
        self.declare_parameter('wheel_radius_m', 0.08)
        self.declare_parameter('wheelbase_m', 0.56)
        self.declare_parameter('min_speed_mps', 0.05)
        self.declare_parameter('max_steer_rad', 0.785)
        self.declare_parameter('publish_rate_hz', 30.0)

        self._wheel_r = float(self.get_parameter('wheel_radius_m').value)
        self._wheelbase = float(self.get_parameter('wheelbase_m').value)
        self._min_v = float(self.get_parameter('min_speed_mps').value)
        self._max_steer = float(self.get_parameter('max_steer_rad').value)
        rate = max(1.0, float(self.get_parameter('publish_rate_hz').value))

        self._steer = 0.0
        self._rear_roll = 0.0
        self._last_stamp: Optional[float] = None

        qos = rclpy.qos.QoSProfile(depth=10)
        topic = str(self.get_parameter('odom_topic').value)
        self._pub = self.create_publisher(JointState, '/joint_states', qos)
        self.create_subscription(Odometry, topic, self._on_odom, qos)
        self.create_timer(1.0 / rate, self._publish)

        self.get_logger().info(
            f'Robot vis joints from {topic!r} (steer + rear roll, r={self._wheel_r:.3f} m)'
        )

    def _on_odom(self, msg: Odometry) -> None:
        t = _stamp_sec(msg)
        if self._last_stamp is not None:
            dt = t - self._last_stamp
            if dt <= 0.0 or dt > 1.0:
                dt = 0.0
        else:
            dt = 0.0
        self._last_stamp = t

        vx = float(msg.twist.twist.linear.x)
        vy = float(msg.twist.twist.linear.y)
        wz = float(msg.twist.twist.angular.z)
        speed = math.hypot(vx, vy)

        if speed >= self._min_v:
            steer = math.atan2(vy, vx)
        elif abs(wz) > 1e-4:
            steer = math.atan2(self._wheelbase * wz, self._min_v)
        else:
            steer = self._steer

        steer = max(-self._max_steer, min(self._max_steer, steer))
        self._steer = steer

        if dt > 0.0 and self._wheel_r > 1e-6:
            self._rear_roll += (vx / self._wheel_r) * dt

    def _publish(self) -> None:
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [
            'front_steer_joint',
            'rear_left_wheel_joint',
            'rear_right_wheel_joint',
        ]
        js.position = [self._steer, self._rear_roll, self._rear_roll]
        js.velocity = []
        js.effort = []
        self._pub.publish(js)


def main() -> None:
    rclpy.init()
    node = RobotVisJointPublisher()
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
