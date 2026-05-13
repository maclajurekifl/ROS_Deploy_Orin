#!/usr/bin/env python3
"""
Quick orientation sanity check: express Livox + Microstrain angular velocity in base_link
and print planar (x,y) gyro and z — useful while slowly yawing the robot on replay or live.

Usage (after: source /opt/ros/humble/setup.bash && source install/setup.bash):
  ros2 bag play your.db3 --clock   # terminal 1
  ros2 launch ros_project_bringup launch_slam.launch.py use_sim_time:=true launch_sensors:=false

  # terminal 2 — run ~15s while rotating in place (~yaw only):
  python3 scripts/compare_imu_gyro_base.py --duration 15

If TF is missing at IMU stamp, the script falls back to latest transform (same as ekf_node for static extrinsics).

LiDAR / mapping / EKF: see docstring at bottom of this file for RViz + topic checks (no extra deps).
"""
from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import TransformStamped, Vector3Stamped
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_geometry_msgs import do_transform_vector3
from tf2_ros import Buffer, TransformException, TransformListener


class CompareImuGyro(Node):
    def __init__(self, args):
        super().__init__("compare_imu_gyro_base")
        self.base = args.base_frame
        self.livox_topic = args.livox_topic
        self.gx5_topic = args.gx5_topic
        self.duration = float(args.duration)
        self.period = float(args.period)
        self._last_print = 0.0
        self._livox: Imu | None = None
        self._gx5: Imu | None = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(Imu, self.livox_topic, self._livox_cb, 50)
        self.create_subscription(Imu, self.gx5_topic, self._gx5_cb, 50)

        self.get_logger().info(
            f"Comparing gyro in {self.base!r}: {self.livox_topic} vs {self.gx5_topic} "
            f"for {self.duration:g}s every {self.period:g}s — yaw slowly in place."
        )

    def _livox_cb(self, msg: Imu):
        self._livox = msg

    def _gx5_cb(self, msg: Imu):
        self._gx5 = msg

    def _to_base(self, msg: Imu, label: str) -> tuple[float, float, float] | None:
        imu_frame = (msg.header.frame_id or "").strip()
        if not imu_frame:
            self.get_logger().warn(f"{label}: empty header.frame_id")
            return None
        vs = Vector3Stamped()
        vs.header = msg.header
        vs.vector = msg.angular_velocity
        try:
            tf_msg: TransformStamped = self.tf_buffer.lookup_transform(
                self.base,
                imu_frame,
                rclpy.time.Time.from_msg(msg.header.stamp),
                timeout=Duration(seconds=0.12),
            )
        except TransformException:
            try:
                tf_msg = self.tf_buffer.lookup_transform(
                    self.base,
                    imu_frame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.12),
                )
            except TransformException as exc:
                self.get_logger().warn(f"{label}: TF {self.base}<-{imu_frame}: {exc}")
                return None
        out = do_transform_vector3(vs, tf_msg)
        return (out.vector.x, out.vector.y, out.vector.z)

    def spin_compare(self):
        end = time.monotonic() + self.duration
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            now = time.monotonic()
            if now - self._last_print < self.period:
                continue
            self._last_print = now
            if self._livox is None or self._gx5 is None:
                self.get_logger().info("waiting for both IMU topics…")
                continue
            lv = self._to_base(self._livox, "livox")
            gx = self._to_base(self._gx5, "gx5")
            if lv is None or gx is None:
                continue
            # Planar magnitude (rad/s) ignoring z for "are we rotating together sideways"
            lv_xy = math.hypot(lv[0], lv[1])
            gx_xy = math.hypot(gx[0], gx[1])
            self.get_logger().info(
                f"Livox@{self._livox.header.frame_id!r} in {self.base}: "
                f"ω=({lv[0]:+.4f},{lv[1]:+.4f},{lv[2]:+.4f}) |xy|={lv_xy:.4f}  ||  "
                f"GX5@{self._gx5.header.frame_id!r} in {self.base}: "
                f"ω=({gx[0]:+.4f},{gx[1]:+.4f},{gx[2]:+.4f}) |xy|={gx_xy:.4f}"
            )
            dz = lv[2] - gx[2]
            self.get_logger().info(
                f"  Δω_z (livox-gx5) in {self.base}: {dz:+.4f} rad/s  "
                f"(near zero when both agree for yaw about base Z; mount yaw on GX5 mixes x,y)"
            )


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--livox-topic", default="/livox/imu")
    p.add_argument("--gx5-topic", default="/imu/data")
    p.add_argument("--base-frame", default="base_link")
    p.add_argument("--duration", type=float, default=15.0)
    p.add_argument("--period", type=float, default=0.5, help="seconds between log lines")
    args = p.parse_args()

    rclpy.init()
    node = CompareImuGyro(args)
    try:
        node.spin_compare()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

    print(
        "\n--- LiDAR / map / EKF quick checks (RViz) ---\n"
        "1) Fixed frame base_link: add /livox/lidar — axes should match robot +X forward.\n"
        "2) Fixed frame odom: same cloud — should move smoothly with /ekf/odom if TF is consistent.\n"
        "3) Add TF display: base_link, livox_frame, imu_link — static offsets should match slam_bringup.\n"
        "4) ros2 topic echo /lidar/odom nav_msgs/msg/Odometry --field pose (while moving) vs /ekf/odom.\n"
        "5) If Livox vs GX5 ω in base disagree under pure yaw, check imu_mount_* and livox extrinsic.\n",
        file=sys.stderr,
    )


if __name__ == "__main__":
    main()
