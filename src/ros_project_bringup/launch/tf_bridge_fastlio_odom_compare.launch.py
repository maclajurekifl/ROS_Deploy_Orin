#!/usr/bin/env python3
"""
TF helpers so FAST-LIO /Odometry (camera_init -> body) can be projected into robot `odom` + `base_link`
when recording with `odom_trajectory_tools.py record --output-parent-frame odom --compose-output-child base_link`.

Typical replay issues:
- FAST-LIO publishes `camera_init` -> `body` but nothing connects `camera_init` to robot `odom`.
- Bag has `map` -> `base_link` (or `map` -> `odom` -> `base_link`) but `tf2_echo odom base_link` still fails
  because `odom` was never on the same tree as `map`.

This launch (with sim time) publishes:
1) **Always:** static `odom` -> `camera_init` (identity). Do not start if your bag already defines a
   different `odom` -> `camera_init` edge.
2) **Optional (`bridge_odom_to_map:=true`):** static `odom` -> `map` (identity). Use **only** when
   the bag has `map` -> `base_link` (or similar) but **no** `map`->`odom` and **no** `odom`->`map`
   (i.e. `map` was the world root). **Do not** enable if the bag already publishes `map`->`odom` or
   `odom`->`map` (would create conflicting parents).
3) **Optional (`bridge_body_to_base_link:=true`):** static `body` -> `base_link` (identity). Prefer **`false`**
   when **EKF** publishes `odom`→`base_link` (else two parents for `base_link`); use FAST-LIO
   **`publish_tf:=true`** instead so `body` is on `/tf`. Enable `bridge_body_to_base_link` only when
   no EKF/bag `odom`→`base_link` and **body** would otherwise be unreachable from **base_link**.

After bridges + bag play + FAST-LIO, verify:
  ros2 run tf2_ros tf2_echo odom base_link
  ros2 run tf2_ros tf2_echo body base_link
"""
from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetUseSimTime


def generate_launch_description():
    sim = LaunchConfiguration('use_sim_time')
    bridge_map = LaunchConfiguration('bridge_odom_to_map')
    bridge_body_bl = LaunchConfiguration('bridge_body_to_base_link')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='true',
                description='Must match `ros2 bag play ... --clock` and FAST-LIO.',
            ),
            DeclareLaunchArgument(
                'bridge_odom_to_map',
                default_value='false',
                description=(
                    'If true, publish static odom->map (identity). Enable when map/base_link tree '
                    'is disconnected from odom (see launch file docstring). Risky if bag already has odom->map.'
                ),
            ),
            DeclareLaunchArgument(
                'bridge_body_to_base_link',
                default_value='false',
                description=(
                    'If true, publish static body->base_link (identity). Leave false when ekf_node '
                    'publishes odom->base_link (conflicting TF parents). Use FAST-LIO publish_tf:=true instead.'
                ),
            ),
            GroupAction(
                actions=[SetUseSimTime(True)],
                condition=IfCondition(sim),
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='odom_to_camera_init_for_fastlio_record',
                parameters=[{'use_sim_time': sim}],
                arguments=[
                    '--x',
                    '0',
                    '--y',
                    '0',
                    '--z',
                    '0',
                    '--roll',
                    str(0.0),
                    '--pitch',
                    str(0.0),
                    '--yaw',
                    str(0.0),
                    '--frame-id',
                    'odom',
                    '--child-frame-id',
                    'camera_init',
                ],
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='odom_to_map_for_fastlio_record',
                parameters=[{'use_sim_time': sim}],
                arguments=[
                    '--x',
                    '0',
                    '--y',
                    '0',
                    '--z',
                    '0',
                    '--roll',
                    str(0.0),
                    '--pitch',
                    str(0.0),
                    '--yaw',
                    str(0.0),
                    '--frame-id',
                    'odom',
                    '--child-frame-id',
                    'map',
                ],
                condition=IfCondition(bridge_map),
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='body_to_base_link_for_fastlio_record',
                parameters=[{'use_sim_time': sim}],
                arguments=[
                    '--x',
                    '0',
                    '--y',
                    '0',
                    '--z',
                    '0',
                    '--roll',
                    str(0.0),
                    '--pitch',
                    str(0.0),
                    '--yaw',
                    str(0.0),
                    '--frame-id',
                    'body',
                    '--child-frame-id',
                    'base_link',
                ],
                condition=IfCondition(bridge_body_bl),
            ),
        ]
    )
