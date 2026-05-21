#!/usr/bin/env python3

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
