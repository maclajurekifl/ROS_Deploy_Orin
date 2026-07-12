#!/usr/bin/env python3
"""
Livox MID360 node — parameters are launch arguments (defaults match prior hardcoded values).

Override from parent launch (e.g. ros_project_bringup) or the command line, e.g.:
  ros2 launch livox_ros_driver2 rviz_MID360_launch.py publish_freq:=20.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    def _t(name: str, default: str, desc: str) -> DeclareLaunchArgument:
        return DeclareLaunchArgument(name, default_value=default, description=desc)

    user_config_default = PathJoinSubstitution(
        [FindPackageShare('livox_ros_driver2'), 'config', 'MID360_config.json']
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'user_config',
                default_value=user_config_default,
                description='Full path to Livox JSON (IP, ports, pattern).',
            ),
            _t('xfer_format', '0', '0=PointCloud2 PointXYZRTL, 1=custom livox point type'),
            _t('multi_topic', '0', '0=shared topic, 1=one topic per unit'),
            _t('data_src', '0', '0=lidar (see driver docs)'),
            _t('publish_freq', '10.0', 'Publish rate (Hz)'),
            _t('output_type', '0', 'output_data_type (see driver)'),
            _t('frame_id', 'livox_frame', 'PointCloud2 / TF frame_id for LiDAR'),
            _t('cmdline_bd_code', 'livox0000000001', 'Command-line board code (see driver)'),
            _t('lvx_file_path', '', 'Path to LVX file; empty = live'),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                description='Follow /clock when true (bag). ros_project_bringup passes true/false explicitly.',
            ),
            Node(
                package='livox_ros_driver2',
                executable='livox_ros_driver2_node',
                name='livox_lidar_publisher',
                output='screen',
                parameters=[
                    {
                        'xfer_format': ParameterValue(
                            LaunchConfiguration('xfer_format'), value_type=int
                        ),
                        'multi_topic': ParameterValue(
                            LaunchConfiguration('multi_topic'), value_type=int
                        ),
                        'data_src': ParameterValue(
                            LaunchConfiguration('data_src'), value_type=int
                        ),
                        'publish_freq': ParameterValue(
                            LaunchConfiguration('publish_freq'), value_type=float
                        ),
                        'output_data_type': ParameterValue(
                            LaunchConfiguration('output_type'), value_type=int
                        ),
                        'frame_id': ParameterValue(
                            LaunchConfiguration('frame_id'), value_type=str
                        ),
                        'lvx_file_path': ParameterValue(
                            LaunchConfiguration('lvx_file_path'), value_type=str
                        ),
                        'user_config_path': ParameterValue(
                            LaunchConfiguration('user_config'), value_type=str
                        ),
                        'cmdline_input_bd_code': ParameterValue(
                            LaunchConfiguration('cmdline_bd_code'), value_type=str
                        ),
                        'use_sim_time': ParameterValue(
                            LaunchConfiguration('use_sim_time'), value_type=bool
                        ),
                    }
                ],
            ),
        ]
    )
