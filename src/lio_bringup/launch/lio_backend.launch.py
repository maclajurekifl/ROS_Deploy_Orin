
from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _lio_backend_setup(context, *args, **kwargs):
    fastlio_rel = LaunchConfiguration('fastlio_params_file').perform(context).strip().lstrip('/')
    overlay_rel = LaunchConfiguration('lio_overlay_params_file').perform(context).strip().lstrip('/')
    bag_rel = LaunchConfiguration('lio_bag_overlay_params_file').perform(context).strip()
    sim_raw = LaunchConfiguration('use_sim_time').perform(context).strip().lower()
    use_sim = sim_raw not in ('false', '0', 'no', 'off')
    relay_tf_raw = LaunchConfiguration('lio_relay_publish_tf').perform(context).strip().lower()
    relay_publish_tf = relay_tf_raw in ('true', '1', 'yes', 'on')
    sync_cloud = LaunchConfiguration('lio_relay_sync_tf_cloud_topic').perform(context).strip()
    relay_yaw_raw = LaunchConfiguration('lio_relay_body_to_base_yaw_deg').perform(context).strip()
    try:
        relay_body_to_base_yaw = float(relay_yaw_raw) if relay_yaw_raw else 0.0
    except ValueError:
        relay_body_to_base_yaw = 0.0

    p_fastlio = os.path.join(get_package_share_directory('fast_lio'), fastlio_rel)
    p_overlay = os.path.join(get_package_share_directory('lio_bringup'), overlay_rel)
    plist: list = [p_fastlio, p_overlay]
    if bag_rel:
        plist.append(
            os.path.join(
                get_package_share_directory('ros_project_bringup'),
                bag_rel.lstrip('/'),
            )
        )
    plist.append({'use_sim_time': use_sim})

    relay_params: dict = {
        'use_sim_time': use_sim,
        'publish_tf': relay_publish_tf,
        'body_to_base_yaw_deg': relay_body_to_base_yaw,
    }
    if sync_cloud:
        relay_params['sync_tf_cloud_topic'] = sync_cloud

    return [
        Node(
            package='fast_lio',
            executable='fastlio_mapping',
            name='fastlio_mapping',
            output='screen',
            parameters=plist,
        ),
        Node(
            package='lio_bringup',
            executable='lio_odom_relay_node',
            name='lio_odom_relay_node',
            output='screen',
            parameters=[relay_params],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'fastlio_params_file',
                default_value='config/mid360.yaml',
                description='Path relative to fast_lio package share',
            ),
            DeclareLaunchArgument(
                'lio_overlay_params_file',
                default_value='config/fastlio_mid360_overlay.yaml',
                description='Path relative to lio_bringup package share',
            ),
            DeclareLaunchArgument(
                'lio_bag_overlay_params_file',
                default_value='',
                description=(
                    'Optional path relative to ros_project_bringup share; merged after '
                    'lio_overlay (e.g. config/fastlio_bag_replay_overlay.yaml). Empty = skip.'
                ),
            ),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                description='Follow /clock when true (bag). launch_slam passes this explicitly.',
            ),
            DeclareLaunchArgument(
                'lio_relay_publish_tf',
                default_value='false',
                description=(
                    'If true, lio_odom_relay_node broadcasts TF odom->base_link. Sparse vs EKF; '
                    'pair with ekf publish_tf false if you want relay as sole TF publisher.'
                ),
            ),
            DeclareLaunchArgument(
                'lio_relay_sync_tf_cloud_topic',
                default_value='',
                description=(
                    'If non-empty and publish_tf true, also publish odom->base_link TF at each '
                    'point cloud stamp (e.g. /livox/lidar) using last LIO pose.'
                ),
            ),
            DeclareLaunchArgument(
                'lio_relay_body_to_base_yaw_deg',
                default_value='0.0',
                description=(
                    'Planar yaw (deg) applied in lio_odom_relay: FAST-LIO child ``body`` -> robot '
                    '``base_link``. Use 180 if /ekf/path +X is aft while driving forward.'
                ),
            ),
            OpaqueFunction(function=_lio_backend_setup),
        ]
    )
