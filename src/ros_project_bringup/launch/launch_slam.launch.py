#!/usr/bin/env python3
"""
Bringup: Livox (optional) + static TF + EKF + optional NDT **or** optional FAST-LIO
(LIO) + optional keyframe scan map + optional RViz.

**Split (front vs back):** **Front-end (fast)** — IMU + LiDAR odom (NDT or LIO relay) + **ekf_node**.
**Back-end (slow)** — keyframe merged map, optional loop detection, optional **pose_graph_node**
(map optimisation on keyframes + loop pairs).

Default odometry is **FAST-LIO** + Python EKF (``use_lio: true``, ``use_lidar_fusion: false`` in
``slam_bringup.yaml``). For **PCL NDT** instead, set **``use_lio: false``** and **``use_lidar_fusion: true``**
(or pass ``use_lio:=false`` ``use_lidar_fusion:=true``, or merge ``config/livox_ndt_bag_axes_overlay.yaml``).
**``use_lio``** wins if both **``use_lidar_fusion``** and **``use_lio``** are true.

--- use_lio:=true — what changes (memory aid) ---
Replaced: **lidar_odometry_node** (NDT) is NOT started; it no longer publishes
**/lidar/odom** or **/lidar/relative_motion**.
Added: **fastlio_mapping** + **lio_odom_relay_node** (**/Odometry** → **/lidar/odom**
with **odom**→**base_link** headers). Default **lio_relay_publish_tf** is **false** (EKF publishes
dense **odom**→**base_link** TF for keyframe interpolation). Set **lio_relay_publish_tf** true and
**ekf_publish_tf_when_lio** false only if you want the relay as sole TF authority (sparse ~scan rate).
Unchanged: Livox driver, static TFs (**base_link**→**livox_frame**); **map**→**odom** is
static identity unless **pose_graph_publish_map_odom_tf** is true with **start_pose_graph**
(then pose_graph_node publishes **map**→**odom**).
**ekf_node** (still subscribes **/lidar/odom** when LIO or NDT path is on),
**keyframe_map_node**, RViz. **use_lidar_fusion** means “start NDT”; with **use_lio**,
NDT is skipped even if **use_lidar_fusion** is true. IMU-only: both flags false.
EKF **lidar_fuse_z_from_odom** in ``slam_bringup.yaml``: **``auto``** (true when LIO, false
for NDT), **``true``**, or **``false``**. Tune LIO in
**lio_bringup/config/fastlio_mid360_overlay.yaml** (``lio_overlay_params_file`` in YAML),
or the FAST_LIO base file.

**Configuration (primary):** **``config/slam_bringup.yaml``** in ``ros_project_bringup`` (see header
in that file). Optional: **``bringup_config``** launch arg or **``ROS_PROJECT_SLAM_CONFIG``** env
points to a *partial* YAML (merged over the installed default — convenient in Docker).

**``launch_sensors``** (launch argument, default ``false``): when ``false``, this launch does **not**
start Livox / Microstrain drivers. Sensors and (usually) ``/tf_static`` come from **robot bringup** on
the same machine or over DDS (same ``ROS_DOMAIN_ID``). Set ``launch_sensors:=true`` to start drivers
here (e.g. all-in-one dev machine).

**Microstrain IMU origin** (``slam_bringup.yaml`` ``microstrain_imu_origin``): ``robot`` = subscribe to
``microstrain_imu_topic`` only (typical with ``launch_sensors:=false``). ``local`` = start the serial
driver on **this** host when ``launch_sensors`` is true.

**``use_sim_time``** (launch argument, default ``false``): wall clock for live robot. Bag replay:
``use_sim_time:=true`` then ``ros2 bag play <bag> --clock``.

**``start_rviz``** (launch arg, optional): non-empty overrides YAML ``start_rviz`` (default **false** —
run RViz on another PC on the same domain).

**Single command (live SLAM, typical robot + second PC for RViz):**
  ``ros2 launch ros_project_bringup launch_slam.launch.py``

**Robot with local USB sensors in this launch:** ``ros2 launch ros_project_bringup launch_slam.launch.py launch_sensors:=true``
  (and ``microstrain_imu_origin: local`` in YAML on that machine).

**Bag replay:** ``ros2 launch ros_project_bringup launch_slam.launch.py use_sim_time:=true`` then play the bag.
If the bag lacks ``/tf_static``, use a bringup overlay with ``publish_robot_static_tf_when_sensors_off: true``
(and often ``publish_livox_imu_sensor_frame_tf: true``).

**Start order (bag replay):** launch **``launch_slam`` first**, wait until nodes (and RViz if used) are up,
then start ``ros2 bag play ... --clock``. If the bag runs **first**, ``/clock`` advances while the stack is
not subscribed; the first scans after bringup can align NDT / keyframe with a **wrong initial yaw**
(**mirrored or flipped map** vs starting together from t=0).

**Default installed YAML / assets** (paths set in ``config/slam_bringup.yaml``):

| Node / stack | Package | Default file (under ``share/<pkg>/``) |
|--------------|---------|--------------------------------------|
| **All bringup** | ``ros_project_bringup`` | ``config/slam_bringup.yaml`` (``bringup_config`` or ``ROS_PROJECT_SLAM_CONFIG``) |
| **ekf_node** | ``localisation_ekf`` | ``ekf_params_yaml`` in bringup (e.g. ``config/ekf_python.yaml``) |
| **lidar_odometry_node** (NDT) | (params from bringup) | see ``slam_bringup`` keys ``lidar_*`` |
| **fastlio_mapping** | ``fast_lio`` | ``fastlio_params_file`` in bringup |
| **FAST-LIO overlay** | ``lio_bringup`` | ``lio_overlay_params_file`` in bringup |
| **keyframe_map_node** | ``keyframe_scan_map`` | ``keyframe_params_yaml`` + per-keyframe ``keyframe_*`` in bringup |
| **pose_graph_node** | ``keyframe_scan_map`` | ``pose_graph_params_yaml`` + ``pose_graph_*`` in bringup |
| **Livox driver** | ``livox_ros_driver2`` | ``livox_*`` in bringup + ``MID360_config.json`` (or ``livox_config_path``) |
| **RViz** | ``ros_project_bringup`` | ``rviz_config_yaml`` in bringup |
| **Microstrain GX5-25** | ``microstrain_inertial_driver`` (apt / source) | Port/baud/rates: ``use_microstrain_imu`` and ``microstrain_*`` in bringup; driver loads ``params.yml``; see README |

**GX5-25 + Livox LiDAR:** set ``use_microstrain_imu: true`` and ``ekf_params_yaml`` to
``config/ekf_python_gx5_microstrain.yaml`` in bringup. See README §1.3.
"""
from __future__ import annotations

import math
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node, SetUseSimTime
from launch_ros.substitutions import FindPackageShare


# =============================================================================
# Config: `config/slam_bringup.yaml` — loaded at launch (see _load_slam_bringup_config)
# =============================================================================


def _resolve_slam_bringup_overlay_path(context) -> str:
    """Path to a YAML file merged over installed defaults: ``slam_bringup`` mapping."""
    cfg_arg = LaunchConfiguration('bringup_config').perform(context).strip()
    if cfg_arg:
        p = os.path.abspath(os.path.expanduser(cfg_arg))
    else:
        envp = os.environ.get('ROS_PROJECT_SLAM_CONFIG', '').strip()
        p = os.path.abspath(os.path.expanduser(envp)) if envp else ''
    if not p:
        p = os.path.join(
            get_package_share_directory('ros_project_bringup'),
            'config',
            'slam_bringup.yaml',
        )
    if not os.path.isfile(p):
        raise FileNotFoundError(
            f'Bringup config not found: {p}. Set bringup_config:=/path or ROS_PROJECT_SLAM_CONFIG.'
        )
    return p


def _load_slam_bringup_config(overlay_path: str) -> dict:
    """If ``overlay_path`` is the same file as the installed default, return it once. Otherwise
    deep-merge: installed ``slam_bringup`` dict then overlay (partial files supported)."""
    default_path = os.path.join(
        get_package_share_directory('ros_project_bringup'),
        'config',
        'slam_bringup.yaml',
    )
    with open(default_path, encoding='utf-8') as f:
        base_raw = yaml.safe_load(f) or {}
    if not isinstance(base_raw, dict):
        raise ValueError(f'{default_path} must be a mapping')
    if 'slam_bringup' in base_raw and isinstance(base_raw['slam_bringup'], dict):
        base = dict(base_raw['slam_bringup'])
    else:
        base = dict(base_raw)
    if os.path.normpath(overlay_path) == os.path.normpath(default_path):
        return base
    with open(overlay_path, encoding='utf-8') as f:
        ovr_raw = yaml.safe_load(f) or {}
    if not isinstance(ovr_raw, dict):
        ovr = {}
    elif 'slam_bringup' in ovr_raw and isinstance(ovr_raw['slam_bringup'], dict):
        ovr = ovr_raw['slam_bringup']
    else:
        ovr = ovr_raw
    if not isinstance(ovr, dict):
        ovr = {}
    return {**base, **dict(ovr)}


def _livox_launch_arg_pairs(user: dict) -> list:
    lcp = str(user.get('livox_config_path', '') or '').strip()
    if lcp:
        ucfg = os.path.abspath(os.path.expanduser(lcp))
    else:
        lpkg = str(user.get('livox_launch_package', 'livox_ros_driver2'))
        lrel = str(
            user.get('livox_config_relpath', 'config/MID360_config.json')
        ).strip().lstrip('/')
        ucfg = _share_file(lpkg, lrel)
    if not os.path.isfile(ucfg):
        raise FileNotFoundError(
            f'Livox JSON not found: {ucfg} (set livox_config_path or livox_config_relpath in bringup)'
        )
    frame_id = str(user.get('livox_cloud_frame_id', 'livox_frame') or 'livox_frame')
    return [
        ('user_config', TextSubstitution(text=ucfg)),
        ('frame_id', TextSubstitution(text=frame_id)),
        ('xfer_format', TextSubstitution(text=str(int(user.get('livox_xfer_format', 0))))),
        ('multi_topic', TextSubstitution(text=str(int(user.get('livox_multi_topic', 0))))),
        ('data_src', TextSubstitution(text=str(int(user.get('livox_data_src', 0))))),
        (
            'publish_freq',
            TextSubstitution(text=str(float(user.get('livox_publish_freq', 10.0)))),
        ),
        (
            'output_type',
            TextSubstitution(text=str(int(user.get('livox_output_type', 0)))),
        ),
        (
            'cmdline_bd_code',
            TextSubstitution(
                text=str(
                    user.get('livox_cmdline_bd_code', 'livox0000000001')
                )
            ),
        ),
    ]


def _share_file(pkg: str, rel: str) -> str:
    rel = str(rel).strip().lstrip('/')
    return os.path.join(get_package_share_directory(pkg), rel)


def _rviz_config_with_keyframe_dot_overrides(
    rviz_cfg_path: str,
    *,
    keyframe_pixels: str,
    keyframe_size_m: str,
) -> str:
    """Return base rviz config path or a temp config with Keyframe map point size overrides."""
    px_raw = str(keyframe_pixels or '').strip()
    m_raw = str(keyframe_size_m or '').strip()
    if not px_raw and not m_raw:
        return rviz_cfg_path
    with open(rviz_cfg_path, encoding='utf-8') as f:
        cfg = yaml.safe_load(f) or {}
    viz = cfg.get('Visualization Manager')
    if not isinstance(viz, dict):
        return rviz_cfg_path
    displays = viz.get('Displays')
    if not isinstance(displays, list):
        return rviz_cfg_path
    target = None
    for d in displays:
        if (
            isinstance(d, dict)
            and d.get('Name') == 'Keyframe map'
            and d.get('Class') == 'rviz_default_plugins/PointCloud2'
        ):
            target = d
            break
    if target is None:
        return rviz_cfg_path
    if px_raw:
        target['Size (Pixels)'] = max(1, int(float(px_raw)))
    if m_raw:
        target['Size (m)'] = max(0.001, float(m_raw))
    fd, tmp_path = tempfile.mkstemp(prefix='slam_rviz_', suffix='.rviz')
    os.close(fd)
    with open(tmp_path, 'w', encoding='utf-8') as f:
        yaml.safe_dump(cfg, f, sort_keys=False)
    return tmp_path


def launch_setup(context, *args, **kwargs):
    U = _load_slam_bringup_config(
        _resolve_slam_bringup_overlay_path(context)
    )
    # CLI overrides (empty = keep value from slam_bringup YAML merge).
    _lio_arg = LaunchConfiguration('use_lio').perform(context).strip().lower()
    if _lio_arg not in ('', 'auto', 'use_yaml', '__yaml__'):
        U['use_lio'] = _lio_arg in ('true', '1', 'yes', 'on')
    _lidar_arg = LaunchConfiguration('use_lidar_fusion').perform(context).strip().lower()
    if _lidar_arg not in ('', 'auto', 'use_yaml', '__yaml__'):
        U['use_lidar_fusion'] = _lidar_arg in ('true', '1', 'yes', 'on')
    _srv = LaunchConfiguration('start_rviz').perform(context).strip().lower()
    if _srv not in ('', 'auto', 'use_yaml', '__yaml__'):
        U['start_rviz'] = _srv in ('true', '1', 'yes', 'on')
    _skf = LaunchConfiguration('start_keyframe_map').perform(context).strip().lower()
    if _skf not in ('', 'auto', 'use_yaml', '__yaml__'):
        U['start_keyframe_map'] = _skf in ('true', '1', 'yes', 'on')
    _spg = LaunchConfiguration('start_pose_graph').perform(context).strip().lower()
    if _spg not in ('', 'auto', 'use_yaml', '__yaml__'):
        U['start_pose_graph'] = _spg in ('true', '1', 'yes', 'on')

    ls = LaunchConfiguration('launch_sensors').perform(context).strip().lower()
    launch_sensors = ls not in ('false', '0', 'no', 'off')

    ust = LaunchConfiguration('use_sim_time').perform(context).strip().lower()
    use_sim_time = ust not in ('false', '0', 'no', 'off')
    sim_time_param = {'use_sim_time': use_sim_time}

    use_lidar = bool(U['use_lidar_fusion'])
    use_lio = bool(U['use_lio'])
    start_ndt = use_lidar and not use_lio
    ekf_use_lidar = use_lidar or use_lio
    fuse_z_raw = str(U['ekf_lidar_fuse_z_from_odom']).strip().lower()
    if fuse_z_raw in ('auto', ''):
        lidar_fuse_z_from_odom = use_lio
    else:
        lidar_fuse_z_from_odom = fuse_z_raw in ('true', '1', 'yes')

    lio_relay_publish_tf = bool(U.get('lio_relay_publish_tf', True))
    lio_relay_sync_tf_cloud_topic = str(
        U.get('lio_relay_sync_tf_cloud_topic', '') or ''
    ).strip()
    if (
        use_lio
        and lio_relay_publish_tf
        and not lio_relay_sync_tf_cloud_topic
        and bool(U.get('lio_relay_auto_sync_tf_cloud', True))
    ):
        lio_relay_sync_tf_cloud_topic = str(
            U.get('keyframe_cloud_topic', '/livox/lidar') or ''
        ).strip()
    lio_relay_body_to_base_yaw_deg = float(U.get('lio_relay_body_to_base_yaw_deg', 0.0) or 0.0)
    if use_lio and lio_relay_publish_tf:
        ekf_publish_tf_effective = bool(U.get('ekf_publish_tf_when_lio', False))
    else:
        ekf_publish_tf_effective = bool(U['ekf_publish_tf'])

    start_rviz = bool(U['start_rviz'])
    rviz_kf_px = LaunchConfiguration('rviz_keyframe_map_size_pixels').perform(context).strip()
    rviz_kf_m = LaunchConfiguration('rviz_keyframe_map_size_m').perform(context).strip()
    start_livox = bool(U['start_livox_driver']) and launch_sensors
    start_keyframe_map = bool(U['start_keyframe_map'])
    start_pose_graph = bool(U['start_pose_graph'])
    pose_graph_pub_tf = bool(U['pose_graph_publish_map_odom_tf'])
    kf_apply_pg = bool(U['keyframe_apply_pose_graph_map'])
    if (
        start_keyframe_map
        and start_pose_graph
        and kf_apply_pg
        and pose_graph_pub_tf
    ):
        print(
            '[launch_slam] WARNING: keyframe_apply_pose_graph_map and '
            'pose_graph_publish_map_odom_tf are both true — double correction '
            '(map cloud rebuild + dynamic map→odom). Prefer one: either '
            'keyframe_apply_pose_graph_map only, or pose_graph_publish_map_odom_tf only.'
        )

    use_microstrain_imu = bool(U['use_microstrain_imu'])
    _imu_origin = str(U.get('microstrain_imu_origin', 'robot') or 'robot').strip().lower()
    _imu_origin_local = _imu_origin in (
        'local', 'laptop', 'usb', 'serial', 'this', 'this_machine', 'onboard', 'host',
    )
    _imu_origin_robot = _imu_origin in (
        'robot', 'remote', 'dds', 'network', 'bag', 'external',
    )
    if not _imu_origin_local and not _imu_origin_robot:
        raise ValueError(
            f'slam_bringup microstrain_imu_origin must be local|robot (got {_imu_origin!r})'
        )
    start_microstrain_driver = (
        use_microstrain_imu and launch_sensors and _imu_origin_local
    )
    # Mount TF: with local USB driver, publish mount here. With ``launch_sensors:=false`` (bag / DDS laptop),
    # many bags omit ``base_link``→``imu_link``; optional publish from ``imu_mount_*`` (see
    # ``publish_robot_static_tf_when_sensors_off``). For ``robot`` IMU on a live robot that already
    # publishes this static TF on DDS, set that key false to avoid duplicates.
    _robot_static_when_sensors_off = bool(
        U.get('publish_robot_static_tf_when_sensors_off', True)
    )
    publish_imu_mount_tf = use_microstrain_imu and (
        (launch_sensors and start_microstrain_driver)
        or (
            (not launch_sensors)
            and _imu_origin_robot
            and _robot_static_when_sensors_off
        )
    )

    lx = float(U['livox_extrinsic_x'])
    ly = float(U['livox_extrinsic_y'])
    lz = float(U['livox_extrinsic_z'])
    lr = float(U['livox_extrinsic_roll_deg'])
    lp = float(U['livox_extrinsic_pitch_deg'])
    lyaw = float(U['livox_extrinsic_yaw_deg'])
    roll_rad = math.radians(lr)
    pitch_rad = math.radians(lp)
    yaw_rad = math.radians(lyaw)

    # map ≡ odom: explicit identity so map→odom is clearly valid (odometry lives in odom; map aligned until pose_graph Option 3).
    # Static TF often shows a huge “rate” in tf2_monitor; that is normal, not “missing data”.
    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        parameters=[sim_time_param],
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'map',
            '--child-frame-id', 'odom',
        ],
    )

    base_to_livox = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_livox',
        parameters=[sim_time_param],
        arguments=[
            '--x', str(lx), '--y', str(ly), '--z', str(lz),
            '--roll', str(roll_rad),
            '--pitch', str(pitch_rad),
            '--yaw', str(yaw_rad),
            '--frame-id', 'base_link',
            '--child-frame-id', 'livox_frame',
        ],
    )

    imu_px = float(U['imu_mount_x'])
    imu_py = float(U['imu_mount_y'])
    imu_pz = float(U['imu_mount_z'])
    imu_rr = float(U['imu_mount_roll_deg'])
    imu_rp = float(U['imu_mount_pitch_deg'])
    imu_ry = float(U['imu_mount_yaw_deg'])
    imu_parent = str(U['imu_mount_parent_frame']).strip()
    imu_child = str(U['imu_mount_child_frame']).strip()
    base_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_imu_link',
        parameters=[sim_time_param],
        arguments=[
            '--x', str(imu_px), '--y', str(imu_py), '--z', str(imu_pz),
            '--roll', str(math.radians(imu_rr)),
            '--pitch', str(math.radians(imu_rp)),
            '--yaw', str(math.radians(imu_ry)),
            '--frame-id', imu_parent,
            '--child-frame-id', imu_child,
        ],
    )

    livox_cloud_frame = str(U.get('livox_cloud_frame_id', 'livox_frame') or 'livox_frame').strip()
    livox_imu_child = str(U.get('livox_imu_child_frame', 'livox_imu') or 'livox_imu').strip()
    lxi_r = float(U.get('livox_imu_bridge_roll_deg', 0.0) or 0.0)
    lxi_p = float(U.get('livox_imu_bridge_pitch_deg', 0.0) or 0.0)
    lxi_y = float(U.get('livox_imu_bridge_yaw_deg', 0.0) or 0.0)
    # Livox IMU chip vs cloud frame when the driver is not on this host. Child must NOT be
    # `sensor` if Microstrain uses that frame_id — it would steal TF from the GX5 (see slam_bringup header).
    livox_frame_to_imu_bridge = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='livox_frame_to_livox_imu_bridge',
        parameters=[sim_time_param],
        arguments=[
            '--x',
            '0',
            '--y',
            '0',
            '--z',
            '0',
            '--roll',
            str(math.radians(lxi_r)),
            '--pitch',
            str(math.radians(lxi_p)),
            '--yaw',
            str(math.radians(lxi_y)),
            '--frame-id',
            livox_cloud_frame,
            '--child-frame-id',
            livox_imu_child,
        ],
    )

    ms_imu_sensor_child = str(
        U.get('microstrain_imu_sensor_child_frame', 'sensor') or 'sensor'
    ).strip()
    publish_ms_imu_sensor_tf = bool(
        U.get('publish_microstrain_imu_sensor_frame_tf', True)
    )
    imu_link_to_sensor_bridge = None
    if (
        use_microstrain_imu
        and publish_ms_imu_sensor_tf
        and ms_imu_sensor_child
        and (not launch_sensors)
        and (
            (not publish_imu_mount_tf)
            or ((not launch_sensors) and _robot_static_when_sensors_off)
        )
        and imu_child.lower() != ms_imu_sensor_child.lower()
        and livox_imu_child.lower() != ms_imu_sensor_child.lower()
    ):
        ms_alias_r = float(U.get('microstrain_sensor_alias_roll_deg', 0.0) or 0.0)
        ms_alias_p = float(U.get('microstrain_sensor_alias_pitch_deg', 0.0) or 0.0)
        ms_alias_y = float(U.get('microstrain_sensor_alias_yaw_deg', 0.0) or 0.0)
        imu_link_to_sensor_bridge = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_link_to_microstrain_sensor_alias',
            parameters=[sim_time_param],
            arguments=[
                '--x',
                '0',
                '--y',
                '0',
                '--z',
                '0',
                '--roll',
                str(math.radians(ms_alias_r)),
                '--pitch',
                str(math.radians(ms_alias_p)),
                '--yaw',
                str(math.radians(ms_alias_y)),
                '--frame-id',
                imu_child,
                '--child-frame-id',
                ms_imu_sensor_child,
            ],
        )

    microstrain_node = None
    if start_microstrain_driver:
        ms_share = get_package_share_directory('microstrain_inertial_driver')
        default_ms = os.path.join(
            ms_share,
            'microstrain_inertial_driver_common',
            'config',
            'params.yml',
        )
        if not os.path.isfile(default_ms):
            raise FileNotFoundError(
                f'microstrain_inertial_driver missing {default_ms}. '
                'Install e.g. `sudo apt install ros-humble-microstrain-inertial-driver` '
                '(replace humble with your distro) or build LORD-MicroStrain/microstrain_inertial.'
            )
        with open(default_ms, encoding='utf-8') as f:
            ms_params = yaml.safe_load(f)
        overlay_rel = str(U.get('microstrain_params_overlay', '') or '').strip()
        if overlay_rel:
            overlay_path = (
                overlay_rel
                if os.path.isabs(overlay_rel)
                else _share_file('ros_project_bringup', overlay_rel.lstrip('/'))
            )
            if not os.path.isfile(overlay_path):
                raise FileNotFoundError(
                    f'slam_bringup microstrain_params_overlay not found: {overlay_path}'
                )
            with open(overlay_path, encoding='utf-8') as f:
                ovr = yaml.safe_load(f) or {}
            if not isinstance(ovr, dict):
                raise ValueError('microstrain_params_overlay must be a YAML mapping (flat keys)')
            for k, v in ovr.items():
                ms_params[k] = v
        port = str(U['microstrain_port']).strip()
        baud = int(U['microstrain_baud'])
        ms_params['port'] = port
        ms_params['baudrate'] = baud
        _ms_imu_frame = str(U['microstrain_frame_id']).strip()
        # Vendor params often default imu_frame_id to "sensor"; Imu.header may use this, not frame_id.
        ms_params['frame_id'] = _ms_imu_frame
        ms_params['imu_frame_id'] = _ms_imu_frame
        ms_params['mount_frame_id'] = str(U['microstrain_mount_frame_id']).strip()
        ms_params['publish_mount_to_frame_id_transform'] = False
        ms_params['imu_data_raw_rate'] = int(U['microstrain_imu_data_raw_rate'])
        ms_params['imu_data_rate'] = int(U['microstrain_imu_data_rate'])
        microstrain_node = Node(
            package='microstrain_inertial_driver',
            executable='microstrain_inertial_driver_node',
            name='microstrain_inertial_driver',
            output='screen',
            parameters=[ms_params, sim_time_param],
        )

    livox_sim = 'true' if use_sim_time else 'false'
    livox_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare(U['livox_launch_package']),
                U['livox_launch_relpath'],
            ])
        ),
        launch_arguments=_livox_launch_arg_pairs(U)
        + [('use_sim_time', livox_sim)],
    )

    ekf_params = _share_file(U['ekf_params_pkg'], U['ekf_params_yaml'])
    keyframe_params = _share_file(U['keyframe_params_pkg'], U['keyframe_params_yaml'])

    ekf_lidar_odom = str(U['ekf_lidar_odom_topic']).strip()
    _delta_t = str(U.get('ekf_lidar_delta_topic', '') or '').strip()
    ekf_lidar_bridge = {
        'lidar_odom_topic': (ekf_lidar_odom if ekf_use_lidar else ''),
        'lidar_pose_topic': str(U['ekf_lidar_pose_topic']).strip(),
        'lidar_z_topic': str(U['ekf_lidar_z_topic']).strip(),
        'lidar_pose_var': float(U['ekf_lidar_pose_var']),
        'lidar_yaw_var': float(U['ekf_lidar_yaw_var']),
        'lidar_z_var': float(U['ekf_lidar_z_var']),
        'lidar_gate_nis': float(U['ekf_lidar_gate_nis']),
        'lidar_fuse_z_from_odom': lidar_fuse_z_from_odom,
        'lidar_require_frames': bool(U['ekf_lidar_require_frames']),
        'lidar_use_roll_pitch': bool(U['ekf_lidar_use_roll_pitch']),
        'lidar_delta_topic': (_delta_t if ekf_use_lidar else ''),
        'lidar_delta_vel_var': float(U.get('ekf_lidar_delta_vel_var', 0.22)),
        'lidar_delta_gate_nis': float(U.get('ekf_lidar_delta_gate_nis', 200.0)),
        'lidar_delta_nominal_dt_sec': float(U.get('ekf_lidar_delta_nominal_dt_sec', 0.1)),
        'lidar_fuse_slow_linear_speed_m_s': float(
            U.get('ekf_lidar_fuse_slow_linear_speed_m_s', 0.0) or 0.0
        ),
        'lidar_pose_var_below_slow_speed_scale': float(
            U.get('ekf_lidar_pose_var_below_slow_speed_scale', 25.0) or 25.0
        ),
    }

    imu_topic_effective = str(U['ekf_imu_topic']).strip()
    if use_microstrain_imu:
        imu_topic_effective = str(U['microstrain_imu_topic']).strip()

    lidar_cloud = str(U['lidar_cloud_topic']).strip()
    # Local Livox or same topic over DDS when sensors run elsewhere.
    lidar_stamp_topic = (
        lidar_cloud
        if (ekf_use_lidar and lidar_cloud and (start_livox or not launch_sensors))
        else ''
    )

    ekf_core_overrides = {
        'imu_topic': imu_topic_effective,
        'publish_topic': str(U['ekf_publish_topic']).strip(),
        'pose_topic': str(U['ekf_pose_topic']).strip(),
        'path_topic': str(U['ekf_path_topic']).strip(),
        'odom_frame': str(U['ekf_odom_frame']).strip(),
        'base_link_frame': str(U['ekf_base_link_frame']).strip(),
        'publish_tf': ekf_publish_tf_effective,
        'nominal_dt': float(U['ekf_nominal_dt']),
        'use_stamp_dt': bool(U['ekf_use_stamp_dt']),
        'imu_source_id': ('microstrain' if use_microstrain_imu else 'livox'),
        'imu_source_topic': str(U['ekf_imu_source_topic']).strip(),
        # Stamp sync (Microstrain vs Livox) + TF sample every cloud when NDT lags/skips.
        'lidar_cloud_stamp_topic': lidar_stamp_topic,
        'max_odom_tf_publish_rate_hz': float(U['ekf_max_odom_tf_publish_rate_hz']),
        'imu_stamp_offset_sec': float(U['ekf_imu_stamp_offset_sec']),
        'lidar_stamp_offset_sec': float(U['ekf_lidar_stamp_offset_sec']),
        'lidar_fusion_debug_log': bool(U.get('ekf_lidar_fusion_debug_log', False)),
        'lidar_fusion_debug_throttle_sec': float(
            U.get('ekf_lidar_fusion_debug_throttle_sec', 1.0)
        ),
        'lidar_soft_fuse_after_gate_reject': bool(
            U.get('ekf_lidar_soft_fuse_after_gate_reject', True)
        ),
        'lidar_fuse_xy_only': bool(U.get('ekf_lidar_fuse_xy_only', False)),
        'imu_gyro_z_bias_rad_s': float(U.get('ekf_imu_gyro_z_bias_rad_s', 0.0) or 0.0),
        'imu_auto_gyro_z_bias_enable': bool(
            U.get('ekf_imu_auto_gyro_z_bias_enable', False)
        ),
        'imu_auto_gyro_z_bias_tune_sec': float(
            U.get('ekf_imu_auto_gyro_z_bias_tune_sec', 4.0) or 4.0
        ),
        'imu_gyro_z_scale': float(U.get('ekf_imu_gyro_z_scale', 1.0) or 1.0),
        'publish_base_link_yaw_offset_deg': float(
            U.get('ekf_publish_base_link_yaw_offset_deg', 0.0) or 0.0
        ),
    }
    if U.get('ekf_predict_use_linear_accel') is not None:
        ekf_core_overrides['predict_use_linear_accel'] = bool(
            U['ekf_predict_use_linear_accel']
        )

    lidar_odom_node = None
    if start_ndt:
        voxel = float(U['lidar_voxel_leaf_size'])
        crop = float(U['lidar_crop_range_m'])
        res = float(U['lidar_ndt_resolution'])
        max_it = int(U['lidar_ndt_max_iterations'])
        max_fit = float(U['lidar_max_fitness_score'])
        reg_mode = str(U['lidar_registration_mode']).strip()
        map_merge = float(U['lidar_map_merge_voxel_leaf_size'])
        map_max_pts = int(U['lidar_map_max_points'])
        lidar_step = float(U['lidar_ndt_step_size'])
        lidar_eps = float(U['lidar_ndt_transformation_epsilon'])
        lidar_min_pts = int(U['lidar_min_points_per_cloud'])
        lidar_pub_tf = bool(U['lidar_publish_tf'])
        lidar_use_tf_guess = bool(U['lidar_use_tf_initial_guess'])
        lidar_tf_timeout = float(U['lidar_tf_initial_guess_timeout_sec'])
        _smooth_lidar = bool(U.get('lidar_odom_smooth_enable', False))
        _pub_lidar = str(U['lidar_odom_topic']).strip()
        _raw_lidar = str(U.get('lidar_odom_raw_topic', '/lidar/odom_raw') or '/lidar/odom_raw').strip()
        _ndt_odom_out = _raw_lidar if _smooth_lidar else _pub_lidar
        lidar_params = {
            'cloud_topic': str(U['lidar_cloud_topic']).strip(),
            'odom_topic': _ndt_odom_out,
            'delta_topic': str(U['lidar_delta_topic']).strip(),
            'pose_correction_topic': str(U['lidar_pose_correction_topic']).strip(),
            'odom_frame': str(U['lidar_odom_frame']).strip(),
            'base_frame': str(U['lidar_base_frame']).strip(),
            'registration_mode': reg_mode,
            'voxel_leaf_size': voxel,
            'crop_range_m': crop,
            'ndt_resolution': res,
            'ndt_coarse_resolution': float(U.get('lidar_ndt_coarse_resolution', 0.0)),
            'ndt_step_size': lidar_step,
            'ndt_transformation_epsilon': lidar_eps,
            'ndt_voxel_min_points': int(U.get('lidar_ndt_voxel_min_points', 10)),
            'ndt_voxel_cov_eig_inflation_ratio': float(
                U.get('lidar_ndt_voxel_cov_eig_inflation_ratio', 0.05)
            ),
            'ndt_max_iterations': max_it,
            'max_fitness_score': max_fit,
            'min_points_per_cloud': lidar_min_pts,
            'publish_tf': lidar_pub_tf,
            'map_max_points': map_max_pts,
            'use_tf_initial_guess': lidar_use_tf_guess,
            'tf_initial_guess_timeout_sec': lidar_tf_timeout,
            'log_ndt_relative': bool(U.get('lidar_log_ndt_relative', False)),
            'log_registration_debug': bool(U.get('lidar_log_registration_debug', False)),
            'log_accumulated_pose': bool(U.get('lidar_log_accumulated_pose', False)),
        }
        if map_merge > 0.0:
            lidar_params['map_merge_voxel_leaf_size'] = map_merge
        lidar_params['scan_to_map_map_refresh_period'] = int(
            U.get('lidar_scan_to_map_map_refresh_period', 0)
        )
        lidar_params['scan_to_map_refresh_keep_scans'] = int(
            U.get('lidar_scan_to_map_refresh_keep_scans', 3)
        )
        lidar_params['scan_to_map_register_sensor_frame'] = bool(
            U.get('lidar_scan_to_map_register_sensor_frame', False)
        )
        lidar_params['ndt_fuse_prior_planar_yaw'] = bool(
            U.get('lidar_ndt_fuse_prior_planar_yaw', False)
        )
        lidar_params['ndt_prior_yaw_blend'] = float(U.get('lidar_ndt_prior_yaw_blend', 0.85))
        lidar_params['ndt_corridor_degeneracy_check'] = bool(
            U.get('lidar_ndt_corridor_degeneracy_check', True)
        )
        lidar_params['ndt_corridor_spin_yaw_min_rad'] = float(
            U.get('lidar_ndt_corridor_spin_yaw_min_rad', 0.5)
        )
        lidar_params['ndt_corridor_spin_max_corr_xy_m'] = float(
            U.get('lidar_ndt_corridor_spin_max_corr_xy_m', 0.1)
        )
        lidar_params['ndt_fallback_if_planar_correction_below_m'] = float(
            U.get('lidar_ndt_fallback_if_planar_correction_below_m', 0.0)
        )
        lidar_params['ndt_reject_opposite_ekf_step'] = bool(
            U.get('lidar_ndt_reject_opposite_ekf_step', False)
        )
        lidar_params['ndt_gate_until_prior_translation_m'] = float(
            U.get('lidar_ndt_gate_until_prior_translation_m', 0.0)
        )
        lidar_params['ndt_gate_force_after_sec'] = float(
            U.get('lidar_ndt_gate_force_after_sec', 0.0)
        )
        lidar_params['ndt_opposite_motion_min_ekf_step_m'] = float(
            U.get('lidar_ndt_opposite_motion_min_ekf_step_m', 0.05)
        )
        lidar_params['ndt_opposite_motion_min_ndt_step_m'] = float(
            U.get('lidar_ndt_opposite_motion_min_ndt_step_m', 0.05)
        )
        lidar_params['map_merge_keyframe_min_translation_m'] = float(
            U.get('lidar_map_merge_keyframe_min_translation_m', 0.0)
        )
        lidar_params['map_merge_keyframe_min_yaw_rad'] = math.radians(
            float(U.get('lidar_map_merge_keyframe_min_yaw_deg', 0.0))
        )
        lr = float(U['livox_extrinsic_roll_deg'])
        lp = float(U['livox_extrinsic_pitch_deg'])
        lyaw = float(U['livox_extrinsic_yaw_deg'])
        lidar_params['sensor_extrinsic_rpy_xyz'] = [
            math.radians(lr),
            math.radians(lp),
            math.radians(lyaw),
            lx,
            ly,
            lz,
        ]
        lidar_odom_node = Node(
            package='lidar_odometry',
            executable='lidar_odometry_node',
            name='lidar_odometry_node',
            output='screen',
            parameters=[lidar_params, sim_time_param],
        )

    ekf_node = Node(
        package='localisation_ekf',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[ekf_params, ekf_lidar_bridge, ekf_core_overrides, sim_time_param],
    )

    actions = []
    if start_livox:
        actions.append(livox_driver)
    if not (start_pose_graph and pose_graph_pub_tf):
        actions.append(map_to_odom)
    if start_microstrain_driver and microstrain_node is not None:
        actions.append(microstrain_node)
    if publish_imu_mount_tf:
        actions.append(base_to_imu)
    # Livox extrinsic static TF: always when local driver; when sensors off, publish from YAML if enabled
    # (bag replay often has no ``base_link``→``livox_frame`` on /tf_static).
    if launch_sensors or ((not launch_sensors) and _robot_static_when_sensors_off):
        actions.append(base_to_livox)
    pub_livox_sensor_tf = bool(U.get('publish_livox_imu_sensor_frame_tf', True))
    if pub_livox_sensor_tf and (not start_livox):
        actions.append(livox_frame_to_imu_bridge)
    if imu_link_to_sensor_bridge is not None:
        actions.append(imu_link_to_sensor_bridge)
    # EKF must publish odom->base_link before NDT uses it as scan_to_map initial guess (avoid first-cloud fallback).
    _ekf_delay = float(U.get('ekf_node_start_delay_sec', 0.0) or 0.0)
    if _ekf_delay > 0.0:
        actions.append(TimerAction(period=_ekf_delay, actions=[ekf_node]))
    else:
        actions.append(ekf_node)

    # EMA low-pass: NDT publishes raw; this republishes smoothed to ``lidar_odom_topic`` for EKF/tools.
    _smooth_lidar = bool(U.get('lidar_odom_smooth_enable', False))
    if lidar_odom_node is not None and _smooth_lidar:
        _pub_lidar = str(U['lidar_odom_topic']).strip()
        _raw_lidar = str(U.get('lidar_odom_raw_topic', '/lidar/odom_raw') or '/lidar/odom_raw').strip()
        _mode = str(U.get('lidar_odom_smooth_mode', 'full') or 'full').strip().lower()
        _ap = float(U.get('lidar_odom_smooth_alpha_pose', 0.18) or 0.18)
        _at = float(U.get('lidar_odom_smooth_alpha_twist_linear', 0.22) or 0.22)
        lidar_smooth_node = Node(
            package='ros_project_bringup',
            executable='lidar_odom_ema_smooth',
            name='lidar_odom_ema_smooth',
            output='screen',
            parameters=[
                {
                    'in_topic': _raw_lidar,
                    'out_topic': _pub_lidar,
                    'smooth_mode': _mode,
                    'alpha_pose': _ap,
                    'alpha_twist_linear': _at,
                },
                sim_time_param,
            ],
        )
        actions.append(lidar_smooth_node)

    if lidar_odom_node is not None:
        _ndt_delay = float(U.get('lidar_node_start_delay_sec', 0.0) or 0.0)
        if _ndt_delay > 0.0:
            actions.append(
                TimerAction(period=_ndt_delay, actions=[lidar_odom_node])
            )
        else:
            actions.append(lidar_odom_node)

    if use_lio:
        lio_sim = 'true' if use_sim_time else 'false'
        # Bag replay only: merge fastlio_bag_replay_overlay (lidar_type 0) when ``use_sim_time`` is true.
        # Do **not** merge on live ``launch_sensors:=false`` (Jetson + Livox on DDS) — that would override
        # MID360 ``lidar_type: 4`` and break preprocessing (log shows ``p_pre->lidar_type 0``).
        _lio_bag_overlay = ''
        if use_sim_time:
            _lio_bag_overlay = str(
                U.get('lio_bag_overlay_params_file', 'config/fastlio_bag_replay_overlay.yaml') or ''
            ).strip()
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('lio_bringup'),
                        'launch',
                        'lio_backend.launch.py',
                    ])
                ),
                launch_arguments=[
                    (
                        'fastlio_params_file',
                        TextSubstitution(text=str(U['fastlio_params_file'])),
                    ),
                    (
                        'lio_overlay_params_file',
                        TextSubstitution(text=str(U['lio_overlay_params_file'])),
                    ),
                    ('use_sim_time', lio_sim),
                    (
                        'lio_bag_overlay_params_file',
                        TextSubstitution(text=_lio_bag_overlay),
                    ),
                    (
                        'lio_relay_publish_tf',
                        'true' if lio_relay_publish_tf else 'false',
                    ),
                    (
                        'lio_relay_sync_tf_cloud_topic',
                        TextSubstitution(text=lio_relay_sync_tf_cloud_topic),
                    ),
                    (
                        'lio_relay_body_to_base_yaw_deg',
                        TextSubstitution(text=str(lio_relay_body_to_base_yaw_deg)),
                    ),
                ],
            )
        )
        # Connect FAST-LIO world (camera_init/body) to robot odom so recorders can compose body->base_link.
        if use_lio and (not launch_sensors) and bool(U.get('lio_auto_tf_bridge', True)):
            actions.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([
                            FindPackageShare('ros_project_bringup'),
                            'launch',
                            'tf_bridge_fastlio_odom_compare.launch.py',
                        ])
                    ),
                    launch_arguments=[
                        ('use_sim_time', lio_sim),
                        ('bridge_odom_to_map', 'false'),
                        ('bridge_body_to_base_link', 'false'),
                    ],
                )
            )

    if start_keyframe_map:
        kf_loop = bool(U['keyframe_loop_closure_enable'])
        kf_deskew_imu = str(U['keyframe_deskew_imu_topic']).strip()
        if use_microstrain_imu and bool(U['keyframe_deskew_imu_follow_ekf']):
            kf_deskew_imu = imu_topic_effective.strip()
        kf_rot_raw = str(
            U.get('keyframe_deskew_imu_rotate_gyro_to_frame', '') or ''
        ).strip()
        if kf_rot_raw.lower() in ('disabled', 'none', 'false', 'off'):
            kf_gyro_to_frame = ''
        elif kf_rot_raw:
            kf_gyro_to_frame = kf_rot_raw
        elif use_microstrain_imu and bool(U['keyframe_deskew_imu_follow_ekf']):
            kf_gyro_to_frame = str(U.get('livox_cloud_frame_id', 'livox_frame') or '').strip() or 'livox_frame'
        else:
            kf_gyro_to_frame = ''
        keyframe_overrides = {
            'cloud_topic': str(U['keyframe_cloud_topic']).strip(),
            'map_cloud_topic': str(U['keyframe_map_cloud_topic']).strip(),
            'keyframe_path_topic': str(U['keyframe_path_topic']).strip(),
            'map_frame': str(U['keyframe_map_frame']).strip(),
            'robot_frame': str(U['keyframe_robot_frame']).strip(),
            'keyframe_min_dist_m': float(U['keyframe_min_dist_m']),
            'keyframe_min_yaw_deg': float(U['keyframe_min_yaw_deg']),
            'keyframe_min_time_sec': float(U['keyframe_min_time_sec']),
            'voxel_leaf_m': float(U['keyframe_voxel_leaf_m']),
            'max_map_points': int(U['keyframe_max_map_points']),
            'max_pts_per_scan': int(U['keyframe_max_pts_per_scan']),
            'publish_keyframe_path': bool(U['keyframe_publish_path']),
            'loop_closure_enable': kf_loop,
            'loop_min_index_gap': int(U.get('keyframe_loop_min_index_gap', 28)),
            'loop_proximity_xy_m': float(U.get('keyframe_loop_proximity_xy_m', 5.0)),
            'loop_proximity_yaw_deg': float(U.get('keyframe_loop_proximity_yaw_deg', 35.0)),
            'loop_store_voxel_leaf_m': float(
                U.get('keyframe_loop_store_voxel_leaf_m', 0.42)
            ),
            'loop_max_stored_pts': int(U.get('keyframe_loop_max_stored_pts', 4500)),
            'loop_sample_points': int(U.get('keyframe_loop_sample_points', 450)),
            'loop_point_match_m': float(U.get('keyframe_loop_point_match_m', 0.38)),
            'loop_overlap_ratio': float(U.get('keyframe_loop_overlap_ratio', 0.32)),
            'loop_cooldown_sec': float(U.get('keyframe_loop_cooldown_sec', 6.0)),
            'apply_pose_graph_corrections': kf_apply_pg,
            'map_batch_store_voxel_m': float(
                U.get('keyframe_map_batch_store_voxel_m', 0.32) or 0.32
            ),
            'map_publish_min_interval_sec': float(
                U['keyframe_map_publish_min_interval_sec']
            ),
            'warmup_clouds_to_skip': int(
                U.get('keyframe_warmup_clouds_to_skip', 0) or 0
            ),
            'tf_allow_latest_fallback': bool(U['keyframe_tf_allow_latest_fallback']),
            'tf_future_extrapolation_use_latest': bool(
                U['keyframe_tf_future_extrapolation_use_latest']
            ),
            'tf_lookup_timeout_sec': float(U['keyframe_tf_lookup_timeout_sec']),
            'tf_buffer_cache_sec': float(U['keyframe_tf_buffer_cache_sec']),
            'deskew_enable': bool(U['keyframe_deskew_enable']),
            'deskew_imu_topic': kf_deskew_imu,
            'deskew_max_imu_age_sec': float(U['keyframe_deskew_max_imu_age_sec']),
            'deskew_model': str(U['keyframe_deskew_model']).strip().lower(),
            'deskew_imu_sign': float(U['keyframe_deskew_imu_sign']),
            'deskew_imu_stamp_offset_sec': float(U['keyframe_deskew_imu_stamp_offset_sec']),
            'deskew_cloud_stamp_offset_sec': float(
                U['keyframe_deskew_cloud_stamp_offset_sec']
            ),
            'deskew_imu_buffer_max_samples': int(
                U['keyframe_deskew_imu_buffer_max_samples']
            ),
            'deskew_imu_interpolate': bool(U['keyframe_deskew_imu_interpolate']),
            'deskew_mean_gyro_fallback': bool(U['keyframe_deskew_mean_gyro_fallback']),
            'deskew_imu_rotate_gyro_to_frame': kf_gyro_to_frame,
            'deskew_imu_rotation_tf_timeout_sec': float(
                U.get('keyframe_deskew_imu_rotation_tf_timeout_sec', 0.08)
            ),
            'deskew_livox_imu_sensor_as_cloud_identity': bool(
                U.get('keyframe_deskew_livox_imu_sensor_as_cloud_identity', True)
            ),
            'rotation_adaptive_keyframes': bool(
                U['keyframe_rotation_adaptive_keyframes']
            ),
            'rotation_gyro_z_thresh_rad_s': float(
                U['keyframe_rotation_gyro_thresh_rad_s']
            ),
            'rotation_keyframe_scale': float(U['keyframe_rotation_keyframe_scale']),
            'reject_unstable_frame_enable': bool(
                U.get('keyframe_reject_unstable_frame_enable', False)
            ),
            'reject_unstable_frame_max_translation_m': float(
                U.get('keyframe_reject_unstable_frame_max_translation_m', 1.0)
            ),
            'reject_unstable_frame_max_yaw_deg': float(
                U.get('keyframe_reject_unstable_frame_max_yaw_deg', 45.0)
            ),
            'auto_level_enable': bool(U.get('keyframe_auto_level_enable', True)),
            'auto_level_min_keyframes': int(U.get('keyframe_auto_level_min_keyframes', 8)),
            'auto_level_min_points': int(U.get('keyframe_auto_level_min_points', 1500)),
            'auto_level_max_points': int(U.get('keyframe_auto_level_max_points', 20000)),
            'auto_level_ransac_iters': int(U.get('keyframe_auto_level_ransac_iters', 140)),
            'auto_level_plane_dist_thresh_m': float(
                U.get('keyframe_auto_level_plane_dist_thresh_m', 0.08)
            ),
            'auto_level_max_tilt_deg': float(U.get('keyframe_auto_level_max_tilt_deg', 35.0)),
            'use_lidar_odom_for_robot_pose': bool(
                U.get('keyframe_use_lidar_odom_for_robot_pose', False)
            ),
            'lidar_odom_topic': str(
                U.get('keyframe_lidar_odom_topic', '/lidar/odom') or '/lidar/odom'
            ).strip(),
            'lidar_odom_max_age_sec': float(
                U.get('keyframe_lidar_odom_max_age_sec', 0.15)
            ),
            'lidar_odom_approximate_sync': bool(
                U.get('keyframe_lidar_odom_approximate_sync', True)
            ),
            'lidar_odom_approx_sync_slop_sec': float(
                U.get('keyframe_lidar_odom_approx_sync_slop_sec', 0.08)
            ),
            'lidar_odom_approx_sync_queue_size': int(
                U.get('keyframe_lidar_odom_approx_sync_queue_size', 10)
            ),
            'prefilter_min_range_m': float(U.get('keyframe_prefilter_min_range_m', 0.5)),
            'prefilter_max_range_m': float(U.get('keyframe_prefilter_max_range_m', 20.0)),
            'prefilter_self_radius_m': float(U.get('keyframe_prefilter_self_radius_m', 0.5)),
            'prefilter_self_bbox_enable': bool(
                U.get('keyframe_prefilter_self_bbox_enable', True)
            ),
            'prefilter_self_bbox_min_x': float(
                U.get('keyframe_prefilter_self_bbox_min_x', -0.3)
            ),
            'prefilter_self_bbox_max_x': float(
                U.get('keyframe_prefilter_self_bbox_max_x', 0.3)
            ),
            'prefilter_self_bbox_min_y': float(
                U.get('keyframe_prefilter_self_bbox_min_y', -0.3)
            ),
            'prefilter_self_bbox_max_y': float(
                U.get('keyframe_prefilter_self_bbox_max_y', 0.3)
            ),
            'prefilter_self_bbox_min_z': float(
                U.get('keyframe_prefilter_self_bbox_min_z', -0.2)
            ),
            'prefilter_self_bbox_max_z': float(
                U.get('keyframe_prefilter_self_bbox_max_z', 0.8)
            ),
            'prefilter_intensity_enable': bool(
                U.get('keyframe_prefilter_intensity_enable', False)
            ),
            'prefilter_min_intensity': float(
                U.get('keyframe_prefilter_min_intensity', 10.0)
            ),
            'prefilter_sor_enable': bool(U.get('keyframe_prefilter_sor_enable', False)),
            'prefilter_sor_mean_k': int(U.get('keyframe_prefilter_sor_mean_k', 20)),
            'prefilter_sor_stddev_mul': float(
                U.get('keyframe_prefilter_sor_stddev_mul', 1.0)
            ),
            'prefilter_sor_max_points': int(
                U.get('keyframe_prefilter_sor_max_points', 4500)
            ),
            'deskew_max_gyro_norm_rad_s': float(
                U.get('keyframe_deskew_max_gyro_norm_rad_s', 5.0)
            ),
        }
        keyframe_node = Node(
            package='keyframe_scan_map',
            executable='keyframe_map_node',
            name='keyframe_map_node',
            output='screen',
            parameters=[keyframe_params, keyframe_overrides, sim_time_param],
        )
        _kf_delay = float(U.get('keyframe_map_node_start_delay_sec', 0.0) or 0.0)
        if _kf_delay > 0.0:
            actions.append(TimerAction(period=_kf_delay, actions=[keyframe_node]))
        else:
            actions.append(keyframe_node)

    if start_keyframe_map and start_pose_graph:
        pose_graph_yaml = _share_file(U['pose_graph_params_pkg'], U['pose_graph_params_yaml'])
        pose_graph_overrides = {
            'publish_map_odom_tf': pose_graph_pub_tf,
            'odom_stamp_topic': str(U['ekf_publish_topic']).strip(),
            'odom_stamp_max_past_sec': float(
                U.get('pose_graph_odom_stamp_max_past_sec', 25.0) or 25.0
            ),
            'odom_stamp_max_future_sec': float(
                U.get('pose_graph_odom_stamp_max_future_sec', 2.0) or 2.0
            ),
            'weight_odom': float(U['pose_graph_weight_odom']),
            'weight_loop': float(U['pose_graph_weight_loop']),
            'max_graph_nodes': int(U['pose_graph_max_nodes']),
            'max_loop_edges': int(U['pose_graph_max_loop_edges']),
            'map_odom_tf_period_sec': float(U['pose_graph_map_odom_tf_period_sec']),
            'map_odom_tf_smooth_alpha': float(
                U.get('pose_graph_map_odom_tf_smooth_alpha', 1.0)
            ),
        }
        _pg_delay_raw = U.get('pose_graph_node_start_delay_sec', None)
        if _pg_delay_raw is None:
            _pg_delay = float(U.get('keyframe_map_node_start_delay_sec', 0.0) or 0.0)
        else:
            _pg_delay = float(_pg_delay_raw or 0.0)
        pose_graph_node = Node(
            package='keyframe_scan_map',
            executable='pose_graph_node',
            name='pose_graph_node',
            output='screen',
            parameters=[pose_graph_yaml, pose_graph_overrides, sim_time_param],
        )
        if _pg_delay > 0.0:
            actions.append(TimerAction(period=_pg_delay, actions=[pose_graph_node]))
        else:
            actions.append(pose_graph_node)

    if start_rviz:
        rviz_cfg = _share_file(U['rviz_config_pkg'], U['rviz_config_yaml'])
        rviz_cfg = _rviz_config_with_keyframe_dot_overrides(
            rviz_cfg,
            keyframe_pixels=rviz_kf_px,
            keyframe_size_m=rviz_kf_m,
        )
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_cfg],
            output='screen',
            parameters=[sim_time_param],
        )
        actions.append(rviz_node)

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description=(
                'If true: all nodes use /clock — use with `ros2 bag play ... --clock`. '
                'Default false: live robot wall clock.'
            ),
        ),
        GroupAction(
            actions=[SetUseSimTime(True)],
            condition=IfCondition(LaunchConfiguration('use_sim_time')),
        ),
        DeclareLaunchArgument(
            'launch_sensors',
            default_value='false',
            description=(
                'If false (default): sensors + TF over DDS only — no Livox/Microstrain drivers or '
                'sensor static TFs on this host. If true: start drivers per slam_bringup on this host.'
            ),
        ),
        DeclareLaunchArgument(
            'bringup_config',
            default_value='',
            description=(
                'Path to a YAML (slam_bringup mapping) merged over defaults. If empty, '
                'use ROS_PROJECT_SLAM_CONFIG, else share/ros_project_bringup/config/slam_bringup.yaml'
            ),
        ),
        DeclareLaunchArgument(
            'use_lio',
            default_value='',
            description=(
                'If non-empty: override slam_bringup ``use_lio`` (true|false). '
                'Empty (default): use YAML. Example: ``use_lio:=true`` + ``use_lidar_fusion:=false`` '
                'for FAST-LIO + relay without NDT.'
            ),
        ),
        DeclareLaunchArgument(
            'use_lidar_fusion',
            default_value='',
            description=(
                'If non-empty: override slam_bringup ``use_lidar_fusion`` (true|false). '
                'Empty (default): use YAML.'
            ),
        ),
        DeclareLaunchArgument(
            'start_rviz',
            default_value='',
            description=(
                'If non-empty: override slam_bringup ``start_rviz`` (true|false). '
                'Use false on headless robots; run RViz on a machine with a display.'
            ),
        ),
        DeclareLaunchArgument(
            'start_keyframe_map',
            default_value='',
            description=(
                'If non-empty: override slam_bringup ``start_keyframe_map`` (true|false). '
                'Empty (default): use YAML.'
            ),
        ),
        DeclareLaunchArgument(
            'start_pose_graph',
            default_value='',
            description=(
                'If non-empty: override slam_bringup ``start_pose_graph`` (true|false). '
                'Empty (default): use YAML.'
            ),
        ),
        DeclareLaunchArgument(
            'rviz_keyframe_map_size_pixels',
            default_value='',
            description=(
                'Optional RViz override for "Keyframe map" display Size (Pixels). '
                'Empty uses value from rviz config.'
            ),
        ),
        DeclareLaunchArgument(
            'rviz_keyframe_map_size_m',
            default_value='',
            description=(
                'Optional RViz override for "Keyframe map" display Size (m). '
                'Empty uses value from rviz config.'
            ),
        ),
        OpaqueFunction(function=launch_setup),
    ])
