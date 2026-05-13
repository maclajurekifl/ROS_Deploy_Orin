# Deployment guide

This document describes how to run **sensors only** vs **processing on another machine**, where LiDAR and IMU settings live, and what to copy to a **minimal robot workspace**.

## `launch_sensors` (bringup)

`ros_project_bringup`’s `launch_slam.launch.py` declares a launch argument **`launch_sensors`** (default `true`).

### Default — sensors on this machine

Same behaviour as before Livox, Microstrain (if enabled in `config/slam_bringup.yaml`), and sensor static transforms are started when the file says so.

```bash
source /opt/ros/<distro>/setup.bash
source /path/to/ROS_Deployment/install/setup.bash
ros2 launch ros_project_bringup launch_slam.launch.py
```

### Sensors off — typical processing host

When **`launch_sensors` is false**, this machine does **not** start:

- Livox driver  
- Microstrain driver  
- Static TF `base_link` → `livox_frame`  
- Static TF for the IMU mount (only published together with the Microstrain driver)

EKF, NDT / FAST-LIO, keyframe map, pose graph, RViz, and `map` → `odom` static TF still follow **`config/slam_bringup.yaml`** (or an overlay). EKF’s `lidar_cloud_stamp_topic` remains set when LiDAR fusion is on but there is no **local** Livox, so stamped sync still works if `/livox/lidar` arrives over DDS.

```bash
ros2 launch ros_project_bringup launch_slam.launch.py launch_sensors:=false
```

Values treated as **off**: `false`, `0`, `no`, `off` (case-insensitive). Any other value is treated as **on**.

### Robot + host over the network

- Use the same **`ROS_DOMAIN_ID`** on both machines and ensure DDS can reach between them (firewall / VLAN / `RMW_IMPLEMENTATION` as appropriate).
- **Robot:** leave `launch_sensors` at default `true` (or pass `launch_sensors:=true`).
- **Host:** `launch_sensors:=false` and keep **topic names** and **bringup** (extrinsics, frames, `use_microstrain_imu`, IMU topics) **aligned** with the robot so TF and subscriptions match.
- Sensor static TFs are **not** republished on the host when `launch_sensors:=false`; the host should receive **`/tf_static`** from the robot for `base_link` → sensors (or you duplicate identical static publishers by policy elsewhere).

---

## Where LiDAR and IMU are configured

### LiDAR (Livox MID360)

| What | Location |
|------|----------|
| **Lidar IP, host/lidar ports, MID360 network** | `src/livox_ros_driver2/config/MID360_config.json` — set `lidar_configs[0].ip` and `host_net_info` for your robot / lidar network. |
| **Driver node options** (e.g. `frame_id`, publish rate, point format, `cmdline_bd_code`) | `src/livox_ros_driver2/launch_ROS2/rviz_MID360_launch.py` — constants at the top; `user_config_path` resolves to `config/MID360_config.json` in the **installed** `livox_ros_driver2` share after `colcon build`. |
| **`base_link` → LiDAR frame extrinsic** (stack / TF; separate from the JSON’s per-lidar extrinsic block) | `config/slam_bringup.yaml` — `livox_extrinsic_*`, `livox_cloud_frame_id`. **`livox_cloud_frame_id` must match** the driver `frame_id` (launch args from bringup; default `livox_frame`). |
| **Cloud topic used by odometry / mapping** | Same dict: e.g. `lidar_cloud_topic`, `keyframe_cloud_topic` (defaults under `/livox/lidar`). |

After install, the driver reads:

`install/livox_ros_driver2/share/livox_ros_driver2/config/MID360_config.json`  
(same content as under `src/...` once the package is built and installed.)

### IMU

**Livox IMU** — Published by the Livox node (e.g. `/livox/imu`). There is no separate Livox IMU config file beyond driver + launch constants above; fusion tuning is in EKF YAML / launch dict.

**Microstrain GX5-25** (optional):

| What | Location |
|------|----------|
| **Vendor default parameters** | `microstrain_inertial_driver` package: `microstrain_inertial_driver_common/config/params.yml` (apt install or source build). `launch_slam.launch.py` loads this file and **overrides** port, baud, rates, frame IDs. |
| **Port, baud, rates, topics, mount frames for this repo** | `config/slam_bringup.yaml` — `use_microstrain_imu`, `microstrain_port`, `microstrain_baud`, `microstrain_imu_data_raw_rate`, `microstrain_imu_topic`, `microstrain_frame_id`, `imu_mount_*`, etc. |
| **EKF preset for Microstrain + Livox LiDAR** | `src/localisation_ekf/config/ekf_python_gx5_microstrain.yaml` (set `ekf_params_yaml` in bringup). |

More context: **README.md** §1.3 (Microstrain) and §7 (file index).

---

## Minimal “standalone sensor” workspace on the robot

To run **only** drivers (and optionally the same static TFs as this bringup), you need at least:

1. **`livox_ros_driver2`** (as in this workspace)  
   - `config/MID360_config.json`  
   - `launch_ROS2/rviz_MID360_launch.py` (or your own launch that starts `livox_ros_driver2_node` with the same parameter set, including `user_config_path` pointing at your JSON).

2. **If you use Microstrain:** the **`microstrain_inertial_driver`** package plus either the bundled `params.yml` with launch-time overrides (as here) or your own maintained copy of `params.yml` if you fork the launch.

3. **`ros_project_bringup`** only if you still drive everything through **`launch_slam.launch.py`**: keep **bringup** (or the same **overlay** YAML) **consistent** with the host for **`livox_extrinsic_*`**, **`livox_cloud_frame_id`**, **`imu_mount_*`**, and Microstrain keys so topics and TF match.

You **do not** need to ship EKF / NDT / FAST-LIO / keyframe YAMLs to the robot unless you run those nodes on the robot. For sensors-only, the critical artefacts are the **Livox JSON + Livox launch (or equivalent)** and, if applicable, **Microstrain** + **port/rate/frame** overrides.

To run the **full algorithm stack** on the host while sensors stay on the robot: use **`launch_sensors:=false`** on the host and default sensors on the robot, with matching domain and configuration as above.
