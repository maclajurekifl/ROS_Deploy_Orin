# LiDAR Odometry Setup Instructions (ROS2 FAST-LIO + Custom EKF)

This guide keeps your same workflow, but updates it to the ROS2 FAST-LIO fork:

- FAST-LIO ROS2 repo: https://github.com/Ericsii/FAST_LIO_ROS2

## Current Status

- You already have `livox_ros_driver2` in `src/`.
- Do **not** clone Livox driver again.
- You should use ROS2 FAST-LIO (not ROS1/catkin FAST_LIO).

## Quick start: Livox MID360 + NDT + EKF + RViz (ROS 2 Humble)

This workspace’s **single-command** bringup (driver, PCL NDT → `/lidar/odom`, Python EKF, RViz) is documented in the root **`README.md`**, **§5.3**.

After **`colcon build`** and **`source install/setup.bash`**, typical run:

```bash
cd ~/ROS_Deployment
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ros_project_bringup launch_slam.launch.py
```

Variants (**driver already running**, **IMU-only**, **no RViz**) use the same `cd` / `source` lines plus launch arguments — see **README §5.3**.

## Goal Architecture

- Livox driver publishes LiDAR + IMU.
- **Default bringup** uses **your** NDT package (`lidar_odometry`) → `/lidar/odom` + **`ekf_node`** (see root **README §5.3**).
- **Optional LIO path:** `ros2 launch ros_project_bringup launch_slam.launch.py use_lio:=true` starts **FAST-LIO** + **`lio_odom_relay_node`**, which republishes **`/Odometry`** as **`/lidar/odom`** so **`ekf_node`** and the keyframe map stay on the same interface. FAST-LIO **`publish_tf`** is off; **`ekf_node`** keeps **`odom` → `base_link`**.
- Your custom `ekf_node` fuses:
  - IMU (`/livox/imu`) for predict
  - LiDAR odometry (`/lidar/odom` by default) for correction
  - optional extra altitude stream (`/lidar/z`)
- Downstream nodes use `/ekf/odom` as the main estimate.

## 1) Add ROS2 FAST-LIO

From `~/ROS_Deployment/src`:

```bash
git clone --recursive -b ros2 https://github.com/Ericsii/FAST_LIO_ROS2.git
```

If already cloned without submodules:

```bash
cd ~/ROS_Deployment/src/FAST_LIO_ROS2
git submodule update --init --recursive
```

## 2) Build

From `~/ROS_Deployment`:

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## 3) Launch order (important for Livox)

**Integrated bringup (recommended):** one launch starts driver + NDT **or** LIO + EKF (see **README §5.3**).

```bash
# NDT + EKF (default)
ros2 launch ros_project_bringup launch_slam.launch.py

# FAST-LIO + EKF + relay (same /lidar/odom topic for EKF)
ros2 launch ros_project_bringup launch_slam.launch.py use_lio:=true use_lidar_fusion:=false
```

**Standalone FAST-LIO** (debugging / comparing to upstream):

```bash
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml
```

Notes:

- This workspace’s driver often uses **`sensor_msgs/PointCloud2`** on **`/livox/lidar`**. The **`lio_bringup`** overlay sets **`preprocess.lidar_type: 4`** (generic PointCloud2 path). If you switch the driver to Livox **CustomMsg** and use type **1**, change the overlay accordingly.
- Topics must match **`common.lid_topic`** and **`common.imu_topic`** in **`FAST_LIO_ROS2/config/mid360.yaml`**.

## 4) Configure and tune FAST-LIO (MID360 + this workspace)

**Primary files**

| File | Role |
|------|------|
| **`src/FAST_LIO_ROS2/config/mid360.yaml`** | Baseline MID360 preset (topics, publish flags, extrinsics seed). |
| **`src/lio_bringup/config/fastlio_mid360_overlay.yaml`** | **Your tuned overrides**: PointCloud2 mode, **`publish_tf: false`**, IMU noise, **`det_range`**, **`pcd_save` off**, ICP depth. **Edit this first** for day-to-day tuning. |

**Parameters that matter most**

1. **`mapping.extrinsic_T`** / **`extrinsic_R`** — IMU–LiDAR geometry inside FAST-LIO. Wrong values dominate errors. After behavior is stable, try **`extrinsic_est_en: false`** and fixed calibration.
2. **`mapping.det_range`** — meters of lidar used (default overlay **55**; increase for open space, decrease if far noise hurts).
3. **`preprocess.blind`** — minimum range (m); reduces body returns (overlay **0.55**).
4. **`mapping.acc_cov`** / **`gyr_cov`** — raise if the solution is jittery or IMU is noisy; lower only with care.
5. **`max_iteration`**, **`filter_size_surf`**, **`filter_size_map`** — accuracy vs CPU.

**Frames:** FAST-LIO still publishes **`/Odometry`** with **`camera_init` / `body`**. The relay only rewrites **frame_id** / **child_frame_id** to **`odom` / `base_link`** for **`ekf_node`**. Calibrate **`base_link` → `livox_frame`** in **`launch_slam`** for clouds and visualization; that is **separate** from FAST-LIO’s internal extrinsics above.

## 5) Verify FAST-LIO output

```bash
ros2 topic list
ros2 topic info <fastlio_odom_topic>
ros2 topic hz <fastlio_odom_topic>
ros2 topic echo <fastlio_odom_topic> --once
```

Expected:

- type is `nav_msgs/msg/Odometry`
- stable publish rate

Tip:

- If FAST-LIO publishes on a different odom topic name, remap or set your EKF parameter accordingly.

## 6) TF frames (`map` → `odom` → `base_link`)

The Python `ekf_node` publishes:

- `nav_msgs/Odometry` with `frame_id = odom`, `child_frame_id = base_link`
- TF: **`odom` → `base_link`**

You must also have:

- **`map` → `odom`** (static identity for testing, or from SLAM / global localization)
- **`base_link` → `livox_frame`** (or your sensor frame), with a calibrated transform when you have extrinsics

`ros_project_bringup/launch/launch_slam.launch.py` publishes placeholder static transforms for the chain above.

## 7) Optional: PCL NDT lidar odometry (`lidar_odometry` package)

This workspace includes **`lidar_odometry_node`**, a lightweight **scan-to-scan** odometry
node using **PCL `NormalDistributionsTransform`** (no ICP / CSM).

- **Input:** `sensor_msgs/PointCloud2` (default `/livox/lidar`)
- **Outputs:**
  - `/lidar/odom` — `nav_msgs/Odometry` (integrated pose in `odom` → `base_link`)
  - `/lidar/relative_motion` — `geometry_msgs/TwistStamped` carrying **Δx**, **Δy**, **Δθ**:
    `twist.linear.x`, `twist.linear.y`, `twist.angular.z` (rad)

```bash
source install/setup.bash
ros2 launch lidar_odometry lidar_odometry.launch.py
```

Tune **`ndt_resolution`**, **`voxel_leaf_size`**, and **`max_fitness_score`** for your environment
(see `src/lidar_odometry/README.md`). Keep **`publish_tf: false`** if the EKF (or another node)
already publishes `odom` → `base_link`.

This is **not** a replacement for FAST-LIO in hard scenarios; it is appropriate for
coursework / fusion experiments alongside your EKF.

## 8) Connect FAST-LIO to your `ekf_node`

**Fusion model:** IMU integration is the **prediction** step; LiDAR pose is the **measurement update**.

- **NDT path:** planar odometry → keep **`lidar_fuse_z_from_odom: false`** (only **x, y, yaw**).
- **`use_lio` bringup:** **`launch_slam.launch.py`** sets **`ekf_lidar_fuse_z_from_odom`** default **`auto`**: **`true`** when **`use_lio`** so the EKF fuses **x, y, z, yaw** from **`/lidar/odom`** (LIO has meaningful height). Override with **`ekf_lidar_fuse_z_from_odom:=false`** if you want planar fusion only.

Use **`lidar_z_topic`** for a separate altitude measurement if needed.

Set EKF parameters (or use `localisation_ekf/config/ekf_python.yaml`):

- `imu_topic: /livox/imu`
- `lidar_odom_topic: /lidar/odom` (relay fills this from FAST-LIO when using **`use_lio`**)
- `lidar_pose_topic: ''`
- `lidar_z_topic: ''` (unless you add **`/lidar/z`**)

Suggested start values (workspace default is **accuracy-first**; increase variances if too jumpy):

- `lidar_pose_var: 0.015`
- `lidar_yaw_var: 0.008`
- `lidar_gate_nis: 32.0`
- `lidar_use_roll_pitch: false`

## 9) Add z reinforcement later (Use Case 4)

When ready, add a node that publishes:

- `/lidar/z` as `std_msgs/Float64` (`data = z`)

Then enable in your EKF params:

- `lidar_z_topic: /lidar/z`
- `lidar_z_var: 0.02` (starting point)

## 10) Tuning Guidelines

If EKF is jumpy:

- increase `lidar_pose_var` (example: `0.08` to `0.2`)
- reduce `lidar_gate_nis` (example: `12.0`)

If EKF is sluggish:

- decrease `lidar_pose_var`

**Process noise (`process_noise_diag` in `ekf_python.yaml`):** length-9 vector for
`px, py, z, yaw, vx, vy, bax, bay, bgz`. Increase velocity entries if the filter lags motion;
increase bias entries slightly if biases look stuck; decrease if estimates get noisy.

## 11) Minimal Progression Plan

1. Keep IMU-only EKF healthy.
2. Add FAST-LIO ROS2 and validate odometry standalone.
3. Fuse FAST-LIO odometry into your EKF.
4. Add optional `/lidar/z` reinforcement once baseline is stable.
