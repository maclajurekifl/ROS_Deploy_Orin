# ROS_Deployment

ROS 2 workspace for **Livox MID360** sensing, **FAST-LIO** mapping, **PCL NDT scan-to-scan odometry**, and a **custom planar EKF** that fuses IMU prediction with LiDAR measurement updates (position and yaw). This document is the **single entry point** for what is in the project, how to run it, and where to tune it. **Section 7** lists **every tuning-related file** (path + role); **section 8** is a compact **parameter cheat sheet**.

---

## 1. What is included

| Package | Role |
|--------|------|
| **`livox_ros_driver2`** (vendored) | Livox driver: point cloud (`/livox/lidar`), IMU (`/livox/imu`), frame `livox_frame`. |
| **`FAST_LIO_ROS2`** | **FAST-LIO** (`fastlio_mapping`). Used by **`launch_slam`** when **`use_lio:=true`** (via **`lio_bringup`**), or run separately with **`launch/mapping.launch.py`**. |
| **`lio_bringup`** | **`lio_backend.launch.py`**: FAST-LIO + **`lio_odom_relay_node`** (`/Odometry` → **`/lidar/odom`** as **`odom`→`base_link`**). Overlay YAML sets **`publish_tf: false`** (EKF owns **`odom`→`base_link`**) and **`preprocess.lidar_type: 4`** for PointCloud2. |
| **`localisation_ekf`** | Python **`ekf_node`**: planar IMU **prediction** + optional LiDAR **correction** (`x, y, yaw`; optional `z`). Params in `config/ekf_python.yaml`. |
| **`lidar_odometry`** | C++ **`lidar_odometry_node`**: PCL **NDT** — **`scan_to_map`** (default: TF motion prior + map) or **`scan_to_scan`** → **`/lidar/odom`**, **`/lidar/relative_motion`**, optional **`/lidar/pose_correction`**. |
| **`keyframe_scan_map`** | **`keyframe_map_node`**: merged **`/keyframe_map`** + **`/keyframes`**; optional loop detection (**`keyframe_loop_closure_enable`**). **`pose_graph_node`**: lightweight **SE2 pose graph** (**`start_pose_graph`**) → **`/pose_graph/corrected_keyframes`**. |
| **`ros_project_bringup`** | Example launch **`launch_slam.launch.py`**, RViz **`rviz/slam.rviz`**. |

### 1.1 Front-end vs back-end (split)

| Tier | Role in this repo | Typical rate |
|------|-------------------|--------------|
| **Front-end (fast)** | **IMU** (`/livox/imu`) + **LiDAR odometry** — either **`lidar_odometry_node`** (PCL NDT) or **`fastlio_mapping`** + **`lio_odom_relay_node`** → **`/lidar/odom`**. **EKF** (**`ekf_node`**) predicts from IMU and corrects from LiDAR odom; publishes **`odom`→`base_link`**. Livox driver supplies sensors. | Sensor / odom rate (tens–hundreds of Hz for IMU; odom as published). |
| **Back-end (slow)** | **Keyframe map** (**`keyframe_map_node`**: merge only on keyframe rule). **Loop closure** (optional overlap detection → `/keyframe_map/loop_closure_*`). **Map optimisation** (**`pose_graph_node`**: SciPy SE2 graph on keyframe path + loop edges; optional map rebuild or **`map`→`odom`** correction). | Keyframe cadence; graph solve on path/loop updates (seconds-scale or longer). |

The front-end keeps pose tracking smooth; the back-end reduces **global** drift when loops exist. You can run **`start_keyframe_map:=false`** to use front-end only.

### 1.2 Deployment checklist (clean bringup)

**Single command:** `ros2 launch ros_project_bringup launch_slam.launch.py` (after `source install/setup.bash`).

**Stack narrative (nodes, NDT vs LIO, devices):** see **`ros_project_bringup/docs/STACK_OVERVIEW.md`** (installed under `share/ros_project_bringup/docs/` after build).

**Where to tune:** edit **`ros_project_bringup/config/slam_bringup.yaml`** (stack toggles, NDT/EKF/keyframe/pose-graph, Livox node settings, topic names, and paths to per-package YAML under each share directory). The launch file documents defaults and the **overlay** mechanism: **`bringup_config:=/path/yaml`**, or **`ROS_PROJECT_SLAM_CONFIG`**, to merge a partial file over the installed default (handy in Docker). Livox hardware: **`livox_config_path`** to a host-mounted JSON, or edit **`livox_ros_driver2/config/MID360_config.json`**. FAST-LIO: **`fastlio_params_file`** / **`lio_overlay_params_file`** in the same bringup file.

### 1.3 Lord Microstrain 3DM-GX5-25 IMU (optional Livox IMU replacement)

**Stack-level issues & fixes** (wiring `imu_topic`, install vs `src`, stamp/TF quirks, NDT/EKF ordering): **`documentation/issues-and-solutions-summary.md`**.

**Goal:** GX5-25 provides **high-rate `sensor_msgs/Imu`** for **EKF prediction**; **Livox** still supplies **LiDAR only** (cloud unchanged). **NDT / LIO** still feeds **`/lidar/odom`** for **EKF correction**.

**1 — Install the official driver**

```bash
sudo apt update
sudo apt install ros-humble-microstrain-inertial-driver
```

(Replace `humble` with your ROS 2 distro.) Or build from [LORD-MicroStrain/microstrain_inertial](https://github.com/LORD-MicroStrain/microstrain_inertial) (`ros2` branch).

**2 — Find the serial port**

- Unplug the GX5, run `ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null`; plug it in; run again — the **new** path (often `/dev/ttyACM0`) is the device.
- `dmesg | tail -30` right after plugging in shows the kernel-assigned device name.
- Prefer a **stable symlink** from the vendor udev rules (e.g. `/dev/microstrain_*`) if you install them, then set `microstrain_port` to that symlink.

Pass the port into bringup:

`microstrain_port:=/dev/ttyACM0`

**3 — Frames (critical)**

The driver publishes IMU with `header.frame_id` = **`imu_link`** (default; overridable via `microstrain_frame_id`). **`ekf_node`** expects accelerations and angular rates in **`base_link`** when they differ from the IMU frame: it uses TF to rotate vectors into `base_link` (`transform_imu_to_base_link` in `ekf_python.yaml`).

Bringup adds a **static transform** `imu_mount_parent_frame` → `imu_mount_child_frame` (defaults **`base_link` → `imu_link`**) with translation/rotation from launch args **`imu_mount_*`**. **Calibrate** these from your CAD or a hand‑eye / IMU calibration so the IMU axes match the robot.

**4 — Launch example**

Set in **`config/slam_bringup.yaml`**, for example: `use_microstrain_imu: true`, `microstrain_port: /dev/ttyACM0`, `ekf_params_yaml: config/ekf_python_gx5_microstrain.yaml`, and the **`imu_mount_*`** entries. Then:

```bash
source /opt/ros/humble/setup.bash
source ~/ROS_Deployment/install/setup.bash
ros2 launch ros_project_bringup launch_slam.launch.py
```

The driver’s **mount→IMU** TF is turned **off** in launch (`publish_mount_to_frame_id_transform: false`) so only your **`imu_mount_*`** static TF defines geometry (no duplicate publishers).

**5 — Raw vs driver-filtered IMU**

| Topic (typical) | Use case |
|-----------------|----------|
| **`/imu/data_raw`** (default `microstrain_imu_topic`) | Raw gyro + accel for **your** planar EKF — recommended here. |
| **`/imu/data`** | Driver-processed IMU (may blend internal filtering). **`ekf_node` still uses only `linear_acceleration` and `angular_velocity`**, not the quaternion, for propagation — so filtered orientation is **not** fused unless you extend the filter. |

Using the device’s **internal EKF orientation** or **mag heading** inside this repo would require **new measurement updates** in `ekf_node` / `EKFPlanarIMU` (not enabled by default). For most SLAM stacks, **raw inertial rates + LiDAR yaw** is the cleaner split.

**6 — EKF tuning preset**

`localisation_ekf/config/ekf_python_gx5_microstrain.yaml` lowers **yaw** process noise slightly and sets **vx/vy** process noise to a **medium** band vs the Livox preset, with a slightly tighter default **`lidar_yaw_var`** so LiDAR can still dominate heading drift. Adjust per vehicle.

**Also in-repo (reference / alternate stack):**

- `src/localisation_ekf/config/ekf_node.yaml` — commented templates for **`robot_localization`** `ekf_filter_node`; the **active** Python EKF uses **`ekf_python.yaml`**, not this file’s bottom block, unless you run `robot_localization` separately.
- `src/LiDAR-Instructions.md` — FAST-LIO setup, TF notes, and progression plan.

---

## 2. Prerequisites

- **ROS 2 Humble** (this workspace is written and tested against **Humble**; use another distro only if you adapt paths and dependencies yourself).
- **colcon** build tools.
- **PCL** development libraries (for `lidar_odometry`), e.g. `libpcl-dev`.
- **Python**: `tf-transformations` (ROS package), NumPy.
- **Hardware**: Livox MID360 (or adapt topics/config for another Livox).

---

## 3. Build and environment

From the workspace root:

```bash
cd ~/ROS_Deployment
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Build a subset while iterating:

```bash
colcon build --packages-select localisation_ekf lidar_odometry ros_project_bringup
```

---

## 4. Coordinate frames and TF

Intended chain for bringup (your launches may vary):

```text
map  --[static identity, or SLAM]-->  odom  --[ekf_node]-->  base_link  --[static extrinsic]-->  livox_frame
```

- **`ekf_node`** publishes **`odom` → `base_link`** when `publish_tf: true` (and fills `/ekf/odom` with the same frames).
- **`ros_project_bringup/launch/launch_slam.launch.py`** starts:
  - static **`map` → `odom`**
  - static **`base_link` → `livox_frame`** (identity placeholder — **replace with calibrated extrinsics** when you have them).
- **`lidar_odometry_node`** defaults to **`publish_tf: false`** so it does not compete with the EKF on `odom` → `base_link`.

**RViz:** `slam.rviz` uses fixed frame **`map`**; `fastlio.rviz` uses **`camera_init`** (FAST-LIO world frame).

---

## 5. How to run — common workflows

Always **`source install/setup.bash`** in each new terminal.

### 5.1 Livox driver only

Launch files are installed under **`launch_ROS2/`** in the package share:

```bash
ros2 launch livox_ros_driver2 launch_ROS2/rviz_MID360_launch.py
```

(Equivalent to what `launch_slam.launch.py` includes.)

Tune: `frame_id`, publish rate, and `MID360_config.json` in the driver package.

### 5.2 FAST-LIO mapping (separate terminal after driver)

```bash
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml
```

Optional: `rviz:=false` if you use another RViz config.

**Config file:** `src/FAST_LIO_ROS2/config/mid360.yaml` — topics, IMU/LiDAR alignment, filters, extrinsics (`extrinsic_T` / `extrinsic_R`), `publish` flags.

### 5.3 SLAM bringup: Livox + EKF + optional NDT LiDAR fusion + keyframe map + RViz

**Default odometry source:** **`lidar_odometry`** (PCL NDT) + **`ekf_node`**. **FAST-LIO** is optional: set **`use_lio:=true`** to start **`fastlio_mapping`** + relay instead of NDT (same **`/lidar/odom`** topic for the EKF and keyframe map).

**`launch_slam.launch.py`** can start the Livox driver (same as **`livox_ros_driver2/launch_ROS2/rviz_MID360_launch.py`**), static TFs, **`ekf_node`**, either **`lidar_odometry_node`** (NDT) **or** **`lio_bringup`** (FAST-LIO + relay), optionally **`keyframe_map_node`** (merged map **`/keyframe_map`**), and RViz (**`slam.rviz`** plots LiDAR, map cloud, **`/lidar/odom`**, **`/ekf/*`**).

All commands below assume **ROS 2 Humble** and that you have already built the workspace (**§3**). In **each new terminal**, source the overlay:

```bash
source /opt/ros/humble/setup.bash
source ~/ROS_Deployment/install/setup.bash
```

#### Full stack (Livox MID360 + NDT + EKF + keyframe map + RViz)

From the workspace root:

```bash
cd ~/ROS_Deployment
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ros_project_bringup launch_slam.launch.py
```

#### Variants

**Livox driver already running** (e.g. another terminal or launch):

```bash
cd ~/ROS_Deployment
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ros_project_bringup launch_slam.launch.py start_livox_driver:=false
```

**IMU only** (no LiDAR measurement updates; NDT / LIO off; the EKF `lidar_odom_topic` is cleared):

```bash
cd ~/ROS_Deployment
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ros_project_bringup launch_slam.launch.py use_lidar_fusion:=false
```

**FAST-LIO instead of NDT** (EKF + keyframe map unchanged; relay republishes LIO **`/Odometry`** as **`/lidar/odom`** with **`odom`→`base_link`** headers). Use **`use_lidar_fusion:=false`** so NDT does not run:

```bash
cd ~/ROS_Deployment
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ros_project_bringup launch_slam.launch.py use_lio:=true use_lidar_fusion:=false
```

If both **`use_lidar_fusion`** and **`use_lio`** are **`true`**, **LIO wins** (NDT is not started).

**What `use_lio:=true` replaces (quick reference)**  
| Stays the same | Replaced / different |
|----------------|----------------------|
| **`ekf_node`**, **`keyframe_map_node`**, Livox driver, static TFs **`map`→`odom`**, **`base_link`→`livox_frame`**, RViz if enabled | **`lidar_odometry_node` (NDT)** is **not** started |
| EKF still uses **`/lidar/odom`** | Source of **`/lidar/odom`** is **FAST-LIO + relay**, not NDT |
| | **`/lidar/relative_motion`** is **not** published (NDT-only); LIO publishes **`/Odometry`** |
| | Same LiDAR/IMU topics are **subscribed by LIO too** (NDT + LIO would duplicate work — do not run both on **`/lidar/odom`**) |

Same table lives in **`launch_slam.launch.py`** (top docstring) and **`localisation_ekf/config/ekf_python.yaml`** (header comments).

**No RViz**:

```bash
cd ~/ROS_Deployment
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ros_project_bringup launch_slam.launch.py start_rviz:=false
```

**No keyframe scan map** (saves CPU / memory):

```bash
cd ~/ROS_Deployment
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ros_project_bringup launch_slam.launch.py start_keyframe_map:=false
```

Arguments: **`use_lidar_fusion`**, **`use_lio`**, **`start_livox_driver`**, **`start_rviz`**, **`start_keyframe_map`**, **`keyframe_loop_closure_enable`**, **`start_pose_graph`**, **`ekf_lidar_fuse_z_from_odom`** (default **`auto`** for z fusion when LIO is on). See §8.2.

#### Calibrated **`base_link` → `livox_frame`**

Clouds are in **`livox_frame`**. The default extrinsic is **identity**; set **meters** and **degrees** on the command line when you have a calibration (improves NDT, EKF, and **`/keyframe_map`**):

| Launch argument | Meaning |
|-----------------|--------|
| **`livox_extrinsic_x`**, **`y`**, **`z`** | Translation **base_link → livox_frame** (m) |
| **`livox_extrinsic_roll_deg`**, **`pitch_deg`**, **`yaw_deg`** | Fixed-frame **roll / pitch / yaw** (deg), passed to `static_transform_publisher` as radians internally |

Example (illustrative numbers only — use your calibration):

```bash
ros2 launch ros_project_bringup launch_slam.launch.py \
  livox_extrinsic_z:=0.06 livox_extrinsic_pitch_deg:=-12.0
```

#### Tune **NDT** and **EKF LiDAR fusion** from the launch file

| Launch argument | Default | Role |
|-----------------|---------|------|
| **`lidar_voxel_leaf_size`** | `0.22` | Pre-NDT voxel (m) |
| **`lidar_crop_range_m`** | `40.0` | Crop half-range (m) |
| **`lidar_ndt_resolution`** | `0.85` | NDT cell size (m) |
| **`lidar_ndt_max_iterations`** | `50` | NDT iterations |
| **`lidar_max_fitness_score`** | `12.0` | Skip threshold (raise if many skips) |
| **`lidar_registration_mode`** | `scan_to_map` | `scan_to_scan` = consecutive scans only |
| **`lidar_use_tf_initial_guess`** | `true` | **scan_to_map:** TF at cloud stamp = **`ekf_node`** prior; `false` = last NDT pose |
| **`lidar_tf_initial_guess_timeout_sec`** | `0.1` | TF lookup timeout (s) |
| **`lidar_map_merge_voxel_leaf_size`** | `-1` | **scan_to_map:** merge voxel (m); `-1` uses **`lidar_voxel_leaf_size`** |
| **`lidar_map_max_points`** | `400000` | **scan_to_map:** map size cap (coarsens merge voxel if exceeded) |
| **`ekf_lidar_pose_var`** | `0.015` | EKF measurement variance **x, y** |
| **`ekf_lidar_yaw_var`** | `0.008` | EKF measurement variance **yaw** |
| **`ekf_lidar_gate_nis`** | `32.0` | NIS gate on LiDAR updates |

Finer **process noise** and **IMU** settings stay in **`config/ekf_python.yaml`**.

**LiDAR “stops correcting” (position drifts, yaw still looks OK):** If **`lidar_odometry_node`** logs **`NDT fitness … > max … — skip`**, raise **`lidar_max_fitness_score:=...`** on the command line (or edit defaults). See **`src/lidar_odometry/README.md`** (“EKF drift and `max_fitness_score`”).

**`ekf_python.yaml`** documents the chain: **`/lidar/odom`** ← **`lidar_odometry_node`**. The launch file **overrides** `lidar_odom_topic` to **`""`** when **`use_lidar_fusion:=false`**. Optional **`/lidar/z`** is left empty unless you add a publisher and set **`lidar_z_topic`** in YAML or launch.

### 5.4 NDT lidar odometry only

```bash
ros2 launch lidar_odometry lidar_odometry.launch.py
```

Requires **`/livox/lidar`** with **valid `header.stamp`**. Outputs:

- **`/lidar/odom`** — `nav_msgs/Odometry`, `odom` → `base_link`, planar pose  
- **`/lidar/relative_motion`** — `geometry_msgs/TwistStamped`: Δx, Δy, Δθ  
- **`/lidar/pose_correction`** — `geometry_msgs/PoseStamped` when using **`scan_to_map`** (NDT planar correction vs prediction)

See **`src/lidar_odometry/README.md`** for algorithm and parameters.

### 5.5 Fusion stack (typical order)

**Default in `launch_slam.launch.py`:**  
1. Livox driver  
2. Either **`lidar_odometry`** (NDT) **or**, with **`use_lio:=true`**, **`lio_bringup`** (FAST-LIO + relay) → **`/lidar/odom`**  
3. **`ekf_node`** (IMU predict + LiDAR correct on **x, y, yaw**)  
4. Optional **`keyframe_map_node`** → **`/keyframe_map`**  
5. Optional **`/lidar/z`** if you add a z publisher and set **`lidar_z_topic`**

**Alternate LiDAR pose without NDT:** **`use_lio:=true`** **`use_lidar_fusion:=false`** (single publisher on **`/lidar/odom`**). You can still run FAST-LIO manually with **`fast_lio/launch/mapping.launch.py`**, but then disable duplicate TF (**`publish_tf: false`** in YAML, patched in **`laserMapping.cpp`**) and remap odometry so only one node feeds **`/lidar/odom`**.

---

## 6. Topics reference

| Topic | Type | Producer | Notes |
|-------|------|----------|--------|
| `/livox/lidar` | `sensor_msgs/PointCloud2` | Livox driver | Input to FAST-LIO & NDT node |
| `/livox/imu` | `sensor_msgs/Imu` | Livox driver | `ekf_node` prediction |
| `/lidar/odom` | `nav_msgs/Odometry` | `lidar_odometry_node` or **`lio_odom_relay_node`** (from LIO **`/Odometry`**) | EKF measurement (`x,y,yaw`; z optional) |
| `/lidar/relative_motion` | `geometry_msgs/TwistStamped` | `lidar_odometry_node` | Δx, Δy, Δθ per step |
| `/lidar/pose_correction` | `geometry_msgs/PoseStamped` | `lidar_odometry_node` (**`scan_to_map`**) | Planar NDT correction vs prediction (`odom` frame) |
| `/lidar/z` | `std_msgs/Float64` | Your node | Optional z correction (`data` = z) |
| `/ekf/odom` | `nav_msgs/Odometry` | `ekf_node` | Fused estimate, `odom` → `base_link` |
| `/ekf/pose`, `/ekf/path` | `PoseStamped`, `Path` | `ekf_node` | Same frame as `/ekf/odom` header |
| `/keyframe_map` | `sensor_msgs/PointCloud2` | `keyframe_map_node` | Merged keyframe scans in **`map`** (drifts with odometry; not SLAM) |
| `/keyframe_map/keyframes` | `nav_msgs/Path` | `keyframe_map_node` | Keyframe robot poses in **`map`** |
| `/keyframe_map/loop_closure_match_index` | `std_msgs/UInt32` | `keyframe_map_node` (if enabled) | Past keyframe index that matched |
| `/keyframe_map/loop_closure_overlap` | `std_msgs/Float32` | `keyframe_map_node` (if enabled) | Overlap score \([0,1]\) |
| `/keyframe_map/loop_closure_anchor_pose` | `geometry_msgs/PoseStamped` | `keyframe_map_node` (if enabled) | Matched pose in **`map`** |
| `/keyframe_map/loop_closure_pair` | `std_msgs/Int32MultiArray` | `keyframe_map_node` (if enabled) | **`[past_idx, new_idx]`** for **`pose_graph_node`** |
| `/pose_graph/corrected_keyframes` | `nav_msgs/Path` | `pose_graph_node` (if started) | Optimized keyframe poses in **`map`** |

FAST-LIO also publishes map clouds and odometry-like messages per its config; see **`FAST_LIO_ROS2/README.md`**.

---

## 7. Tuning file index (where each file lives & what it controls)

Paths are relative to the workspace root **`ROS_Deployment/`** (i.e. under **`src/`** unless noted).

### 7.0 Full-stack bringup (`ros_project_bringup`)

| File | What it is | What you tune there |
|------|------------|---------------------|
| **`src/ros_project_bringup/config/slam_bringup.yaml`** | **Single entry point** for stack toggles, NDT/EKF/keyframe/pose_graph overrides, topic names, Microstrain, Livox driver + JSON path, FAST-LIO paths, extrinsics. | All **`lidar_*`**, **`ekf_*`**, **`keyframe_*`**, **`pose_graph_*`**, **`livox_*`**, etc. (see in-file header). Merged with optional **overlay** (`bringup_config` launch arg, **`ROS_PROJECT_SLAM_CONFIG`**, or a mounted file in Docker). |
| **`src/ros_project_bringup/launch/launch_slam.launch.py`** | Loads the YAML, starts nodes, passes parameter overlays to **`ekf_node`**, NDT, keyframe, pose graph. | Code paths only; prefer editing **`slam_bringup.yaml`**. |

### 7.1 EKF (`localisation_ekf`)

| File | What it is | What you tune there |
|------|------------|---------------------|
| **`src/localisation_ekf/config/ekf_python.yaml`** | **Primary** ROS 2 parameter file for **`ekf_node`** (`ekf_node` / `ros__parameters` block). | IMU topic, frames, **Q** (`process_noise_diag`), initial **P**, LiDAR topic names (default **`/lidar/odom`** for NDT), **`lidar_pose_var` / `lidar_yaw_var` / `lidar_z_var`**, **`lidar_fuse_z_from_odom`**, **`lidar_gate_nis`**, output topics, `publish_tf`. |
| **`src/localisation_ekf/localisation_ekf/ekf_node.py`** | Node implementation: declares parameters, subscribes to IMU + LiDAR, publishes `/ekf/*` and TF. | Only if you change **defaults**, add params, or alter **fusion logic** (e.g. frame checks, stamp handling). |
| **`src/localisation_ekf/localisation_ekf/ekf_filter.py`** | Planar EKF math: predict, `update_lidar_pose`, `update_lidar_xy_yaw`, `update_lidar_z`, NIS gating. | **Model / matrix** behavior: noise shaping, which states are updated, Joseph form, etc. |
| **`src/localisation_ekf/config/ekf_node.yaml`** | **Not** used by the Python `ekf_node`. Commented **`robot_localization`** `ekf_filter_node` templates + one example profile. | Only if you run **`robot_localization`** separately; ignore for the custom Python EKF. |

### 7.2 NDT lidar odometry (`lidar_odometry`)

| File | What it is | What you tune there |
|------|------------|---------------------|
| **`src/lidar_odometry/launch/lidar_odometry.launch.py`** | Standalone defaults for **`lidar_odometry_node`**. | Same parameter set as in **`launch_slam`** NDT block. |
| **`src/lidar_odometry/launch/lidar_odometry_scan_to_map.launch.py`** | Preset **`registration_mode:=scan_to_map`**. | Quick try of scan-to-map without editing **`launch_slam`**. |
| **`src/ros_project_bringup/config/slam_bringup.yaml`** (via launch) | Passes NDT parameters and **`sensor_extrinsic_rpy_xyz`** from **`livox_extrinsic_*`**. | **`lidar_*`** and **`livox_extrinsic_*`** in bringup YAML. |
| **`src/lidar_odometry/src/lidar_odometry_node.cpp`** | PCL NDT: **scan-to-scan** or **scan-to-map** (voxel map, **`/lidar/pose_correction`**). | Algorithm; tuning in **launch** / YAML. |
| **`src/lidar_odometry/README.md`** | Parameter table + both pipelines. | Reference only. |

### 7.3 Keyframe scan map (`keyframe_scan_map`)

| File | What it is | What you tune there |
|------|------------|---------------------|
| **`src/keyframe_scan_map/config/keyframe_map.yaml`** | Parameters for **`keyframe_map_node`**. | Keyframe spacing, voxels, **`loop_*`** (simple loop detection; default off). |
| **`src/keyframe_scan_map/launch/keyframe_map.launch.py`** | Standalone launch (same YAML). | Use when not using **`launch_slam.launch.py`**. |
| **`src/keyframe_scan_map/keyframe_scan_map/keyframe_map_node.py`** | TF cloud → **`map`**, merge, voxel; optional **loop closure** + **`loop_closure_pair`**. | **`loop_*`** in YAML. |
| **`src/keyframe_scan_map/keyframe_scan_map/pose_graph_node.py`** | SE2 graph: keyframe path + loop pairs → **`/pose_graph/corrected_keyframes`**. | **`config/pose_graph.yaml`**; needs **`python3-scipy`**. |
| **`src/keyframe_scan_map/config/pose_graph.yaml`** | **`pose_graph_node`** weights and topic names. | **`weight_loop`** vs **`weight_odom`**, **`max_graph_nodes`**. |
| **`src/keyframe_scan_map/README.md`** | Topic / parameter summary. | Reference only. |

### 7.4 Livox driver (`livox_ros_driver2`)

| File | What it is | What you tune there |
|------|------------|---------------------|
| **`src/livox_ros_driver2/config/MID360_config.json`** | MID360 connection / device config (as used by the driver). | Host/IP, lidar-specific options per Livox docs. |
| **`src/livox_ros_driver2/launch_ROS2/rviz_MID360_launch.py`** | ROS 2 launch: passes params into the driver node. | **`frame_id`** (e.g. `livox_frame`), **`publish_freq`**, **`xfer_format`** (PointCloud2 vs custom), path to **`user_config_path`** → `MID360_config.json`. |

### 7.5 FAST-LIO (`fast_lio` package in `FAST_LIO_ROS2`)

| File | What it is | What you tune there |
|------|------------|---------------------|
| **`src/FAST_LIO_ROS2/config/mid360.yaml`** | **Primary** runtime config when using `config_file:=mid360.yaml` or **`lio_bringup`** (merged with overlay). | **`lid_topic`**, **`imu_topic`**, preprocess (Livox type, blind, rates), **mapping** covariances & **`det_range`**, **IMU–LiDAR extrinsics** (`extrinsic_T`, `extrinsic_R`, `extrinsic_est_en`), **publish** flags (scans, map, path). **`publish_tf`** (node param): set **`false`** when **`ekf_node`** publishes **`odom`→`base_link`** (default in **`lio_bringup`** overlay). |
| **`src/FAST_LIO_ROS2/config/avia.yaml`**, **`horizon.yaml`**, **`mid360.yaml`**, **`ouster64.yaml`**, **`velodyne.yaml`** | Presets for different LiDARs. | Same YAML structure as `mid360.yaml`; pick matching sensor + fix topics. |
| **`src/FAST_LIO_ROS2/launch/mapping.launch.py`** | Entry launch: `config_path`, `config_file`, `use_sim_time`, RViz on/off and RViz config path. | Which YAML is loaded; sim time; default RViz layout. |
| **`src/FAST_LIO_ROS2/rviz/fastlio.rviz`** | RViz displays for FAST-LIO. | Visualization only (topics, styles, fixed frame `camera_init`). |

### 7.6 LIO bringup relay (`lio_bringup`)

| File | What it is | What you tune there |
|------|------------|---------------------|
| **`src/lio_bringup/launch/lio_backend.launch.py`** | **`fastlio_mapping`** + **`lio_odom_relay_node`**. | Uses **`fast_lio`** **`mid360.yaml`** + overlay (see next row). |
| **`src/lio_bringup/config/fastlio_mid360_overlay.yaml`** | **Primary LIO tuning file** for this workspace: **`publish_tf: false`**, PointCloud2 (**`lidar_type: 4`**), **`det_range`**, IMU covariances, **`pcd_save` off**, ICP depth. Top-of-file comments list tuning order. | **`extrinsic_T` / `extrinsic_R`**, **`det_range`**, **`blind`**, **`acc_cov` / `gyr_cov`**. See **`src/LiDAR-Instructions.md` §4**. |

### 7.7 Bringup & EKF-centric RViz (`ros_project_bringup`)

| File | What it is | What you tune there |
|------|------------|---------------------|
| **`src/ros_project_bringup/launch/launch_slam.launch.py`** | Livox (optional) + static TFs + **`ekf_node`** + optional NDT **or** **`lio_bringup`** + optional **`keyframe_map_node`** + optional **`pose_graph_node`** + RViz (optional). | Adds **`start_pose_graph`**, **`keyframe_loop_closure_enable`**, plus NDT/LIO/EKF args. |
| **`src/ros_project_bringup/rviz/slam.rviz`** | RViz for bringup / EKF / Livox / keyframe map. | Fixed frame **`map`**, **`/keyframe_map`**, **`/keyframe_map/keyframes`**, point style/decay, grid. |

### 7.8 Project-level guides (no runtime effect)

| File | Purpose |
|------|---------|
| **`README.md`** (this file) | Workspace overview, run instructions, tuning map. |
| **`src/LiDAR-Instructions.md`** | FAST-LIO setup, TF chain, EKF wiring, progression. |
| **`src/localisation_ekf/localisation_ekf/README.md`** | EKF behavior & parameters in prose. |
| **`src/lidar_odometry/README.md`** | NDT node behavior & parameters. |
| **`src/keyframe_scan_map/README.md`** | Keyframe map topics & tuning. |
| **`src/FAST_LIO_ROS2/README.md`** | Upstream FAST-LIO ROS 2 notes. |

---

## 8. Where to tune what (parameter quick reference)

### 8.1 Custom EKF — `localisation_ekf` / `config/ekf_python.yaml`

Loaded by **`launch_slam.launch.py`**, which **merges** `slam_bringup.yaml` overrides (LiDAR topic enable/disable, variances, etc.) on top of this file. Set **`use_lidar_fusion:=false`** and **`use_lio:=false`** in bringup to force **`lidar_odom_topic: ""`** regardless of `ekf_python.yaml` defaults.

| Parameter | Meaning |
|-----------|--------|
| **`imu_topic`** | IMU for **prediction** |
| **`nominal_dt`**, **`use_stamp_dt`** | IMU integration timestep |
| **`odom_frame`**, **`base_link_frame`** | Published `/ekf/odom` and TF parent/child |
| **`publish_tf`** | Broadcast `odom` → `base_link` |
| **`process_noise_diag`** (length **9**) | **Q** (per second, scaled by Δt in filter): order `px, py, z, yaw, vx, vy, bax, bay, bgz` |
| **`initial_cov_diag`** (length **9**) | Initial **P** diagonal |
| **`lidar_odom_topic`**, **`lidar_pose_topic`**, **`lidar_z_topic`** | Measurement inputs (empty string = disabled) |
| **`lidar_pose_var`** | Measurement variance for **x, y** (and **z** when z fusion from odom is on) |
| **`lidar_yaw_var`** | Variance on **yaw** measurement (smaller ⇒ trust LiDAR heading more) |
| **`lidar_z_var`** | z-only update variance |
| **`lidar_fuse_z_from_odom`** | `false`: fuse **x, y, yaw** only from LiDAR odom (avoids pulling z to 0 from planar NDT). `true`: fuse **z** too (e.g. FAST-LIO with meaningful z) |
| **`lidar_gate_nis`** | Mahalanobis gate on LiDAR updates |
| **`lidar_require_frames`** | Warn if `/lidar/odom` frames are not `odom` / `base_link` |

**Defaults (accuracy-first):** tight LiDAR measurement variances (**`lidar_pose_var`** 0.015, **`lidar_yaw_var`** 0.008) and a **higher** **`lidar_gate_nis`** (32) so those updates are not dropped too often. For a smoother (less jumpy) estimate, **increase** the variances and/or **lower** the gate; see §11.

**Behaviour:** After a LiDAR correction, published `/ekf/odom` and TF use the **LiDAR message stamp**; IMU `last_imu_stamp` is **not** overwritten so IMU **dt** stays consistent.

More detail: **`src/localisation_ekf/localisation_ekf/README.md`**.

### 8.2 Bringup launch — `ros_project_bringup/launch/launch_slam.launch.py`

- Launch args: **`use_lidar_fusion`**, **`use_lio`**, **`start_livox_driver`**, **`start_rviz`**, **`start_keyframe_map`**, **`keyframe_loop_closure_enable`**, **`start_pose_graph`**, **`ekf_lidar_fuse_z_from_odom`** (see §5.3).  
- Static transforms: **`map`→`odom`**, **`base_link`→`livox_frame`**.  
- When **`use_lio:=true`**: includes **`lio_bringup`** (FAST-LIO + relay); NDT is not started; EKF **`lidar_odom_topic`** is **`/lidar/odom`**.  
- When **`use_lidar_fusion:=true`** and **`use_lio:=false`**: starts **`lidar_odometry_node`** and sets EKF **`lidar_odom_topic`** to **`/lidar/odom`**.  
- **`ekf_lidar_fuse_z_from_odom`** default **`auto`**: **`true`** with **`use_lio`** (fuse **z** from LIO), **`false`** on NDT-only runs.  
- When **`start_keyframe_map:=true`**: starts **`keyframe_map_node`** with **`keyframe_map.yaml`** plus **`keyframe_loop_closure_enable`** override.  
- When **`start_pose_graph:=true`** (and keyframe map on): starts **`pose_graph_node`** (**`pose_graph.yaml`**).  
- EKF merge dict also sets **`lidar_pose_var`**, **`lidar_yaw_var`**, **`lidar_gate_nis`**, **`lidar_fuse_z_from_odom`**.  
- RViz: **`rviz/slam.rviz`** (includes **`/lidar/odom`**, **`/ekf/*`**, **`/keyframe_map`**).

### 8.3 NDT odometry — `lidar_odometry`

- **Launch defaults:** `launch/lidar_odometry.launch.py` or **`launch_slam.launch.py`** (NDT args + **`lidar_registration_mode`**, **`lidar_map_*`**).  
- **Tunable:** voxel size, crop range, NDT resolution / iterations / fitness threshold, `publish_tf`, **`registration_mode`** (`scan_to_scan` vs **`scan_to_map`**), map merge / max points, Livox extrinsic (wired to **`sensor_extrinsic_rpy_xyz`** in bringup).  

Full table: **`src/lidar_odometry/README.md`**.

### 8.4 FAST-LIO — `FAST_LIO_ROS2/config/mid360.yaml` (and others)

- **`common.lid_topic`**, **`imu_topic`** — must match Livox  
- **`mapping`** — covariances, `det_range`, **extrinsics** (`extrinsic_T`, `extrinsic_R`, `extrinsic_est_en`)  
- **`preprocess`** — Livox type, blind distance, rates (with **`lio_bringup`**, overlay sets **`lidar_type: 4`** for PointCloud2)  
- **`publish`** — which clouds / path / map to publish  
- **`publish_tf`** — if **`false`**, FAST-LIO does not broadcast **`camera_init`→`body`** (use with EKF **`publish_tf: true`** on **`odom`→`base_link`**)  

RViz layout: **`src/FAST_LIO_ROS2/rviz/fastlio.rviz`**.

### 8.5 RViz

| File | Typical use |
|------|-------------|
| `ros_project_bringup/rviz/slam.rviz` | EKF / Livox-centric SLAM view, fixed frame `map` |
| `FAST_LIO_ROS2/rviz/fastlio.rviz` | FAST-LIO, fixed frame `camera_init` |

### 8.6 Driver

- **`src/livox_ros_driver2/config/MID360_config.json`**  
- Launch parameters in **`launch_ROS2/rviz_MID360_launch.py`** (e.g. `frame_id`, `publish_freq`)

---

## 9. Design summary (for reports / debugging)

- **EKF state (9):** `px, py, z, yaw, vx, vy, bax, bay, bgz` — planar motion with IMU biases; **roll/pitch** not estimated in the filter output.  
- **Prediction:** IMU accelerometer + gyro (planar).  
- **Correction:** LiDAR pose as absolute measurement on **`x, y, yaw`**; optional **`z`** via odom or **`/lidar/z`**.  
- **NDT node:** consecutive-scan alignment; **planar** increment from **T_ndt⁻¹**; integrated pose is yaw + xy only in published `/lidar/odom`.

---

## 10. Further reading

| Document | Content |
|----------|--------|
| **`src/LiDAR-Instructions.md`** | FAST-LIO clone/build, launch order, TF, EKF wiring, tuning progression |
| **`src/localisation_ekf/localisation_ekf/README.md`** | EKF modes, parameters, TF convention |
| **`src/lidar_odometry/README.md`** | NDT pipeline, parameters, limitations |
| **`src/FAST_LIO_ROS2/README.md`** | Upstream FAST-LIO ROS 2 notes |

---

## 11. Troubleshooting (short)

| Symptom | Things to check |
|---------|-----------------|
| No TF | `publish_tf` on `ekf_node`; static publishers in `launch_slam`; frame names in RViz |
| EKF diverges / jumps | `lidar_pose_var` / `lidar_yaw_var`, `lidar_gate_nis`, `process_noise_diag` |
| Heading drifts | Lower **`lidar_yaw_var`**; ensure LiDAR odom yaw is meaningful and fused |
| z collapses to 0 | Keep **`lidar_fuse_z_from_odom: false`** for planar NDT; use **`/lidar/z`** or FAST-LIO with z fusion enabled |
| NDT never updates | Point count after crop/voxel; **`max_fitness_score`**; **`ndt_resolution`** vs scene scale |
| Position drifts, yaw OK | Repeated **NDT fitness above max** (skip warnings) → no **`/lidar/odom`** → EKF is IMU-only; **raise `max_fitness_score`** (see **`lidar_odometry` README**) |
| **`/keyframe_map` bent / smeared / drifts** | Map follows **odometry**; reduce NDT skips (**`max_fitness_score`**), tune NDT (**`ndt_resolution`**, **`voxel_leaf_size`**), use **FAST-LIO** (or similar) for **`lidar_odom_topic`**, calibrate **`base_link`→`livox_frame`**; see **`src/keyframe_scan_map/README.md`** |
| Wrong motion direction | NDT increment convention + physical extrinsics; verify forward/left signs on `/lidar/odom` |

---

*Last updated to reflect the packages and parameters in this workspace; after large refactors, rebuild and re-check topic names with `ros2 topic list`.*
