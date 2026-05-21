# ROS_Deploy_Orin

ROS 2 (Humble) workspace: **Livox MID360**, **planar EKF**, optional **PCL NDT** or **FAST-LIO** LiDAR odometry, and **keyframe map** + pose graph.

---

## Build

```bash
cd ~/ROS_Deploy_Orin
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

---

## Run (main entry)

**Live robot** (sensors on this machine):

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ros_project_bringup launch_slam.launch.py launch_sensors:=true
```

**Bag replay** (sensors already in the bag or on another host):

```bash
ros2 launch ros_project_bringup launch_slam.launch.py use_sim_time:=true launch_sensors:=false
ros2 bag play ~/bags/your_session --clock
```

If recorded TF fights the stack, use `python3 scripts/bag_play_no_recorded_tf.py` — see **`scripts/README.md`**.

**Headless** (no RViz on the robot):

```bash
ros2 launch ros_project_bringup launch_slam.launch.py start_rviz:=false
```

**FAST-LIO instead of NDT** (still feeds `/lidar/odom` for the EKF):

```bash
ros2 launch ros_project_bringup launch_slam.launch.py use_lio:=true use_lidar_fusion:=false
```

**IMU-only EKF** (no LiDAR odometry updates):

```bash
ros2 launch ros_project_bringup launch_slam.launch.py use_lio:=false use_lidar_fusion:=false
```

---

## Launch parameters (defaults)

Empty launch args mean “use **`src/ros_project_bringup/config/slam_bringup.yaml`**”. The table below is what you should assume when **not** passing overrides (documented product defaults).

| Launch argument | Default | Effect |
|-----------------|---------|--------|
| **`use_lio`** | **`false`** | `true` → FAST-LIO + relay to `/lidar/odom`; `false` → NDT if fusion is on |
| **`use_lidar_fusion`** | **`true`** | `true` → start NDT when `use_lio` is false; `false` → no NDT |
| **`launch_sensors`** | **`false`** | `true` → Livox + Microstrain drivers and sensor static TFs on this host |
| **`use_sim_time`** | **`false`** | `true` → follow `/clock` (bag replay) |
| **`start_rviz`** | **`false`** | `true` → RViz with `slam.rviz` |
| **`start_keyframe_map`** | **`true`** | Merged map `/keyframe_map` |
| **`start_pose_graph`** | **`true`** | Pose graph (needs keyframe map) |
| **`bringup_config`** | *(empty)* | Path to YAML merged over `slam_bringup.yaml`, or set **`ROS_PROJECT_SLAM_CONFIG`** |

**Odometry source when using defaults above:** **NDT** (`lidar_odometry`) → **`/lidar/odom`** → **EKF** → **`/ekf/odom`** and TF **`odom` → `base_link`**.

If both **`use_lidar_fusion`** and **`use_lio`** are true, **LIO wins** (NDT is not started).

---

## What to edit for tuning

| File | Use for |
|------|---------|
| **`src/ros_project_bringup/config/slam_bringup.yaml`** | Stack toggles, NDT/EKF/keyframe, Livox/Microstrain, extrinsics |
| **`readme/TUNING.md`** | Deskew, map shear, parameter cheat sheet |
| **`src/localisation_ekf/config/ekf_python_gx5_microstrain.yaml`** | EKF process noise (Microstrain preset) |
| **`src/lio_bringup/config/fastlio_mid360_overlay.yaml`** | FAST-LIO overlay when `use_lio:=true` |

Rebuild after YAML changes: `colcon build --packages-select <package>` then re-source `install/setup.bash`.

---

## Useful topics

| Topic | Producer |
|-------|----------|
| `/livox/lidar` | Livox driver |
| `/lidar/odom` | NDT or LIO relay |
| `/ekf/odom` | EKF fused pose |
| `/keyframe_map` | Keyframe map node |

---

## Scripts and more detail

- **`scripts/README.md`** — bag compare, diagnostics, recording helpers
- **`readme/deployment.md`** — deployment on Jetson / field robot
- **`readme/autorun.md`** — autorun / startup notes
- **`readme/comments.md`** — extracted source comments (developer reference)
- **`src/LiDAR-Instructions.md`** — FAST-LIO topics, extrinsics, TF
- **`src/ros_project_bringup/docs/STACK_OVERVIEW.md`** — node graph (in package share after build)
