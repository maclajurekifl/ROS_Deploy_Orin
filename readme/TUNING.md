# SLAM stack tuning notes (EKF, Microstrain, Livox, keyframe map)

Primary config: **`src/ros_project_bringup/config/slam_bringup.yaml`**. Optional overlay: launch arg **`bringup_config:=/path/to.yaml`** or env **`ROS_PROJECT_SLAM_CONFIG`**.

## Livox `frame_id` and deskew gyro rotation

If you run:

```bash
ros2 topic echo /livox/lidar --once
```

and see `header.frame_id: livox_frame`, that **matches** the defaults in `ros_project_bringup/launch/launch_slam.launch.py`:

- `livox_cloud_frame_id`: `livox_frame`
- With Microstrain + `keyframe_deskew_imu_follow_ekf`, deskew automatically rotates IMU angular velocity from `imu_link` (Microstrain) into **`livox_frame`** so per-point deskew uses the same frame as the point cloud.

If you change the Livox driver `frame_id`, set **`livox_cloud_frame_id`** (and/or **`keyframe_deskew_imu_rotate_gyro_to_frame`**) to the same string.

### Fast DDS XML warning

A line like `[XMLPARSER Error] realpath failed No such file or directory -> Function loadDefaultXMLFile` comes from the **Fast DDS** default XML profile lookup, not from Livox or deskew. It is usually harmless unless you rely on a custom DDS XML file.

---

## Map shear / rotation distortion (checklist)

1. **Gyro vs LiDAR frame** — External IMU deskew must rotate ω into the LiDAR frame (see above). Mismatch → shear when turning.
2. **Extrinsics** — `imu_mount_*` and `livox_extrinsic_*` in `config/slam_bringup.yaml` should match the physical rig (`base_link` → `imu_link`, `base_link` → `livox_frame`).
3. **Time sync** — If shear appears mostly while rotating, tune **`ekf_imu_stamp_offset_sec`**, **`keyframe_deskew_imu_stamp_offset_sec`**, **`keyframe_deskew_cloud_stamp_offset_sec`** so Microstrain and Livox share one timeline.
4. **Planar EKF** — **`predict_use_linear_accel: false`** in `ekf_python_gx5_microstrain.yaml` avoids integrating horizontal accel through bumps (gyro-only translation between LiDAR updates).

---

## Parameter tuning guide

| Goal | What to try |
|------|-------------|
| **Frame names** | Keep **`livox_cloud_frame_id`** equal to `/livox/lidar` `header.frame_id`. Set **`keyframe_deskew_imu_rotate_gyro_to_frame`** explicitly if you do not use the launch auto-default, or use **`disabled`** to skip rotation when deskew uses Livox `/livox/imu` only. |
| **Extrinsics** | Refine **`livox_extrinsic_*`** and **`imu_mount_*`** (CAD or calibration). Bad TF → wrong deskew rotation and wrong EKF IMU transform. |
| **Time sync** | Adjust **`ekf_imu_stamp_offset_sec`** / **`keyframe_deskew_*_stamp_offset_sec`** for clock skew between IMU and LiDAR. |
| **Deskew model** | **`keyframe_deskew_model`**: `yaw_only` favors flat yaw rotation (ωz only in the LiDAR frame after gyro rotation). `rodrigues` uses full 3D ω (better if the sensor tilts a lot). |
| **Deskew sign** | If the merged map twists the wrong way when spinning, try **`keyframe_deskew_imu_sign`**: `-1.0`. |
| **More keyframes while turning** | Lower **`keyframe_min_yaw_deg`** (e.g. 3–4°), lower **`keyframe_rotation_keyframe_scale`** (e.g. 0.35–0.4), or tune **`keyframe_rotation_gyro_z_thresh_rad_s`** so adaptive spacing triggers earlier. |
| **NDT (scan-to-map)** | **`lidar_ndt_resolution`**, **`lidar_voxel_leaf_size`**, **`lidar_crop_range_m`**, **`lidar_max_fitness_score`** — rotation is demanding; slightly finer resolution or stricter fitness can help at the cost of CPU. |
| **EKF vs LiDAR** | **`ekf_lidar_yaw_var`** (tighter → LiDAR dominates heading), **`ekf_lidar_pose_var`**, **`ekf_lidar_gate_nis`** if updates are rejected (watch `ekf_node` warnings). |
| **Microstrain raw rate** | **`microstrain_imu_data_raw_rate`** in `launch_slam.launch.py` (driver default for `/imu/data_raw` was 1 Hz; override to e.g. 200 Hz to match EKF **`nominal_dt`**). |
| **Microstrain filtered `/imu/data`** | **`microstrain_imu_data_rate`**: default **0** = driver does not stream filtered IMU (saves bandwidth if you only use **`/imu/data_raw`**). Set e.g. **100** if another node needs **`/imu/data`**. |

---

## Where parameters live

- **Global bringup**: `src/ros_project_bringup/config/slam_bringup.yaml` (optional overlay: `bringup_config`, `ROS_PROJECT_SLAM_CONFIG`).
- **EKF (Microstrain preset)**: `src/localisation_ekf/config/ekf_python_gx5_microstrain.yaml`.
- **Keyframe map defaults**: `src/keyframe_scan_map/config/keyframe_map.yaml` (overridden by launch).

Rebuild after YAML or Python changes: `colcon build` for the touched packages and re-source the workspace.
