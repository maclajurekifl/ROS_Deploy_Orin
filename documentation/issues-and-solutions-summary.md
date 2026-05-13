# Issues encountered and solutions (summary)

This document summarizes problems that showed up while integrating the **Lord Microstrain GX5-25** IMU with **Livox MID360** LiDAR, the **Python planar EKF** (`localisation_ekf`), and **keyframe mapping** (`keyframe_scan_map`). It is a high-level record, not an exhaustive troubleshooting guide (see `readme/TUNING.md` for parameter-level notes).

---

## 1. Very low `/imu/data_raw` publish rate (~1 Hz)

**Issue:** `ros2 topic hz /imu/data_raw` reported ~1 Hz even though the sensor is capable of much higher rates.

**Cause:** The `microstrain_inertial_driver` ships a default `params.yml` where **`imu_data_raw_rate`** is set to **1 Hz**, while **`imu_data_rate`** (filtered) defaults to 100 Hz. Bringup loaded that file and only overrode port/baud/frame IDs.

**Solution:** Override **`imu_data_raw_rate`** in bringup (e.g. **200 Hz** to align with EKF **`nominal_dt: 0.005`**). Implemented via **`microstrain_imu_data_raw_rate`** in `launch_slam.launch.py` → `ms_params['imu_data_raw_rate']`.

The filtered topic **`/imu/data`** (`imu_data_rate` in the driver) was left at the vendor default (100 Hz) until explicitly overridden. If you only subscribe to **`/imu/data_raw`**, set **`microstrain_imu_data_rate`** to **0** so the driver does not stream **`/imu/data`** (saves serial bandwidth and a bit of CPU). This is now the default in `config/slam_bringup.yaml`.

---

## 2. Motion / prediction felt worse with IMU enabled; vertical bumps and jolts

**Issue:** With IMU fusion enabled, motion could feel less stable; vertical motion or sharp jolts hurt prediction and corrections sometimes struggled to recover.

**Cause:** The stack uses a **planar EKF** (`EKFPlanarIMU`): it integrates **body x/y linear acceleration** into world velocity using **yaw only** and assumes **no roll/pitch** in the model. Real bumps change attitude briefly; gravity and vertical dynamics leak into horizontal acceleration components, so integrating raw accel between LiDAR updates injects error. **Z** is not propagated from the IMU in this filter.

**Solution:** Added **`predict_use_linear_accel`** (default **true** for backward compatibility). When **false**, the filter uses **gyro-only translation** (constant-velocity coast between LiDAR updates; yaw still from gyro). Enabled for the Microstrain preset in **`ekf_python_gx5_microstrain.yaml`**. LiDAR odometry continues to correct **x, y, yaw, vx, vy**.

---

## 3. Map skew / distortion, especially when rotating

**Issue:** Merged keyframe maps showed shear or smearing; rotation was a common trigger. Using Microstrain for the EKF while deskew used **`/livox/imu`** is fine, but **gyro must be expressed in the LiDAR cloud frame** before Rodrigues deskew.

**Causes (rough):**

- **Frame mismatch for deskew:** Points are in **`livox_frame`** (cloud `header.frame_id`). Livox **`/livox/imu`** must be rotatable into that frame (this repo’s Livox driver stamps IMU with the same **`frame_id`** as the cloud). With **`deskew_imu_rotate_gyro_to_frame`** empty, keyframe used **raw ω** in the wrong basis → **rotation shear** and streaking.
- **Missing IMU bridge in TF** during bag / DDS-only bringup (Livox driver off): no static link from cloud frame to the IMU message’s `frame_id` → deskew TF rotation fails. Launch publishes identity **`livox_frame`→`livox_imu`** by default (**not** **`sensor`** — that name collides with Microstrain **`/imu/data`**).
- **Planar EKF + accel** (issue §2): Bad horizontal prediction between scans worsens TF and map alignment.
- **Timing:** IMU vs LiDAR clock skew can hurt deskew and stamped TF.

**Solution (implemented in bringup):**

- Set **`keyframe_deskew_imu_rotate_gyro_to_frame`** to **`livox_frame`** (same as **`livox_cloud_frame_id`**) in **`slam_bringup.yaml`** / **`keyframe_map.yaml`** so ω is rotated via TF2 before deskew.
- When the Livox driver is **not** started on this machine (`launch_sensors:=false`, typical laptop + bag), **`launch_slam.launch.py`** publishes **identity** static TF **`livox_frame`→`livox_imu`** (override with **`livox_imu_child_frame`**) if **`publish_livox_imu_sensor_frame_tf: true`** (default). Using child frame **`sensor`** here breaks Microstrain when **`/imu/data`** uses **`header.frame_id: sensor`**. Disable the flag only if you already publish an equivalent bridge on DDS.
- **`keyframe_deskew_imu_follow_ekf: true`** still repoints deskew IMU to the EKF topic (GX5); use only when you intentionally deskew with the same IMU as the EKF and understand Livox scan timing vs that IMU.
- Extrinsics (**`imu_mount_*`**, **`livox_extrinsic_*`**) must stay consistent for EKF (`imu_link`→`base_link`) and NDT.

---

## 4. Ancillary: Fast DDS XML parser warning

**Issue:** Console messages such as `[XMLPARSER Error] realpath failed ... loadDefaultXMLFile` when running ROS 2 tools.

**Cause:** Fast DDS looking for a default XML profile path that is not present in the environment.

**Impact:** Usually **cosmetic** unless a custom DDS XML configuration is required.

---

## 5. Documentation layout in this repo

| Location | Purpose |
|----------|---------|
| **`documentation/issues-and-solutions-summary.md`** (this file) | Narrative of issues and fixes at stack level |
| **`readme/TUNING.md`** | Practical tuning parameters and tables |
| **`src/ros_project_bringup/config/slam_bringup.yaml`** | primary runtime bringup knobs (merged with optional overlay) |

Rebuild and re-source the workspace after changing launch or package YAML.

---

## 6. Microstrain publishing on `/imu/data_raw` but EKF still using Livox IMU

**Issue:** `ros2 topic echo /imu/data_raw` shows good data, yet fusion behavior matches **Livox** IMU or ignores Microstrain entirely.

**Cause:** In `launch_slam.launch.py`, **`use_microstrain_imu`** must be **`True`** for the launch logic to set **`imu_topic`** to **`microstrain_imu_topic`** (default **`/imu/data_raw`**). If it is **`False`**, **`imu_topic`** stays on **`ekf_imu_topic`** (default **`/livox/imu`**). The EKF never subscribes to Microstrain regardless of YAML filenames.

**Solution:** Set **`use_microstrain_imu: True`**, set **`ekf_params_yaml`** to **`config/ekf_python_gx5_microstrain.yaml`**, rebuild/install if needed, and confirm at runtime: **`ros2 param get /ekf_node imu_topic`** → expect **`/imu/data_raw`** (or your override).

---

## 7. Stale parameters after editing YAML in `src/` only

**Issue:** Changed **`config/ekf_python_gx5_microstrain.yaml`** (or other package config) but runtime behavior is unchanged.

**Cause:** Bringup resolves config paths with **`get_package_share_directory`** → the **install** tree **`share/<pkg>/config/`**, not the **`src/`** tree. Until **`colcon build`** (or **`colcon install`**) copies sources into **`install/`**, the running node can load an older file.

**Solution:** Rebuild the affected package and **`source install/setup.bash`** before **`ros2 launch`**. Quick check: compare timestamps under **`install/<pkg>/share/.../config/`** vs **`src/.../config/`**.

---

## 8. `/ekf/odom` and `odom→base_link` header stamps stuck between LiDAR frames (high-rate IMU)

**Issue:** With LiDAR fusion + Livox, the launch stack sets **`lidar_cloud_stamp_topic`** so **`ekf_node`** can align outputs with **`PointCloud2`** time. Between clouds, **`publish_outputs`** may stamp **`/ekf/odom`** and TF using the **last cloud** time repeatedly while IMU callbacks run at hundreds of Hz. Tools (and **`tf2`**) that assume **monotonic, unique stamps** can then show a **frozen** pose or confusing TF even though the internal filter state is updating.

**Mitigation (conceptual):** Stamp **IMU-driven** publishes with the **IMU message time** when you need strict time progression between clouds; keep **cloud-driven** publishes at LiDAR time for RViz / MessageFilter alignment. This is a design trade-off rather than a one-line toggle in the current tree—worth knowing when debugging “IMU looks dead” in RViz or TF monitors.

*(Related: §3 already mentions IMU vs LiDAR **clock skew** for deskew; this section is about **which stamp** is written on **`/ekf/odom`** when cloud sync is enabled.)*

---

## 9. NDT warning: TF `odom→base_link` at cloud time unavailable

**Issue:** **`lidar_odometry_node`** logs that **`odom→base_link`** at the **point cloud stamp** is missing, then falls back to the last NDT pose.

**Causes (typical):** **`ekf_node`** and NDT both subscribe to **`/livox/lidar`**—who processes first is undefined, so the EKF may not yet have published **`odom→base_link`** at that exact stamp when NDT looks it up. Separately, **stamp mismatch** between Livox cloud time and the EKF TF timeline (different clocks vs Microstrain) can make a stamped lookup fail even when both nodes are up.

**Mitigations:** Increase **`lidar_tf_initial_guess_timeout_sec`** in **`slam_bringup.yaml`** to give the buffer longer to receive the EKF transform; tune **`ekf_imu_stamp_offset_sec`** / **`ekf_lidar_stamp_offset_sec`** if Livox and Microstrain clocks diverge. A code-level fallback (stamped lookup, then **latest** `odom→base_link`) is an optional hardening path if timeouts alone are insufficient.

---

## 10. Startup: EKF warns that TF from `imu_link` to `base_link` failed

**Issue:** One-time (or early) warning that **`base_link`** or the transform to **`imu_link`** was missing during **`lookupTransform`**.

**Cause:** **`ekf_node`** rotates IMU vectors into **`base_link`** using static **`base_link`→`imu_link`**. If IMU messages arrive before those transforms are in the TF buffer, or **`imu_tf_lookup_timeout_sec`** is too short for the first lookup, the node falls back to **raw** IMU components in the sensor frame until TF succeeds.

**Mitigations:** Ensure **`base_link`→`imu_link`** static TF is started (Microstrain path in **`launch_slam`**); optionally increase **`imu_tf_lookup_timeout_sec`** in the EKF YAML; avoid relying on raw components for long if extrinsics matter.

---

## 11. “IMU has no effect” while `/lidar/odom` runs at high rate

**Issue:** Rotating or moving the robot does not seem to change **`/ekf/odom`** in the way IMU alone would suggest.

**Cause:** With **NDT (or LIO) + EKF**, LiDAR odometry applies a **strong measurement update** every scan (**`ekf_lidar_pose_var`**, **`ekf_lidar_yaw_var`**, **`ekf_lidar_gate_nis`**). The visible pose is dominated by **LiDAR**; IMU mainly shapes **prediction between** scans and **smoothness**. Expecting open-loop IMU-like motion on **`/ekf/odom`** while fusion is on is misleading.

**Mitigation:** To isolate IMU behavior, run **IMU-only** bringup (**`use_lidar_fusion`** / **`use_lio`** off so **`lidar_odom_topic`** is empty) temporarily, or loosen LiDAR variances only for experiments—not for production without retuning.

