# Comments archive

Notes removed from `src/` to keep configs and scripts minimal.
See `docs/tuning.md` for tuning knobs; this file is the former inline prose.

## `src/keyframe_scan_map/config/keyframe_map.yaml`

### header

keyframe_map_node — knobs = docs/tuning.md §6–7; rest below.

### ros__parameters

=========================================================================
KNOBS — only parameters listed in docs/tuning.md §6–7
=========================================================================

### max_pts_per_scan

=========================================================================
REST
=========================================================================

### lidar_odom_topic

Used when lidar_odom_approximate_sync is false (nearest-neighbor fallback).

## `src/keyframe_scan_map/config/pose_graph.yaml`

### header

pose_graph_node — knobs = docs/tuning.md §8; rest below.

### ros__parameters

=========================================================================
KNOBS — only parameters listed in docs/tuning.md §8
=========================================================================

### max_loop_edges

=========================================================================
REST
=========================================================================

## `src/keyframe_scan_map/keyframe_scan_map/keyframe_map_node.py`

### module docstring

Keyframe scan map: optional Livox per-point ``timestamp`` + IMU **3D** deskew in the **LiDAR**
``header.frame_id`` (typically ``livox_frame``). External IMUs (e.g. Microstrain in ``imu_link``)
must have angular velocity rotated into that frame via ``deskew_imu_rotate_gyro_to_frame`` / TF.
then transform each cloud to `map`, keep scans that pass a distance/yaw/time keyframe rule,
merge and voxel-downsample, publish a single PointCloud2.

When ``use_lidar_odom_for_robot_pose`` is true, optional **approximate time sync** pairs each
cloud with ``/lidar/odom`` so pose exists before insertion (see ``lidar_odom_approximate_sync``).

Optional **simple loop closure** (`loop_closure_enable`) and optional **pose-graph map rebuild**
(`apply_pose_graph_corrections`): when `/pose_graph/corrected_keyframes` matches this node's
keyframe count, each stored map batch is transformed **T_new * inv(T_old)** per keyframe, poses
and `/keyframe_map` are rebuilt, and `/keyframe_map/keyframes` republished (Option 1+2). Use
`pose_graph_node` Option 3 (`publish_map_odom_tf`) only **without** map rebuild to avoid double
correction — see README.

### pose_to_matrix4_from_odometry

4x4: p_parent = T @ p_child (same convention as transform_matrix_from_stamped).

### transform_points_xyz

pts (N,3), m 4x4 -> (N,3)

### docstring

Voxel-centroid xyz + per-voxel **max** intensity (newest keyframe index wins in cell).

### statistical_outlier_mask

Keep points with kNN mean distance <= mean + std_mul*std.

### points_from_cloud2

Livox (and others) often use float32 x,y,z with padded point_step; read_points returns a structured array.

### docstring

Return xyz plus optional intensity and optional Livox timestamp fields.

### docstring

Deskew with per-row angular velocity ``omega`` (N,3) and ``dt_sec`` (N,) to scan end.

### docstring

Motion-compensate points in the **sensor** frame toward the end of the frame (Livox ``timestamp``).

    Livox IMU gyro is in ``livox_frame``; points are in the same frame. Uses constant angular
    velocity over the frame: **yaw_only** rotates x,y by ωz·dt; **rodrigues** applies the exact
    rotation about axis ω̂ for angle ‖ω‖·dt per point (handles tilting / off-axis spins).

### docstring

Per-point absolute time (ns) aligned with ``header.stamp`` sweep end + offset.

### interp_gyro_batch

Linear interpolation of angular velocity (N,3) at query times (N,) from (M,) / (M,3).

### docstring

If ``intensity`` is set (length N), adds ``intensity`` field for RViz rainbow (FAST-LIO style).

### docstring

Fraction of subsampled points in a with a neighbor in b within match_m (both map frame).

### _rotation_matrix_from_axis_angle

Rodrigues formula for a 3x3 rotation matrix.

### _gyro_mean_for_timespan

Mean angular velocity over IMU samples in [t_lo, t_hi] (with margin) — stable under spin.

### docstring

Express angular velocity in ``deskew_imu_rotate_gyro_to_frame`` (LiDAR frame).

### _deskew_imu_fresh_wall

Legacy gate: IMU arrived recently in wall time (cloud stamp missing or broken).

### _deskew_imu_usable_for_cloud

Gate deskew on IMU vs *cloud* time alignment, not processing latency.

        Using only wall-clock ``now()`` caused deskew to be skipped whenever the cloud
        callback ran slightly late → motion-smearing persisted while rotating even though
        gyro was valid at capture time.

### docstring

Return transform target_frame <- source_frame at ``stamp``, with optional fallbacks.

### _rebuild_merged_map_from_batches

Voxel-merge ``_kf_map_batches`` into ``_map_pts`` (pose-graph mode only).

        When pose-graph corrections are enabled, the published map must come **only** from
        stored keyframe batches so it stays consistent with warped geometry. Incrementally
        stacking full-resolution ``pts_map`` duplicates points vs batch-based merges and
        produces layered walls.

### _apply_pose_graph_path_msg

Apply corrected poses to batches and republish map (all lengths must match).

### _try_apply_pending_pose_graph

Apply stashed /pose_graph/corrected_keyframes once keyframe count catches up.

### top

!/usr/bin/env python3

### top

Livox (and most rclcpp default publishers) use Reliable; sensor_data (Best Effort) will not match.

### top

Small depth: long deskew callbacks + ApproximateTime sync must not backlog many seconds of clouds.

### top

Match external IMU drivers (Microstrain, etc.): best-effort high rate.

### deskew_points_to_scan_end

Livox driver stores offset_time as double (typically ns from frame start).

### overlap_ratio

(n, 1, 3) - (1, M, 3) -> (n, M) distances

### __init__

Min wall time between full /keyframe_map publishes (map still merges every keyframe).

### __init__

Skip first N point clouds before any keyframe (TF/LIO/FAST-LIO IMU init — avoids ghost first scan).

### __init__

If false, skip keyframe when TF at cloud stamp is missing (avoids livox vs pose mismatch).

### __init__

Use /lidar/odom (or NDT/LIO topic) for T_odom<-base at cloud time; only TF for map<-odom

### __init__

and base<-livox extrinsic. Avoids EKF lag / sparse odom TF at scan insertion.

### __init__

Pair cloud + /lidar/odom by stamp (avoids processing cloud before matching odom arrives).

### __init__

Prefilter in sensor frame to remove self-hits / distant clutter before map insertion.

### __init__

Skip deskew update when gyro norm is implausibly large (protect against spikes).

### __init__

If strict TF (above false), still use latest TF when stamp is slightly ahead of the buffer

### __init__

(Livox cloud time vs EKF publish order — avoids dropped keyframes / map tearing).

### __init__

Livox per-point ``timestamp`` + IMU yaw-rate deskew (reduces smear when rotating).

### __init__

rodrigues: full 3D gyro deskew (handheld / tilted spin); yaw_only: ωz on x,y only.

### __init__

If map shears the wrong way vs rotation, try -1.0 (sensor/driver convention).

### __init__

Tighter keyframe spacing while ‖ω‖ is high (more overlap during fast motion).

### __init__

Optional safety gate: drop clouds that imply impossible pose jumps vs last keyframe.

### __init__

Auto-level map from dominant horizontal plane (floor or ceiling), applied once.

### __init__

Clock sync: add to IMU / cloud stamps so external IMU + Livox LiDAR share one timeline.

### __init__

If varying-gyro deskew fails, average IMU in buffer over each point's time span (rotation).

### __init__

If set (e.g. livox_frame): rotate Imu.angular_velocity from header.frame_id into this

### __init__

frame before deskew. Required when deskew uses Microstrain (imu_link) but points are in livox_frame.

### __init__

Livox /livox/imu often uses header ``sensor`` (MID360). That is NOT the same frame as

### __init__

Microstrain ``/imu/data`` ``sensor``; do not TF-rotate Livox gyro via the GX5 tree.

### __init__

When true and deskew IMU is ``/livox/imu``, treat ``sensor`` as already in the rotate target basis.

### __init__

Threshold on ‖ω‖ (rad/s), not only ωz — handheld rotation is rarely pure z.

### __init__

rclpy.time.Time (receive time; wall-clock fallback only)

### __init__

IMU time (offset) for deskew gate vs cloud

### __init__

Per-point RViz intensity: keyframe index (newer = higher); cleared on pose-graph rebuild.

### __init__

Persistent map-level correction (roll/pitch) applied to all future inserts/poses.

### _interp_gyro_from_buffer

Clamp query times to buffered span so fast spins still get interp (edges use endpoint ω).

### _angular_velocity_to_deskew_frame

Avoid using global TF frame ``sensor`` (Microstrain) to rotate Livox chip gyro.

### _deskew_imu_usable_for_cloud

One Livox frame ~100 ms; allow IMU stamp slightly before/after cloud stamp.

### _prefilter_points

Avoid O(N^2) distance matrix explosions on dense scans (process subset only).

### _try_loop_closure

Past keyframe index i with (new_idx - i) >= loop_min_index_gap (not recent chain).

## `src/keyframe_scan_map/keyframe_scan_map/pose_graph_node.py`

### module docstring

Lightweight planar pose graph:
  - Nodes: keyframe poses in `map` (from /keyframe_map/keyframes).
  - Edges: odometry-like constraints between consecutive keyframes.
  - Loop edges: /keyframe_map/loop_closure_pair [i, j] → identity relative constraint.

**Optimization (no g2o):** variables are ``(x, y, yaw)`` for poses 1..N-1; pose 0 is fixed.
Stacked SE(2) edge residuals (weighted) are minimized with **``scipy.optimize.least_squares``**
(method ``trf``). Depends on ``python3-scipy`` (declared in ``package.xml``).

Publishes /pose_graph/corrected_keyframes (nav_msgs/Path).

Optional **map → odom** correction (`publish_map_odom_tf`): after each successful solve, sets
`T_map_odom = T(corrected_last) * inv(T(raw_last))` (SE2) and republishes it periodically so the
TF tree absorbs drift at the map–odom boundary. **Disable** `launch_slam`'s static `map→odom`
when this is enabled (bringup does that automatically).

### _on_odom_stamp

Latch latest *reasonable* /ekf/odom stamp for map→odom TF (avoids TF_OLD_DATA from stale DDS).

### top

!/usr/bin/env python3

### __init__

SE(2) low-pass on map→odom: 1.0 = use each graph solve as-is; **0.1–0.35** eases TF jumps / map skew

### __init__

when new keyframes or loop edges re-optimize the chain.

### __init__

Stamp map→odom with latest /ekf/odom time so tf2 matches lidar-timed odom→base_link.

### __init__

Ignore /ekf/odom samples whose header stamp is this far behind node clock (stale DDS / replay).

### __init__

Ignore stamps this far in the future vs clock (bad clock sync).

### _publish_map_odom_tf

Match EKF odometry stamp so map→odom chains with odom→base_link at LiDAR/IMU times.

### fun

residual vector for scipy.least_squares

### fun

trust-region reflective; dense Jacobian from finite diff

## `src/keyframe_scan_map/keyframe_scan_map/pose_graph_se2.py`

### module docstring

Planar SE(2) helpers for lightweight pose-graph residuals (numpy 3x3).

### odom_measurement

Relative transform T_i^{-1} T_j from current poses (3x3).

### residual_between

Log-like error vector (3,) from pred and meas SE2 homogeneous (3x3).

### transform_points_se2

Apply SE(2) in xy; z column unchanged (planar correction of map points).

## `src/keyframe_scan_map/launch/keyframe_map.launch.py`

### top

!/usr/bin/env python3

## `src/lidar_odometry/launch/lidar_odometry.launch.py`

### top

!/usr/bin/env python3

## `src/lidar_odometry/launch/lidar_odometry_scan_to_map.launch.py`

### module docstring

Same as lidar_odometry.launch.py but registration_mode:=scan_to_map.

### top

!/usr/bin/env python3

## `src/lidar_odometry/src/lidar_odometry_node.cpp`

### file

*
 * Lidar odometry using PCL Normal Distributions Transform (NDT).
 *
 * registration_mode = "scan_to_scan":
 *   Consecutive scans; same as original behavior.
 *
 * registration_mode = "scan_to_map":
 *   Maintains a voxel map in odom (accumulated aligned scans).
 *   Default high-performance path (scan_to_map_register_sensor_frame=true): source = filtered scan in
 *   **sensor** frame, target = map in **odom**, ndt.align(..., T_odom_sensor_pred) — full 3D T_ndt kept
 *   internally; global_pose_full_ is kept planar (SE2 in odom) after each scan_to_map update;
 *   global_pose_ matches it for publish/2D. Optional
 *   yaw blend (ndt_fuse_prior_planar_yaw + ndt_prior_yaw_blend) and corridor degeneracy check.
 *   Map merge uses transformPointCloud(..., T_odom_sensor_new). Legacy path: T_ndt * T_pred. Optional
 *   coarse NDT, keyframe merge, opposite-motion / tiny-correction fallbacks.
 *   Optional ndt_gate_until_prior_translation_m: skip NDT until planar EKF prior |xy| exceeds threshold
 *   (reduces bad startup alignment when TF is not yet stable).
 *
 * Motion prior for scan_to_map (parameter use_tf_initial_guess, default true):
 *   TF lookup odom_frame -> base_frame at the cloud stamp (e.g. from ekf_node). If that stamp is
 *   not in the buffer yet (NDT often receives the cloud before ekf_node's same-topic callback
 *   publishes odom->base_link at that time), falls back to latest TF, then to last integrated pose.
 *
 * Subscribes: sensor_msgs/PointCloud2
 * Publishes:
 *   - nav_msgs/Odometry : integrated planar pose in odom_frame -> base_frame
 *   - geometry_msgs/TwistStamped : planar step in **odom** (Δx, Δy, Δθ); header.frame_id = odom_frame
 *   - geometry_msgs/PoseStamped (scan_to_map only): NDT planar correction vs prediction

### file

*
 * PCL 1.12 NDT builds the target with VoxelGridCovariance but does not expose
 * setMinPointPerVoxel / setCovEigValueInflationRatio. Without those, fine resolution +
 * sparse Livox-style cells often yield singular covariances and repeated
 * "[VoxelGridCovariance::applyFilter] Invalid eigen value! (0, 0, 0)" warnings.

### file

*
 * NDT returns full 6-DOF; for planar driving, keep translation and replace rotation with Rz(yaw)
 * using the same yaw convention as yawFrom2DBlock (valid when roll/pitch are small).

### file

* Planar SE2 pose (z=0, yaw only) extracted from a general rigid transform.

### file

* Build planar increment in odom (xy + yaw about z) from T_ndt^{-1} (scan-to-scan).

### file

* stdout + logger: integrated odom->base planar pose (not raw getFinalTransformation).

### file

* Per-scan NDT output only — must be composed into global_pose_; do not assign pose = this.

### file

* Publishable planar odom→base from accumulated full 3D pose.

### file

* VoxelGridCovariance settings for PCL 1.12 NDT (no public setCovarianceEpsilon on this version).

### file

* Planar odom->base prior: EKF TF at cloud time; else latest TF; else last integrated NDT pose.

### file

* If prior_odom_base is set, fill twist from planar (T_pred^{-1} * T_new) / dt (child_frame convention).

### file

* Sensor frame -> odom using planar base pose in odom.

### file

* If enabled, periodically discard accumulated map and rebuild target from recent aligned scans.

### file

* Accumulated odom→base (6-DOF); global_pose_ is planar projection for publish/2D logic.

### file

scan_to_map only: every N successful alignments, replace voxel map from a short ring of

### file

recent aligned scans (drops accumulated bad geometry; global_pose_ unchanged). 0 = disabled.

### file

scan_to_map: register sensor-frame source to odom map with align(..., T_odom_sensor_pred).

### file

base_link <- lidar frame: roll, pitch, yaw (rad), x, y, z (m). Default identity if cloud

### file

is already in base_link; set for livox_frame offset from base.

### file

Planar step T_new * T_pred^{-1} in odom (world xy); NOT body-frame despite TwistStamped norms.

### file

Map is built in odom using T_odom_base_pred; keep integrated pose in sync so the first

### file

/lidar/odom is not stuck at identity while EKF already moved (avoids NIS rejects / spin).

### file

namespace lidar_odometry

## `src/lio_bringup/config/fastlio_mid360_agile_timing_overlay.yaml`

### header

FAST-LIO agile timing isolation overlay:
- enforce single IMU source from Livox hardware sync domain
- start with zero lidar<->imu offset for clean baseline

### mapping

Increase IMU noise to reduce over-trust during aggressive motion.

### fov_degree

Frame-alignment fix: 180 deg yaw rotation about +Z.

## `src/lio_bringup/config/fastlio_mid360_overlay.yaml`

### header

FAST-LIO overrides (merged after mid360.yaml). Knobs = docs/tuning.md §10.

### preprocess

--- knobs (tuning.md §10) ---

### blind

--- rest ---

### mapping

--- knobs (tuning.md §10) ---

### extrinsic_est_en

--- rest ---

## `src/lio_bringup/launch/lio_backend.launch.py`

### module docstring

FAST-LIO backend for launch_slam when use_lio:=true.

Starts fastlio_mapping + lio_odom_relay_node (/Odometry -> /lidar/odom as odom->base_link).
When ``lio_relay_publish_tf`` is true, the relay also broadcasts TF odom->base_link (~scan rate).
That is sparse vs EKF TF and can worsen map smear; use with ``ekf_node`` ``publish_tf`` false only
if you accept that trade-off.

Config paths (under each package's **share** after install):
  **fast_lio** — ``fastlio_params_file`` (default ``config/mid360.yaml``).
  **lio_bringup** — ``lio_overlay_params_file`` (default ``config/fastlio_mid360_overlay.yaml``).
  **Optional** — ``lio_bag_overlay_params_file``: path **relative to ros_project_bringup** share,
  merged **last** (e.g. ``config/fastlio_bag_replay_overlay.yaml``). Use for PointCloud2 bags that
  omit Livox ``tag``/``line`` fields (``preprocess.lidar_type: 0``). Empty = skip (robot / live Livox).

## `src/lio_bringup/lio_bringup/lio_odom_relay_node.py`

### module docstring

Relay FAST-LIO /Odometry (camera_init -> body) onto /lidar/odom (odom -> base_link)
with the same pose/twist/covariance so localisation_ekf + frame checks stay unchanged.

Assumes odom is aligned with FAST-LIO world (camera_init) and base_link with body
for this workspace (see lio_bringup README).

Optional ``body_to_base_yaw_deg`` (default 0): planar rotation from FAST-LIO **body** to robot
``base_link`` (REP-103 +X forward). When RViz ``/ekf/path`` +X points **aft** while the robot
drives **forward**, try **180.0** — Livox mount yaw is separate (``livox_extrinsic_yaw_deg``).

Optional ``publish_tf`` (default false): broadcast ``odom`` -> ``base_link`` from each relayed
message so one node owns that TF (use with ``ekf_node`` ``publish_tf``: false in LIO mode).

Optional ``sync_tf_cloud_topic``: when set and ``publish_tf`` is true, also broadcast the same
transform with each point-cloud ``header.stamp`` so ``tf2`` / message_filters can interpolate at
LiDAR time (last odometry pose held forward until the next /Odometry).

### _rotate_twist_body_to_base

Twist is expressed in child frame; rotate linear + angular from body to base (planar Z).

### _apply_body_to_base_yaw

Return a copy with pose + twist adjusted by fixed yaw about Z (body -> base_link).

    Orientation uses **quaternion_multiply** (post-multiply by R_z(yaw)): T_odom_base = T_odom_body
    @ R_body_base. Do **not** add yaw to Euler-decomposed angles — that desynchronizes heading from
    LiDAR position when roll/pitch are non-zero, so /ekf/path can look flipped vs the fused arrow.

### top

!/usr/bin/env python3

## `src/localisation_ekf/config/ekf_node.yaml`

### header

HOW TO USE THIS FILE
--------------------
For the **Python** `ekf_node` (this repo), use `config/ekf_python.yaml` instead.
This file is mainly **robot_localization** `ekf_filter_node` templates + one active IMU-only profile.

This file contains one ACTIVE profile (at bottom) plus commented templates.

Profiles:
1) IMU-only:
- Keep "two_d_mode: true"
- Keep all LiDAR inputs disabled
- Best for bringup/testing only (drift is expected)

2) IMU + LiDAR pose (x,y,z,yaw):
- Set "two_d_mode: false"
- Enable odom0 and odom0_config for x,y,z,yaw
- Use this when LiDAR odometry is available

3) IMU + LiDAR z-only:
- Set "two_d_mode: false"
- Do NOT enable odom0 pose fusion unless needed
- Feed LiDAR z into your custom node path (ekf_node parameter: lidar_z_topic)

4) IMU + LiDAR pose + extra z-only:
- Set "two_d_mode: false"
- Enable odom0 for x,y,z,yaw
- Also feed an additional z-only odometry stream as odom1
- Use this when you want tighter altitude correction

NOTE:
This YAML config is for robot_localization-style ekf_filter_node.
Your custom Python ekf_node uses different parameters (imu_topic, lidar_*_topic, etc).
Keep that distinction in mind when switching stacks.

### header

----------------------------------------------------------
TEMPLATE A: IMU + LiDAR pose (x,y,z,yaw)  [COMMENTED OUT]
----------------------------------------------------------
ekf_filter_node:
ros__parameters:
use_sim_time: true
frequency: 50.0
sensor_timeout: 0.1
two_d_mode: false

map_frame: map
odom_frame: odom
base_link_frame: base_link
world_frame: odom
publish_tf: true
print_diagnostics: true

imu0: /imu/data
imu0_queue_size: 20
imu0_nodelay: true
imu0_config: [
false, false, false,
true,  true,  true,
false, false, false,
true,  true,  true,
true,  true,  true
]
imu0_differential: false
imu0_relative: false
imu0_remove_gravitational_acceleration: true

odom0: /lidar/odom
odom0_queue_size: 10
odom0_nodelay: true
odom0_config: [
true,  true,  true,
false, false, true,
false, false, false,
false, false, false,
false, false, false
]
odom0_differential: false
odom0_relative: false
odom0_pose_rejection_threshold: 5.0
odom0_twist_rejection_threshold: 1.0

----------------------------------------------------------
TEMPLATE B: IMU + LiDAR z-only  [COMMENTED OUT]
----------------------------------------------------------
For robot_localization ekf_filter_node there is no direct "z-only Float64" input.
If you only have LiDAR altitude, you can provide it through an odometry source where
only z is enabled in odom0_config as shown below.

ekf_filter_node:
ros__parameters:
use_sim_time: true
frequency: 50.0
sensor_timeout: 0.1
two_d_mode: false

map_frame: map
odom_frame: odom
base_link_frame: base_link
world_frame: odom
publish_tf: true
print_diagnostics: true

imu0: /imu/data
imu0_queue_size: 20
imu0_nodelay: true
imu0_config: [
false, false, false,
true,  true,  true,
false, false, false,
true,  true,  true,
true,  true,  true
]
imu0_differential: false
imu0_relative: false
imu0_remove_gravitational_acceleration: true

odom0: /lidar/z_odom
odom0_queue_size: 10
odom0_nodelay: true
odom0_config: [
false, false, true,
false, false, false,
false, false, false,
false, false, false,
false, false, false
]
odom0_differential: false
odom0_relative: false
odom0_pose_rejection_threshold: 5.0
odom0_twist_rejection_threshold: 1.0

### header

----------------------------------------------------------
TEMPLATE C: IMU + LiDAR pose + extra z-only  [COMMENTED OUT]
----------------------------------------------------------
This profile fuses:
- odom0: LiDAR pose for x,y,z,yaw
- odom1: additional z-only stream for stronger altitude correction

ekf_filter_node:
ros__parameters:
use_sim_time: true
frequency: 50.0
sensor_timeout: 0.1
two_d_mode: false

map_frame: map
odom_frame: odom
base_link_frame: base_link
world_frame: odom
publish_tf: true
print_diagnostics: true

imu0: /imu/data
imu0_queue_size: 20
imu0_nodelay: true
imu0_config: [
false, false, false,
true,  true,  true,
false, false, false,
true,  true,  true,
true,  true,  true
]
imu0_differential: false
imu0_relative: false
imu0_remove_gravitational_acceleration: true

odom0: /lidar/odom
odom0_queue_size: 10
odom0_nodelay: true
odom0_config: [
true,  true,  true,
false, false, true,
false, false, false,
false, false, false,
false, false, false
]
odom0_differential: false
odom0_relative: false
odom0_pose_rejection_threshold: 5.0
odom0_twist_rejection_threshold: 1.0

odom1: /lidar/z_odom
odom1_queue_size: 10
odom1_nodelay: true
odom1_config: [
false, false, true,
false, false, false,
false, false, false,
false, false, false,
false, false, false
]
odom1_differential: false
odom1_relative: false
odom1_pose_rejection_threshold: 5.0

### header

----------------------------------------------------------
ACTIVE PROFILE: IMU-only  [CURRENT DEFAULT]
----------------------------------------------------------

### ekf_filter_node

IMU-only fusion (no LiDAR). Drift in x/y/yaw is expected.

### imu0_nodelay

[x,y,z, roll,pitch,yaw, vx,vy,vz, vroll,vpitch,vyaw, ax,ay,az]

## `src/localisation_ekf/config/ekf_python.yaml`

### header

=============================================================================
ekf_node parameters (localisation_ekf) — IMU prediction + optional LiDAR correction
=============================================================================

Primary fusion stack in this repo: **your** NDT (`lidar_odometry`) + this EKF, OR
FAST-LIO via `use_lio:=true` in `ros_project_bringup/launch_slam.launch.py`.

LiDAR measurement source — default (NDT path):
Package: lidar_odometry
Node:    lidar_odometry_node
Publishes: /lidar/odom  (nav_msgs/Odometry, odom -> base_link), /lidar/relative_motion

LiDAR measurement source — use_lio:=true (see launch file docstring for full detail):
NDT node is NOT started. FAST-LIO publishes /Odometry; lio_odom_relay republishes
/lidar/odom (odom -> base_link). This EKF file does not change: lidar_odom_topic
stays /lidar/odom. No /lidar/relative_motion from NDT. Tune LIO in lio_bringup overlay
+ FAST_LIO_ROS2/config/mid360.yaml; consider lidar_fuse_z_from_odom below for 3D LIO.

Launch overrides LiDAR R / gate: ekf_lidar_pose_var, ekf_lidar_yaw_var, ekf_lidar_gate_nis

Typical bringup (starts driver + NDT + EKF + RViz):
ros2 launch ros_project_bringup launch_slam.launch.py

IMU-only (no /lidar/odom for EKF — need BOTH off):
ros2 launch ros_project_bringup launch_slam.launch.py use_lidar_fusion:=false use_lio:=false

LIO instead of NDT (EKF still fuses /lidar/odom):
ros2 launch ros_project_bringup launch_slam.launch.py use_lio:=true use_lidar_fusion:=false

Livox driver already running elsewhere (same machine):
ros2 launch ros_project_bringup launch_slam.launch.py start_livox_driver:=false

Optional altitude-only correction (std_msgs/Float64, data = z); leave "" if unused:
lidar_z_topic: "/lidar/z"

State (9): px, py, z, yaw, vx, vy, bax, bay, bgz
=============================================================================

### ros__parameters

Overridden by launch_slam (default true). Standalone: false for live robot.

### imu_topic

Fallback dt if stamps glitch; Livox IMU is often 100–200 Hz — real dt comes from stamps when use_stamp_dt is true

### use_stamp_dt

true: use body (ax,ay) in prediction; false: yaw from gyro only, LiDAR corrects motion (see gx5 yaml)

### predict_use_linear_accel

Seconds added to IMU / LiDAR message header stamps (fixed skew vs Livox clock). launch_slam overrides.

### publish_tf

When IMU header.frame_id != base_link_frame, rotate accel/gyro into base (needs TF)

### imu_tf_lookup_timeout_sec

Latched std_msgs/String: data is "livox" or "microstrain" (set by launch_slam from slam_bringup.yaml)

### imu_source_id

Between NDT scans, IMU coasts: modest px/py/yaw Q; vx/vy not too tight so wrong velocity decays when LiDAR returns.

### process_noise_diag

px

### process_noise_diag

py

### process_noise_diag

z (mostly LiDAR-driven)

### process_noise_diag

yaw

### process_noise_diag

vx

### process_noise_diag

vy

### process_noise_diag

bax random walk

### process_noise_diag

bay random walk

### process_noise_diag

bgz random walk

### path_topic

--- LiDAR correction topics (empty string = disabled) ---
Default matches NDT output; launch_slam.launch.py overrides when use_lidar_fusion:=false

### lidar_z_topic

Accuracy-first (default): low variances => EKF tracks /lidar/odom tightly (can look jumpy).

### lidar_require_frames

Fuse z from /lidar/odom (x,y,z,yaw EKF update). false = planar NDT (z not trusted).
launch_slam sets auto: true when use_lio, false for NDT-only (override with ekf_lidar_fuse_z_from_odom).

### lidar_fuse_z_from_odom

With tight R, raise gate so good LiDAR updates are not dropped as “outliers” too often

## `src/localisation_ekf/config/ekf_python_gx5_microstrain.yaml`

### header

ekf_node — GX5 + LiDAR (see docs/tuning.md §3). Knobs = process_noise_diag only.

### ros__parameters

Overridden by launch_slam (default true). Bag replay needs true + `ros2 bag play --clock`.

### use_sim_time

=========================================================================
KNOBS — only parameters listed in docs/tuning.md §3
=========================================================================

### process_noise_diag

px

### process_noise_diag

py

### process_noise_diag

z

### process_noise_diag

vx

### process_noise_diag

vy

### process_noise_diag

bax

### process_noise_diag

bay

### process_noise_diag

Gyro bias random walk (rad/s)^2 * dt; was too small → bgz barely learned from yaw meas.
bgz

### process_noise_diag

=========================================================================
REST (frames, topics, LiDAR R, P0 — launch + slam_bringup override much of this)
=========================================================================

### imu_topic

false: yaw from gyro; horizontal motion from LiDAR (accel in planar model skews with tilt/shock).

### imu_stamp_offset_sec

Subtracted from ωz after TF to base_link (see ekf_node). Set ≈ stationary mean to remove bias drift.

### initial_cov_diag

bgz — allow faster initial bias learn (tighten after calibration)

### lidar_odom_topic

Optional TwistStamped planar delta (same as lidar_odometry delta_topic); launch sets from slam_bringup.

### lidar_use_roll_pitch

Launch overrides from slam_bringup; defaults match ekf_node.

## `src/localisation_ekf/localisation_ekf/ekf_filter.py`

### module docstring

Planar IMU + LiDAR EKF for ground robots.

State (9): [px, py, z, yaw, vx, vy, bax, bay, bgz]
  - Pose in world: (x, y, z); heading yaw (theta).
  - Velocities vx, vy in world (for IMU propagation).
  - Biases: horizontal accel biases bax, bay (body x/y), yaw-rate bias bgz.

z is not integrated from IMU (LiDAR / z updates only). Roll and pitch are not
estimated; output orientation uses roll=pitch=0, yaw from state.

Assumes IMU linear acceleration is specific force including gravity; horizontal
motion is obtained by rotating body (x,y) accel into the world frame at the
current yaw. Small roll/pitch is assumed unless LiDAR corrects pose frequently.

### wrap_angle

Planar EKF with accel biases (bax, bay) and yaw gyro bias (bgz).

### nis_lidar_xy

NIS for a hypothetical x,y position update only (no yaw).

### nis_lidar_xy_yaw

NIS for a hypothetical xy,yaw update at the current state (no state change).

### update_lidar_xy

Measurement update on x, y only (ignore LiDAR yaw — yaw stays IMU-predicted).

### update_lidar_xy_yaw

Measurement update on x, y, yaw only (planar LiDAR odom with no reliable z).

### update_lidar_velocity_xy

Optional weak update on world-frame planar velocities (e.g. from NDT scan delta / dt).

### get_state

Return position [px,py,pz] and euler [roll, pitch, yaw] (roll/pitch zero).

### get_biases

Accel biases (body x,y) and gyro bias (z), SI units.

### top

!/usr/bin/env python3

### EKFPlanarIMU

Indices: px, py, z, yaw, vx, vy, bax, bay, bgz

### __init__

Default Q scale per state (tuned for ~100 Hz IMU, indoor LiDAR correction)

### __init__

px, py, z, yaw

### __init__

vx, vy

### __init__

bax, bay (m/s²), bgz (rad/s) random walk

### __init__

If False: integrate yaw from gyro only; do not use body (ax,ay) for velocity/position.

### __init__

Reduces map skew when vertical shocks / tilt couple into horizontal accel (planar model).

### predict

Constant-velocity coast in world frame; yaw from gyro; LiDAR corrects (x,y,yaw,vx,vy).

### update_lidar_pose

roll, pitch from LiDAR orientation are ignored here (xy, z, yaw only).

### _update

gate_nis=None skips NIS (used for LiDAR soft-pull retry). To use the filter default,

### _update

pass self.nis_gate_default explicitly from the caller.

### get_biases

Backwards-compatible alias for older imports

## `src/localisation_ekf/localisation_ekf/ekf_node.py`

### lidar_delta_callback

Fuse NDT planar step (dx, dy) / dt as a weak velocity measurement in odom frame.

### _lidar_bootstrap_from_xy

Snap position only; keep current yaw (xy-only LiDAR fusion mode).

### docstring

Scale pose/yaw measurement variance when planar speed is low (LIO jitter at rest).

        Uses ``max(|v_lidar_twist|, |v_ekf|)`` for gating: FAST-LIO often fills twist.linear with
        ~0 even while moving; trusting only twist would inflate variance during motion and break
        xy fusion while yaw stays IMU-only (``lidar_fuse_xy_only``), which looks like a huge
        heading/path mismatch.

### top

!/usr/bin/env python3

### top

Microstrain (and many IMU drivers) publish sensor_msgs/Imu with best-effort QoS.

### top

Default rclpy subscription is reliable — no IMU callbacks → EKF looks LiDAR-only.

### __init__

--------------------

### __init__

Parameters

### __init__

--------------------

### __init__

False: yaw from gyro only; do not integrate body (ax,ay) into velocity (see EKFPlanarIMU).

### __init__

TF / odometry parent frame (ROS convention: odom -> base_link)

### __init__

Legacy: was used as odometry parent; ignored if set (use odom_frame + map->odom TF)

### __init__

Optional: stamp /ekf/odom + TF with ROS time (usually leave false so TF matches

### __init__

sensor-timed PointCloud2; pose_graph can stamp map→odom from /ekf/odom instead).

### __init__

Rotate IMU linear_accel + angular_velocity into base_link when header.frame_id differs

### __init__

Latched std_msgs/String: which IMU hardware the stack was configured for (livox | microstrain)

### __init__

Planar EKF noise (state order: px,py,z,yaw,vx,vy,bax,bay,bgz)

### __init__

Uncertainty on unobserved roll/pitch (rad^2) in published odometry

### __init__

Optional LiDAR fusion inputs

### __init__

Often tighten yaw vs x,y,z so LiDAR corrects heading drift strongly

### __init__

If false: fuse only x,y,yaw from LiDAR odom (z from /lidar/z or IMU hold)

### __init__

If true (planar path only): fuse only x,y from LiDAR; yaw from IMU integration only.

### __init__

Use when LiDAR/NDT heading is untrustworthy but position is good — fix IMU mount if yaw still wrong.

### __init__

If set (e.g. /livox/lidar), stamp /ekf/odom + TF from this cloud header when not

### __init__

fusing LiDAR odom — aligns TF with Livox time when IMU uses a different clock (Microstrain).

### __init__

Limit /ekf/odom + TF rate during IMU-only coast (0 = unlimited). LiDAR/cloud publishes ignore this.

### __init__

Add to sensor header stamps so IMU + LiDAR share one timeline (fixed clock skew vs Livox).

### __init__

Subtracted from ωz (rad/s) after TF to base_link, before EKF predict. At rest, set ≈ mean(ωz)

### __init__

on /imu/data (e.g. -0.012 if the GX5 reads -0.012 stationary) to kill pure bias integration drift.

### __init__

First tune_sec of IMU time: average ωz (after TF); then subtract that mean + manual bias. Use when

### __init__

bag starts stationary (robot still); set manual to 0 to avoid double correction.

### __init__

Applied after bias subtraction (try -1.0 if yaw runs opposite to truth / TF sign error).

### __init__

Added to published yaw only (odom/pose/path/TF); does not change LiDAR fusion or internal state.

### __init__

Optional: fuse planar scan delta (TwistStamped linear.x/y = dx,dy in odom; dz unused)

### __init__

into vx, vy between full LiDAR pose updates (gyro-only translation mode).

### __init__

Verbose LiDAR fusion line (throttled): NIS, innovations, applied flags.

### __init__

After gated xy,yaw reject (NDT path), run one soft ungated update (large R). Default

### __init__

true: avoids gyro-only coast when NIS fails on yaw but |Δxy| is small.

### __init__

Planar speed from /lidar/odom twist.linear (or EKF vx,vy when absent): below this threshold,

### __init__

multiply lidar_pose_var (+ yaw var when fused) by lidar_pose_var_below_slow_speed_scale so

### __init__

scan-matching jitter while stopped/slow does not yank the EKF (LIO/NDT noise at v≈0).

### __init__

Throttle state only; re-read lidar_fusion_debug_log each callback so

### __init__

``ros2 param set`` works without restart.

### __init__

frozen mean, rad/s

### __init__

--------------------

### __init__

EKF

### __init__

--------------------

### __init__

``odom``→``base_link`` TF uses **node clock time** (not odometry header time) so tf2 never sees

### __init__

out-of-order transforms during bag replay (LiDAR/IMU stamps vs /clock). /ekf/odom headers unchanged.

### __init__

If LiDAR odom is NIS-rejected while EKF is near origin, snap once (gyro-only + tight R).

### __init__

Further snaps when LiDAR xy disagrees strongly with EKF (rejects / yaw–xy coupling).

### __init__

--------------------

### __init__

Publishers

### __init__

--------------------

### __init__

--------------------

### __init__

TF Broadcaster

### __init__

--------------------

### __init__

--------------------

### __init__

Subscribers

### __init__

--------------------

### __init__

Cloud stamp first: publish odom→base_link at each cloud time before other callbacks in the

### __init__

same executor tick where possible (helps tf2 vs NDT scan time).

### _on_lidar_cloud_stamp

NDT can skip or lag vs Livox; RViz TF MessageFilter wants odom→base_link near each cloud time.

### _on_lidar_cloud_stamp

--------------------

### _on_lidar_cloud_stamp

Time handling

### _on_lidar_cloud_stamp

--------------------

### compute_dt

--------------------

### compute_dt

IMU callback (CORE)

### compute_dt

--------------------

### imu_callback

Static base_link→imu_link may not exist at the IMU message time yet

### imu_callback

(startup ordering / clock); latest transform is correct for fixed extrinsic.

### imu_callback

Skip path on IMU: full Path at ~IMU Hz overloads RViz; path updates on LiDAR fusion only.

### _fuse_lidar_pose

Planar correction: use yaw only; roll/pitch from LiDAR are ignored.

### _fuse_lidar_pose

Second line of defence: planar LiDAR after gated reject. Large R, no NIS.

### _fuse_lidar_pose

Default always runs when lidar_soft_fuse_after_gate_reject (NDT + gyro-only translation).

### _fuse_lidar_pose

Do not overwrite last_imu_stamp: keep IMU dt chain; publish this

### _fuse_lidar_pose

correction with the LiDAR message time for TF/odom sync.

### _fuse_lidar_pose

--------------------

### _fuse_lidar_pose

Publish results

### _fuse_lidar_pose

--------------------

### publish_outputs

Planar EKF holds vx,vy in **world** frame; odometry twist must be in ``child_frame_id``

### publish_outputs

(base_link). Rotate into the **published** heading ``yaw_pub`` (includes publish-only offset).

### publish_outputs

Must match odom.header.stamp (sensor / filter time). NDT looks up odom→base_link at

### publish_outputs

PointCloud2.header.stamp; stamping TF with wall/sim "now" leaves the cloud in the TF

### publish_outputs

future → lookup fails → NDT falls back to its stale pose while EKF still moves.

## `src/localisation_ekf/test/test_copyright.py`

### top

Copyright 2015 Open Source Robotics Foundation, Inc.

### top

Licensed under the Apache License, Version 2.0 (the "License");

### top

you may not use this file except in compliance with the License.

### top

You may obtain a copy of the License at

### top

http://www.apache.org/licenses/LICENSE-2.0

### top

Unless required by applicable law or agreed to in writing, software

### top

distributed under the License is distributed on an "AS IS" BASIS,

### top

WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.

### top

See the License for the specific language governing permissions and

### top

limitations under the License.

### top

Remove the `skip` decorator once the source file(s) have a copyright header

## `src/localisation_ekf/test/test_flake8.py`

### top

Copyright 2017 Open Source Robotics Foundation, Inc.

### top

Licensed under the Apache License, Version 2.0 (the "License");

### top

you may not use this file except in compliance with the License.

### top

You may obtain a copy of the License at

### top

http://www.apache.org/licenses/LICENSE-2.0

### top

Unless required by applicable law or agreed to in writing, software

### top

distributed under the License is distributed on an "AS IS" BASIS,

### top

WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.

### top

See the License for the specific language governing permissions and

### top

limitations under the License.

## `src/localisation_ekf/test/test_pep257.py`

### top

Copyright 2015 Open Source Robotics Foundation, Inc.

### top

Licensed under the Apache License, Version 2.0 (the "License");

### top

you may not use this file except in compliance with the License.

### top

You may obtain a copy of the License at

### top

http://www.apache.org/licenses/LICENSE-2.0

### top

Unless required by applicable law or agreed to in writing, software

### top

distributed under the License is distributed on an "AS IS" BASIS,

### top

WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.

### top

See the License for the specific language governing permissions and

### top

limitations under the License.

## `src/ros_project_bringup/config/fastlio_bag_replay_overlay.yaml`

### header

Laptop / bag replay — merged automatically when ``launch_slam`` runs FAST-LIO with
``launch_sensors:=false`` (see ``slam_bringup.yaml`` ``lio_bag_overlay_params_file``).

Merged **last** (after fast_lio mid360.yaml + lio_bringup fastlio_mid360_overlay.yaml):
- PointCloud2 bags often omit Livox ``tag``/``line`` → ``preprocess.lidar_type: 0`` (avoids
``Failed to find match for field 'line'/'tag'`` when overlay used ``lidar_type: 4`` alone).
- ``publish_tf: true`` so ``body`` exists on ``/tf`` for tools; EKF normally owns
``odom``→``base_link``. If you set ``lio_relay_publish_tf: true`` and EKF TF off, expect
sparse relay TF (~scan rate) and possible smear — prefer defaults (relay TF false).

Robot (``launch_sensors:=true``): this file is **not** loaded; ``fastlio_mid360_overlay.yaml`` stays.

## `src/ros_project_bringup/config/livox_extrinsic_yaw_0_overlay.yaml`

### header

Merge overlay for `bringup_config:=...` — explicit Livox yaw 0° (same as package default in slam_bringup.yaml).
Base: share/ros_project_bringup/config/slam_bringup.yaml (deep-merge at slam_bringup key level).

## `src/ros_project_bringup/config/livox_extrinsic_yaw_180_overlay.yaml`

### header

Use when the Livox cloud frame is physically **reversed** vs `base_link` +X (sensor “looks” backward).
Package default `slam_bringup.yaml` is 0°; merge this via bringup_config on those robots.

## `src/ros_project_bringup/config/livox_ndt_bag_axes_overlay.yaml`

### header

NDT bag replay overlay — forces PCL NDT (overrides package default FAST-LIO when this file is merged).
**EKF uses /imu/data** in `header.frame_id` (often `sensor`): rotation
`imu_link`→`sensor` is `microstrain_sensor_alias_*`. That affects ω in **base_link** as much as `imu_mount_*`.
Start aliases at **0**; add ±90° only if a specific bag’s GX5 frame convention requires it.

EKF fuses **raw** NDT odom (not EMA-smoothed /lidar/odom) so pose does not lag/fight the filter; keep
lidar_odom_smooth_enable true if you still want /lidar/odom for RViz/tools.

### use_lidar_fusion

Keyframe map uses EKF IMU for deskew (see slam_bringup); explicit here for bag replay clarity.

### ekf_lidar_odom_topic

Trust NDT pose more than "loose" R: large lidar_pose_var => tiny Kalman gain once P has
shrunk ⇒ EKF barely moves while NDT jumps (pipeline_translation_debug ratio_ekf/lidar << 1).

### ekf_lidar_pose_var

Smaller → EKF follows NDT yaw more (see slam_bringup comment). Raise if LiDAR heading is noisy.

### ekf_lidar_fusion_debug_throttle_sec

Ignore NDT yaw in EKF (noisy scan matching); fix stationary drift with ekf_imu_gyro_z_bias_rad_s.

### ekf_lidar_fuse_xy_only

Keep 0 when auto bias is on (below). Otherwise set mean stationary ωz after TF.

### ekf_imu_auto_gyro_z_bias_tune_sec

Still permissive for scan_to_map jitter; raise if you see frequent LiDAR fuse rejects in logs.

### ekf_lidar_gate_nis

false: gyro-only prediction between LiDAR steps (GX5 preset); true can add bogus translation if IMU axes don’t match planar model.

### ekf_predict_use_linear_accel

Finer structure + smaller LM step + more iterations (reduce rotation-only NDT basins on Livox).

### lidar_ndt_resolution

Tighter NDT cells need more points per voxel / stronger inflation if rosout still shows invalid eigen.

### lidar_ndt_max_iterations

Coarse → fine NDT (translation basins); ~1.0–1.5 typical for Livox at ~0.5 fine resolution.

### lidar_log_ndt_relative

true: blend NDT yaw toward motion prior (reduces scan-matching yaw noise / diagonal drift); weight → prior.

### lidar_ndt_fallback_if_planar_correction_below_m

Stricter spin-without-translation guard (was 0.5 rad / 0.1 m in package default).

## `src/ros_project_bringup/config/microstrain_params_overlay.yaml`

### header

Merged on top of the vendor microstrain `params.yml` when `launch_slam` starts the
Microstrain node (`microstrain_imu_origin: local` + `launch_sensors:=true`).

Source of truth for geometry: `slam_bringup.yaml` `imu_mount_*` + `docs/imu_mount_note.md`.
Port/baud/rates are still overridden again from `slam_bringup.yaml` after this merge.
EKF `microstrain_imu_topic`: `/imu/data` (Foxy robot driver) or `/imu/data_raw` (Humble raw-only). Overlay
matches Humble local driver rates; robot Foxy ignores imu_data_raw_rate — set topic in slam_bringup.yaml.

`mount_to_frame_id_transform`: [x, y, z, qx, qy, qz, qw] = base_link → imu_link
Translation (0, 0, 0.065) m; yaw −90° about +Z (GX5 +X toward connector vs robot +X forward).

### mount_frame_id

Match launch_slam: static TF from `imu_mount_*` defines mount; avoid duplicate TF from driver.

### publish_mount_to_frame_id_transform

Kept accurate for standalone `microstrain_inertial_driver` runs or firmware-related use.

## `src/ros_project_bringup/config/ndt_fastlio_equivalent_overlay.yaml`

### header

NDT-only bringup: PCL NDT tuned to match ``lio_bringup/config/fastlio_mid360_overlay.yaml``.
Does not change the default ``use_lio: true`` pipeline in ``slam_bringup.yaml``.
bringup_config:=$(ros2 pkg prefix ros_project_bringup)/share/ros_project_bringup/config/ndt_fastlio_equivalent_overlay.yaml
Mapping (FAST-LIO overlay → NDT):

## `src/ros_project_bringup/config/overlay_agile_mode.yaml`

### slam_bringup

Agile isolation mode: pure FAST-LIO front-end.

### start_pose_graph

Keep keyframe deskew path out of this test.

### keyframe_deskew_imu_topic

Keep deskew offset neutral; leave EKF offsets untouched for this phase.

### keyframe_deskew_imu_stamp_offset_sec

Use agile FAST-LIO overlay (Livox IMU + fixed identity extrinsic).

### lio_overlay_params_file

No sign/identity hacks.

## `src/ros_project_bringup/config/overlay_diag_frontend_livox_imu.yaml`

### slam_bringup

Isolate FAST-LIO front-end during bag replay.

### keyframe_deskew_enable

Remove timing offsets for clean baseline.

### ekf_lidar_stamp_offset_sec

Use dedicated FAST-LIO agile timing overlay from lio_bringup package.
This switches LIO IMU to /livox/imu and disables online extrinsic estimation.

## `src/ros_project_bringup/config/overlay_diag_no_hacks.yaml`

### slam_bringup

Remove deskew sign/identity hacks for geometry sanity checks.

## `src/ros_project_bringup/config/overlay_diag_test12_zero_offsets_no_deskew.yaml`

### slam_bringup

Test 1: remove timing offsets

### ekf_lidar_stamp_offset_sec

Test 2: disable external deskew (FAST-LIO already compensates)

## `src/ros_project_bringup/config/overlay_diag_testC_deskew_off.yaml`

### slam_bringup

Test C: isolate deskew path.

## `src/ros_project_bringup/config/overlay_diag_testF_lio_only.yaml`

### slam_bringup

Test F: isolate FAST-LIO odometry from mapping back-end.

## `src/ros_project_bringup/config/overlay_ema_C1_alpha_pose_0p08.yaml`

### header

EMA C1 — bag replay: more XY/Z smoothing with yaw still pass-through (slam_bringup xyz mode).
Baseline lidar_odom_smooth_alpha_pose 0.10 → 0.08 (requires lidar_odom_smooth_mode: xyz in base YAML).

Terminal 1:
export OVR=$(ros2 pkg prefix ros_project_bringup)/share/ros_project_bringup/config/overlay_ema_C1_alpha_pose_0p08.yaml
ros2 launch ros_project_bringup launch_slam.launch.py launch_sensors:=false bringup_config:=$OVR
Terminal 5: --out-dir /tmp/bench_50s_lockwin_ema_C1_alpha

## `src/ros_project_bringup/config/overlay_ema_F1_heavy.yaml`

### header

F1 — heavy smoothing (target: lower path-length ratio / less sawtooth; more lag vs FAST-LIO)

## `src/ros_project_bringup/config/overlay_ema_F1p5.yaml`

### header

F1.5 — between F1 heavy and F3 light (yaw a bit snappier than F1; try vs same bag + tune_rollup)

## `src/ros_project_bringup/config/overlay_ema_F2_mid.yaml`

### header

F2 — same as current defaults in slam_bringup.yaml (reference run)

## `src/ros_project_bringup/config/overlay_ema_F3_light.yaml`

### header

F3 — light smoothing (snappier; expect more micro-jitter if α is too high)

## `src/ros_project_bringup/config/overlay_fastlio_replay_tf.yaml`

### header

Merge with slam_bringup via:
bringup_config:=$(ros2 pkg prefix ros_project_bringup)/share/ros_project_bringup/config/overlay_fastlio_replay_tf.yaml
Use with: launch_slam.launch.py use_sim_time:=true launch_sensors:=false
Goal: map→odom static, base_link↔livox/imu static bridges, EKF publishing odom→base_link,
without NDT/keyframe/Rviz competing with FAST-LIO on the same LiDAR topics.

## `src/ros_project_bringup/config/overlay_ndt_E3_fitness_tighter.yaml`

### header

NDT experiment E3 — bag replay / laptop: one knob vs current slam_bringup.yaml.
Baseline lidar_max_fitness_score 7.0 → 5.5 (reject worse registrations; may drop scans if too tight).

Terminal 1:
export OVR=$(ros2 pkg prefix ros_project_bringup)/share/ros_project_bringup/config/overlay_ndt_E3_fitness_tighter.yaml
ros2 launch ros_project_bringup launch_slam.launch.py launch_sensors:=false bringup_config:=$OVR

Terminal 5: use --out-dir /tmp/bench_50s_lockwin_ndt_E3_fitness (same REC_* as prior benches).

## `src/ros_project_bringup/config/overlay_ndt_E4_step_smaller.yaml`

### header

NDT experiment E4 — bag replay / laptop: one knob vs current slam_bringup.yaml.
Baseline lidar_ndt_step_size 0.035 → 0.028 (smaller LM step; often stabilizes pose jumps).
Keep lidar_max_fitness_score at default 7.0 (E3 tighter fitness hurt on locked-window bench).

Terminal 1:
export OVR=$(ros2 pkg prefix ros_project_bringup)/share/ros_project_bringup/config/overlay_ndt_E4_step_smaller.yaml
ros2 launch ros_project_bringup launch_slam.launch.py launch_sensors:=false bringup_config:=$OVR

Terminal 5: --out-dir /tmp/bench_50s_lockwin_ndt_E4_step (same REC_* as prior benches).

## `src/ros_project_bringup/config/overlay_ndt_E5_ndt_resolution.yaml`

### header

NDT E5 — bag replay: coarser NDT map grid (single knob vs slam_bringup.yaml).
Baseline lidar_ndt_resolution 0.95 → 1.06 (~12% coarser; often damps XY micro-jitter).

Terminal 1:
export OVR=$(ros2 pkg prefix ros_project_bringup)/share/ros_project_bringup/config/overlay_ndt_E5_ndt_resolution.yaml
ros2 launch ros_project_bringup launch_slam.launch.py launch_sensors:=false bringup_config:=$OVR
Terminal 5: --out-dir /tmp/bench_50s_lockwin_ndt_E5_resolution

## `src/ros_project_bringup/config/overlay_ndt_E6_transformation_epsilon.yaml`

### header

NDT E6 — bag replay: tighter LM stop (single knob vs slam_bringup.yaml).
Baseline lidar_ndt_transformation_epsilon 0.01 → 0.006 (stricter convergence).
If NDT logs show thrashing or slower scans, try 0.014 instead (separate overlay).

Terminal 1:
export OVR=$(ros2 pkg prefix ros_project_bringup)/share/ros_project_bringup/config/overlay_ndt_E6_transformation_epsilon.yaml
ros2 launch ros_project_bringup launch_slam.launch.py launch_sensors:=false bringup_config:=$OVR
Terminal 5: --out-dir /tmp/bench_50s_lockwin_ndt_E6_epsilon

## `src/ros_project_bringup/config/overlay_ndt_tune_A_baseline.yaml`

### slam_bringup

A) Baseline (current)

## `src/ros_project_bringup/config/overlay_ndt_tune_B_smoother.yaml`

### slam_bringup

B) Smoother cloud + gentler optimizer step
Goal: reduce local zig-zag and path-length inflation.

## `src/ros_project_bringup/config/overlay_ndt_tune_C_strong_smooth.yaml`

### slam_bringup

C) Strong smoothing + capped iterations
Goal: suppress jitter further while avoiding overfitting each scan.

## `src/ros_project_bringup/config/overlay_ndt_tune_D_refine.yaml`

### slam_bringup

D) Refine after C: stronger smoothing + smaller optimizer step
Start from C winner and target lower local zig-zag / path-length inflation.

## `src/ros_project_bringup/config/overlay_ndt_tune_E_between_C_D.yaml`

### header

Same NDT + EMA knobs as the current default in slam_bringup.yaml (explicit pin for A/B diffs).

## `src/ros_project_bringup/config/slam_bringup.yaml`

### header

=============================================================================
SLAM stack bringup — loaded by: ros2 launch ros_project_bringup launch_slam.launch.py
=============================================================================

Path: bringup_config → ROS_PROJECT_SLAM_CONFIG → this file.
Knob list matches docs/tuning.md (§1–9). Suggested values + what each does: docs/tuning.md
All other keys: “REST OF BRINGUP” below.

Default profile: **live robot + SLAM on robot**, sensors and RViz elsewhere (`launch_sensors:=false`,
`start_rviz: false`, `use_sim_time` launch default false). Livox + Microstrain run from robot bringup;
this launch only starts SLAM nodes. **FAST-LIO** (`use_lio: true`, `use_lidar_fusion: false`).
For PCL NDT: `use_lio: false` and `use_lidar_fusion: true` (or `config/livox_ndt_bag_axes_overlay.yaml`).
Set `microstrain_imu_origin: local` only when this host runs the GX5 USB driver (`launch_sensors:=true`).
Livox `/livox/imu` in this workspace’s driver uses the same `frame_id` as the cloud (`livox_cloud_frame_id`).
`keyframe_deskew_imu_rotate_gyro_to_frame` must match that cloud frame so ω is rotated before deskew.
When the local Livox driver is off (`launch_sensors:=false`), launch publishes identity TF
`livox_frame`→`livox_imu_child_frame` if `publish_livox_imu_sensor_frame_tf` is true.
**Do not use child frame name `sensor` here** — Microstrain often publishes `/imu/data` with
`header.frame_id: sensor`; a Livox bridge named `sensor` hijacks that TF and breaks EKF / gyro checks.
For DDS laptop bringup, use `publish_microstrain_imu_sensor_frame_tf` + `microstrain_imu_sensor_child_frame`
to publish `imu_mount_child_frame`→`sensor` (identity) instead of overloading Livox.
Default `livox_extrinsic_yaw_deg` is **0** (typical bag laptop + forward Livox vs base +X). Reversed sensor:
merge `config/livox_extrinsic_yaw_180_overlay.yaml` via bringup_config, or set `ROS_PROJECT_SLAM_CONFIG` to
a machine YAML that overrides `livox_extrinsic_*` / `imu_mount_*` to match CAD.
`use_sim_time` is set from the launch arg (default **false** = wall clock). Bag replay: pass
``use_sim_time:=true`` and play with ``ros2 bag play ... --clock``. Start `launch_slam` *before* the bag
so `/clock` does not advance ahead of the stack (avoids flipped / mirrored first alignment).
**ZED (or any camera) on the same ROS_DOMAIN_ID:** if the ZED wrapper publishes ``odom``→``zed_camera_link``
(positional tracking) while this stack publishes ``map``→``odom`` and ``odom``→``base_link``, both compete
for ``odom``. Stopped or mis-clocked ZED nodes leave **old** ZED TF stamps in the buffer → ``TF_OLD_DATA``
for frame ``odom`` (``view_frames`` shows ``zed_*`` at past times vs fresh ``base_link``). Fix: disable ZED
publish_tf / odometry TF (camera-only), run ZED on another ``ROS_DOMAIN_ID``, or do not run ZED while SLAM
is validating TF.
-----------------------------------------------------------------------------

### slam_bringup

===========================================================================
KNOBS — only parameters listed in docs/tuning.md
===========================================================================

### use_microstrain_imu

EKF subscribes here when use_microstrain_imu is true. With microstrain_imu_origin: robot, this must match
the robot driver: Foxy lifecycle build publishes /imu/data only. Humble driver with raw-only streaming
uses /imu/data_raw — set that here if your robot runs the newer Microstrain stack.

### microstrain_imu_topic

local = serial driver on THIS machine when launch_sensors; robot = IMU on DDS only (default laptop).

### microstrain_imu_origin

When `launch_sensors:=false` and GX5 `/imu/data` uses `header.frame_id: sensor`, publish identity
`imu_mount_child_frame` → `microstrain_imu_sensor_child_frame` so EKF can transform ω,a into base_link.

### microstrain_imu_sensor_child_frame

TF `imu_mount_child_frame` → `microstrain_imu_sensor_child_frame` when driver is off (default identity).
If RViz shows Microstrain axes 90° wrong about +Z, set yaw ±90 (REP-103: positive yaw = CCW from above).

### microstrain_sensor_alias_pitch_deg

Rotation imu_link → GX5 `frame_id` (often `sensor`). Non-zero here + wrong imu_mount_yaw → wrong ωz in base_link → spin.

### microstrain_sensor_alias_yaw_deg

When launch_sensors:=false, publish ``imu_mount_*`` and Livox extrinsic static TFs from this YAML if true
(bag replay / laptop without robot ``/tf_static``). **true** by default so ``base_link``→``imu_link`` and
``base_link``→``livox_frame`` exist for EKF + deskew when this launch does not start the drivers.
Set **false** only after ``ros2 topic echo /tf_static --once`` confirms the robot already publishes the
same transforms (avoids duplicate ``static_transform_publisher`` warnings).

### publish_robot_static_tf_when_sensors_off

Odometry front-end: FAST-LIO (default) vs PCL NDT (`use_lidar_fusion: true`, `use_lio: false`).
Launch still allows `use_lio:=true|false` / `use_lidar_fusion:=...` to override merged YAML.

### use_lidar_fusion

Match docs/tuning.MD. Loosen only if rosout shows "LiDAR odom EKF update rejected (NIS gate)".
Keep LiDAR fusion conservative to avoid EKF yaw-basin flips.

### ekf_lidar_pose_var

Measurement variance on LiDAR yaw (rad²): **smaller** → Kalman trusts /lidar/odom yaw **more** (pulls EKF heading toward NDT).
**Larger** → trusts LiDAR yaw less → heading follows IMU integration more. If NDT yaw matches motion but /ekf/odom spins, try **lowering** (e.g. 0.012–0.018).

### ekf_lidar_gate_nis

Optional: set to /lidar/relative_motion to fuse NDT step into EKF vx,vy (can disturb map yaw vs scan — leave "" unless tuned).

### ekf_lidar_stamp_offset_sec

ekf_node: throttled LiDAR fusion line (nis, innov_xy/yaw, gated vs soft apply). Enable while diagnosing spin / NIS rejects.

### ekf_lidar_fusion_debug_throttle_sec

After gated (x,y,yaw) reject on NDT path, one soft ungated step (default true — fixes small-Δxy rejects).

### ekf_lidar_soft_fuse_after_gate_reject

true: fuse only x,y from /lidar/odom (ignore NDT yaw — use with imu_gyro_z_bias calibration or stable gyro).

### ekf_lidar_fuse_xy_only

Mean ωz on /imu/data at rest (after TF to base_link), same sign as reading. Use 0 when auto bias is on.

### ekf_imu_gyro_z_bias_rad_s

Bag starts still: average ωz for first N seconds (after TF), subtract automatically (+ manual above).

### ekf_imu_auto_gyro_z_bias_tune_sec

1.0 normal; -1.0 if integrated yaw runs opposite (TF/sign experiment).

### ekf_imu_gyro_z_scale

Below this planar speed (m/s), inflate LiDAR pose variance — gate uses ``max(|twist|,|EKF v|)``.
Re-enabled to calm stop/slow jitter without weakening normal-motion fusion.

### ekf_lidar_pose_var_below_slow_speed_scale

NDT defaults tuned vs FAST-LIO (locked-window bench 2026-05: voxel 0.46 vs 0.38 lowered align RMSE).

### lidar_voxel_leaf_size

Moderate acceptance (too tight → brittle scan_to_map + NIS rejects on hard segments).

### lidar_min_points_per_cloud

Re-enable TF prior to keep NDT heading in the same basin as base_link motion.
Debug: false → initial guess from integrated lidar pose only (not EKF TF).

### lidar_use_tf_initial_guess

Fine phase trim for residual yaw smear (small negative keeps IMU slightly ahead of cloud stamp).

### keyframe_deskew_cloud_stamp_offset_sec

Loosen if deskew falls back during motion (less scan smear in turns; try 0.10–0.15).

### keyframe_deskew_enable

false: deskew uses ``keyframe_deskew_imu_topic`` (default /livox/imu) — **same clock as Livox
per-point timestamps**, best for turn deskew. true: EKF IMU rotated into livox_frame; stamps
rarely align with Livox sweep time → residual skew/smear when rotating despite correct TF.

### keyframe_deskew_imu_follow_ekf

During **turns**: more / tighter keyframes + stable deskew reduce curved walls. Straight runs stay good.

### keyframe_min_time_sec

Final merge voxel for ``/keyframe_map`` (centroid per cell). Smaller → sharper walls, more points.

### keyframe_max_pts_per_scan

With ``keyframe_apply_pose_graph_map: true``, each keyframe is stored at this voxel **before** pose-graph warp.
Package default 0.32 m is sparse on Livox; **0.12–0.18** restores detail (more RAM). Independent of ``keyframe_voxel_leaf_m`` merge step.

### keyframe_map_batch_store_voxel_m

Odom edges vs loop closure: high ``pose_graph_weight_loop`` can skew the map if a loop is soft/wrong.
If map twists after revisiting a place: lower loop weight (e.g. 40) or tighten ``keyframe_loop_overlap_ratio``.

### pose_graph_max_loop_edges

Stricter overlap → fewer false loop edges (see launch_slam keyframe_overrides).

### keyframe_loop_overlap_ratio

===========================================================================
REST OF BRINGUP (required by launch_slam.launch.py)
===========================================================================

### microstrain_imu_data_rate

Merged over apt `microstrain_inertial_driver_common/config/params.yml` before port/frame overrides.
Set to "" to skip (vendor defaults only).

### imu_mount_pitch_deg

Yaw from base_link → imu_link: use 0 if gyro axes match REP-103 base (fix path spin from wrong ωz).
If ``/ekf/path`` **heading** looks correct but **translation** still moves opposite drive direction,
the Microstrain linear accel may be wrong in ``base_link`` — try **``imu_mount_yaw_deg: 180.0``**
(or 90 / −90 per CAD) so IMU integration matches LiDAR odom; see docs/imu_mount_note.md.

### imu_mount_yaw_deg

Livox node from this launch only when ``launch_sensors:=true`` (ignored when sensors off).

### start_livox_driver

RViz on this machine; use false when RViz runs on another PC (same ``ROS_DOMAIN_ID``).

### start_keyframe_map

Disable loop closure / pose graph while front-end (NDT + EKF) is being validated against FAST-LIO.

### start_pose_graph

Pose-graph drift correction — pick **one** (see ``keyframe_scan_map/README.md``):
**A)** ``pose_graph_publish_map_odom_tf: true`` + ``keyframe_apply_pose_graph_map: false`` — TF-only fix:
merged map points are **not** warped when ``map``→``odom`` updates → ghost walls / layered scans.
**B)** ``pose_graph_publish_map_odom_tf: false`` + ``keyframe_apply_pose_graph_map: true`` — rebuild ``/keyframe_map``
from corrected ``/pose_graph/corrected_keyframes`` (launch adds **static** identity ``map``→``odom``).
Prefer **B** for cleaner walls when loops run; use **A** if you cannot afford map rebuilds.

### livox_extrinsic_pitch_deg

LiDAR is mounted facing backwards: rotate base_link -> livox_frame by 180° about +Z.

### livox_cloud_frame_id

Child frame for TF livox_cloud_frame_id → child (default `livox_imu`). Not `sensor` — see header comment.

### livox_imu_child_frame

Livox built-in IMU frame vs cloud frame (bag laptop; driver off). Default identity; set yaw 180 if chip is
reversed about +Z vs `livox_frame` after `livox_extrinsic_*` is correct.

### livox_imu_bridge_yaw_deg

Identity ``livox_frame``→``livox_imu`` when this launch does not start the Livox node (deskew / TF).
Set false only if another node already publishes that link on ``/tf_static``.

### lidar_cloud_topic

When lidar_odom_smooth_enable is true, NDT publishes raw here; smoothed poses go to lidar_odom_topic.

### lidar_odom_smooth_enable

`xyz` = smooth x,y,z only; orientation/twist pass-through (validated vs FAST-LIO locked window; fixes EMA yaw lag vs `full`).

### lidar_odom_smooth_mode

EMA F1 (α_pose 0.10, α_twist 0.12): best vs FAST-LIO on locked-window replays; F1p5 (0.12/0.14) worse RMSE/θ.

### lidar_ndt_transformation_epsilon

NDT VoxelGridCovariance (PCL 1.12): more points per cell + eigen inflation reduce invalid-eigen spam on sparse Livox cells.

### lidar_ndt_resolution

scan_to_map: if > 0, run a coarse NDT pass then refine at lidar_ndt_resolution (translation stability).

### lidar_ndt_coarse_resolution

After NDT: blend planar yaw between EKF/T_pred (weight) and NDT (1-weight). Requires fuse_prior true.
false: use NDT planar yaw (do not blend toward EKF prior — avoids IMU yaw overpowering LiDAR).

### lidar_ndt_prior_yaw_blend

Large |Δyaw| with tiny planar correction → likely corridor degeneracy (reject, use prediction).

### lidar_ndt_corridor_spin_max_corr_xy_m

If > 0 and planar |Δxy| from NDT is below this, reject and use prediction (can false-trigger when stopped).

### lidar_ndt_fallback_if_planar_correction_below_m

Reject update if planar NDT base step vs EKF base step dot < 0 (both steps above mins below).

### lidar_ndt_opposite_motion_min_ndt_step_m

If either > 0, only merge aligned scan into map when Δxy or |Δyaw| exceeds threshold (keyframe map).

### lidar_registration_mode

Wider window: EKF TF at exact cloud stamp can lag a few ms at startup; avoids first-scan fallback.

### lidar_tf_initial_guess_timeout_sec

Start NDT this many seconds after EKF (bag/DDS: EKF needs a few IMU steps before odom->base_link exists).
Set 0.0 on a live robot if you want NDT from the first instant.

### lidar_node_start_delay_sec

After static TFs: delay ``ekf_node`` so tf2 has ``base_link``→``imu_link`` (and Livox mounts) before
first IMU / deskew lookups — avoids transient "frame does not exist" and TF stamp races at startup.
Set **0.0** if you need EKF from the first instant (e.g. tests).

### ekf_node_start_delay_sec

After EKF + /lidar/odom: static map→odom exists; FAST-LIO + relay + lidar link TF should be stable.

### keyframe_map_node_start_delay_sec

Run after keyframe so /keyframe_map/keyframes exists before first subscribe.

### pose_graph_node_start_delay_sec

Drop first N Livox clouds before any keyframe (Jetson: avoids flipped ghost from TF/IMU init races).

### lidar_map_max_points

scan_to_map: every N successful NDT steps, rebuild target map from last K aligned scans (0=off).
Helps map poisoning but drops long-range structure → can increase drift / hurt EKF; enable only when tuning that tradeoff.

### lidar_scan_to_map_refresh_keep_scans

scan_to_map: true = sensor-frame source + T_odom_sensor_pred guess (recommended); false = legacy odom-frame source.

### lidar_scan_to_map_register_sensor_frame

Per-scan planar NDT correction vs prediction: stdout + logger lines "NDT_RELATIVE: tx, ty ..." (scan_to_map / scan_to_scan).

### lidar_log_ndt_relative

Stdout POSE: x,y after each scan_to_map step (and PRED vs NEW when registration debug on).

### lidar_log_accumulated_pose

Skip NDT until EKF odom prior has moved this far in xy (m), reducing early bad alignment / map poisoning.
ndt_gate_force_after_sec > 0 allows NDT anyway after that time (e.g. rotation-in-place at start).
0 → do not wait for EKF prior motion before running NDT (avoids “never runs” if EKF stuck).

### lidar_ndt_gate_force_after_sec

``auto`` → true with LIO (fuses z + full pose); then ``ekf_lidar_fuse_xy_only`` is ignored. Use **false** for planar + xy_only.

### ekf_lidar_use_roll_pitch

Overridden to microstrain_imu_topic when use_microstrain_imu is true (EKF uses GX5 on /imu/data).

### ekf_base_link_frame

Publish-only yaw on ``/ekf/odom``/pose/TF (not fusion). **Twist** is world→base_link at this heading.
Keep 0 unless you have verified /lidar/odom yaw+translation are consistent and only the RViz triad is backwards.

### ekf_publish_tf

LIO: relay can publish odom->base_link TF (set true) so EKF does not — single authority.
Default false: relay TF is only ~LiDAR rate → sparse /tf vs EKF+cloud-stamp TF → tf2 interpolation
smear in keyframe/RViz. Prefer EKF TF unless you disable EKF or add a high-rate TF filler.

### lio_relay_publish_tf

When relay owns TF: also publish odom->base_link at each LiDAR cloud stamp (topic below).

### lio_relay_sync_tf_cloud_topic

FAST-LIO ``body`` vs robot ``base_link`` (REP-103 +X forward). Relay uses **quaternion_multiply**
(not Euler+yaw) so heading stays consistent with LiDAR ``(x,y)``. Set **0.0** if ``/ekf/path`` +X already matches nose.
With LiDAR extrinsic yaw fixed above, FAST-LIO body should align with base_link; keep relay yaw at 0.

### lio_relay_body_to_base_yaw_deg

~40 Hz effective /imu/data on typical GX5 @115200; use_stamp_dt still refines per step.

### ekf_use_stamp_dt

null = keep predict_use_linear_accel from ekf_params_yaml (GX5 preset often false). true/false
forces override — true can strengthen translation for NDT TF guess on bag replay; false reduces accel junk when tilted/bumpy.
If LiDAR odom heading is right but **path still walks backward**, try **false** to rely on LiDAR xy more than IMU accel integration.

### keyframe_map_publish_min_interval_sec

false: skip keyframes when TF missing at cloud stamp (reduces smear from wrong/latest pose).

### keyframe_tf_future_extrapolation_use_latest

Map insertion: T_map<-livox = T_map<-odom (TF) * T_odom<-base (/lidar/odom) * T_base<-livox (TF).

### keyframe_lidar_odom_topic

Nearest /lidar/odom stamp vs cloud (only if approximate_sync false). Prefer sync below.

### keyframe_lidar_odom_approx_sync_slop_sec

Small queue + shallow LiDAR sub QoS avoids multi-second backlog when deskew is CPU-heavy.

### keyframe_lidar_odom_approx_sync_queue_size

Sensor-frame prefilter: remove very near self-hits (RTK mast/chassis), far clutter, weak returns.

### keyframe_prefilter_self_bbox_enable

LiDAR mount is forward of base_link origin (x=+0.12, y≈0): bias self mask forward.

### keyframe_deskew_imu_topic

For residual rotational (yaw) smear, keep deskew in planar yaw mode.

### keyframe_deskew_model

~1.3 s at 200 Hz — enough for a sweep; very large buffers widen IMU time span unnecessarily.

### keyframe_deskew_imu_rotation_tf_timeout_sec

Livox /livox/imu often uses frame_id ``sensor``; that must not use Microstrain's ``sensor`` TF.

### keyframe_rotation_keyframe_scale

Safety gate (disabled by default): drop clouds with implausibly large jumps vs last keyframe.

### pose_graph_map_odom_tf_period_sec

**1.0** = map→odom matches each graph solve. Lower α can ease jumps but may add visible lag/skew in map.

### pose_graph_map_odom_tf_smooth_alpha

Reject ``/ekf/odom`` header stamps this far behind wall clock when stamping map→odom (stale DDS / replay).

### lio_overlay_params_file

When ``use_lio`` and ``launch_sensors:=false`` (bag / DDS laptop), merged **after** lio_overlay:
PointCloud2 without Livox ``tag``/``line`` → ``preprocess.lidar_type: 0`` + ``publish_tf: true``.
Set to "" to skip (e.g. custom Livox handler). On robot (``launch_sensors:=true``) this is never loaded.

### lio_bag_overlay_params_file

Bag + LIO: start ``tf_bridge_fastlio_odom_compare`` (static ``odom``->``camera_init``) so
``odom_trajectory_tools`` / RViz can relate ``camera_init``/``body`` to EKF ``odom``->``base_link``.
Set false if you already publish that edge or run the bridge manually.

## `src/ros_project_bringup/launch/launch_slam.launch.py`

### module docstring

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

### _resolve_slam_bringup_overlay_path

Path to a YAML file merged over installed defaults: ``slam_bringup`` mapping.

### _load_slam_bringup_config

If ``overlay_path`` is the same file as the installed default, return it once. Otherwise
    deep-merge: installed ``slam_bringup`` dict then overlay (partial files supported).

### docstring

Return base rviz config path or a temp config with Keyframe map point size overrides.

### top

!/usr/bin/env python3

### top

=============================================================================

### top

Config: `config/slam_bringup.yaml` — loaded at launch (see _load_slam_bringup_config)

### top

=============================================================================

### launch_setup

CLI overrides (empty = keep value from slam_bringup YAML merge).

### launch_setup

Mount TF: with local USB driver, publish mount here. With ``launch_sensors:=false`` (bag / DDS laptop),

### launch_setup

many bags omit ``base_link``→``imu_link``; optional publish from ``imu_mount_*`` (see

### launch_setup

``publish_robot_static_tf_when_sensors_off``). For ``robot`` IMU on a live robot that already

### launch_setup

publishes this static TF on DDS, set that key false to avoid duplicates.

### launch_setup

map ≡ odom: explicit identity so map→odom is clearly valid (odometry lives in odom; map aligned until pose_graph Option 3).

### launch_setup

Static TF often shows a huge “rate” in tf2_monitor; that is normal, not “missing data”.

### launch_setup

Livox IMU chip vs cloud frame when the driver is not on this host. Child must NOT be

### launch_setup

`sensor` if Microstrain uses that frame_id — it would steal TF from the GX5 (see slam_bringup header).

### launch_setup

Vendor params often default imu_frame_id to "sensor"; Imu.header may use this, not frame_id.

### launch_setup

Local Livox or same topic over DDS when sensors run elsewhere.

### launch_setup

Stamp sync (Microstrain vs Livox) + TF sample every cloud when NDT lags/skips.

### launch_setup

Livox extrinsic static TF: always when local driver; when sensors off, publish from YAML if enabled

### launch_setup

(bag replay often has no ``base_link``→``livox_frame`` on /tf_static).

### launch_setup

EKF must publish odom->base_link before NDT uses it as scan_to_map initial guess (avoid first-cloud fallback).

### launch_setup

EMA low-pass: NDT publishes raw; this republishes smoothed to ``lidar_odom_topic`` for EKF/tools.

### launch_setup

Bag replay only: merge fastlio_bag_replay_overlay (lidar_type 0) when ``use_sim_time`` is true.

### launch_setup

Do **not** merge on live ``launch_sensors:=false`` (Jetson + Livox on DDS) — that would override

### launch_setup

MID360 ``lidar_type: 4`` and break preprocessing (log shows ``p_pre->lidar_type 0``).

### launch_setup

Connect FAST-LIO world (camera_init/body) to robot odom so recorders can compose body->base_link.

## `src/ros_project_bringup/launch/tf_bridge_fastlio_odom_compare.launch.py`

### module docstring

TF helpers so FAST-LIO /Odometry (camera_init -> body) can be projected into robot `odom` + `base_link`
when recording with `odom_trajectory_tools.py record --output-parent-frame odom --compose-output-child base_link`.

Typical replay issues:
- FAST-LIO publishes `camera_init` -> `body` but nothing connects `camera_init` to robot `odom`.
- Bag has `map` -> `base_link` (or `map` -> `odom` -> `base_link`) but `tf2_echo odom base_link` still fails
  because `odom` was never on the same tree as `map`.

This launch (with sim time) publishes:
1) **Always:** static `odom` -> `camera_init` (identity). Do not start if your bag already defines a
   different `odom` -> `camera_init` edge.
2) **Optional (`bridge_odom_to_map:=true`):** static `odom` -> `map` (identity). Use **only** when
   the bag has `map` -> `base_link` (or similar) but **no** `map`->`odom` and **no** `odom`->`map`
   (i.e. `map` was the world root). **Do not** enable if the bag already publishes `map`->`odom` or
   `odom`->`map` (would create conflicting parents).
3) **Optional (`bridge_body_to_base_link:=true`):** static `body` -> `base_link` (identity). Prefer **`false`**
   when **EKF** publishes `odom`→`base_link` (else two parents for `base_link`); use FAST-LIO
   **`publish_tf:=true`** instead so `body` is on `/tf`. Enable `bridge_body_to_base_link` only when
   no EKF/bag `odom`→`base_link` and **body** would otherwise be unreachable from **base_link**.

After bridges + bag play + FAST-LIO, verify:
  ros2 run tf2_ros tf2_echo odom base_link
  ros2 run tf2_ros tf2_echo body base_link

### top

!/usr/bin/env python3

## `src/ros_project_bringup/ros_project_bringup/__init__.py`

### top

ROS project bringup package

## `src/ros_project_bringup/ros_project_bringup/lidar_odom_ema_smooth.py`

### module docstring

Exponential moving average filter for nav_msgs/Odometry (NDT -> EKF bridge).

NDT publishes to ``in_topic`` (typ. /lidar/odom_raw); this node publishes ``out_topic``
(typ. /lidar/odom) so EKF and tooling see a smoother pose.

Modes:
  xy   — smooth position.x,y only; z,q,twist from measurement
  xyz  — smooth x,y,z; orientation + twist from measurement
  full — smooth xyz + euler RPY + twist.xyz (helps reduce yaw jitter in EKF)

### top

!/usr/bin/env python3

### __init__

use_sim_time is set by launch (sim_time_param); do not re-declare → ParameterAlreadyDeclaredException

### _cb

Position EMA

### _cb

Orientation

### _cb

Twist linear EMA when full twist smoothing helps shape

## `src/ros_project_bringup/ros_project_bringup/ndt_ekf_time_diagnose.py`

### module docstring

One-terminal probe for NDT stuck / rotate-only: TF at cloud time vs NDT step sizes.

Run while launch_slam (NDT path) + bag play are running.

What it checks
--------------
1) ``lookup_transform(odom, base_link, cloud.header.stamp)`` — same query NDT uses for the
   scan_to_map prior. FAIL here ⇒ NDT falls back to its last integrated pose ⇒ tiny /lidar/odom
   bubble while /ekf/odom can still move (IMU + weak LiDAR).

2) ``lookup_transform(..., Time())`` — latest TF. If (1) fails but (2) works ⇒ time / stamp
   mismatch between EKF-published TF and LiDAR stamps.

3) Last ``/lidar/relative_motion`` (dx, dy, dθ): if dx,dy stay ~0 while TF (2) translates a lot,
   NDT is skipping (fitness / convergence) or the prior chain is wrong.

How to read
-----------
- ``at_cloud_stamp=FAIL`` with "extrapolation into the future" but ``TF_latest=OK`` ⇒ the cloud
  stamp is **ahead of** the newest ``odom``→``base_link`` sample (NDT/diagnose often run **before**
  ``ekf_node``'s ``/livox/lidar`` callback publishes that stamp). **Not** a broken graph; NDT can use
  latest TF as initial guess (see ``lidar_odometry_node``). ``FAIL`` **and** ``TF_latest=FAIL`` ⇒
  real TF / clock / sim_time problem.
- ``at_cloud_stamp=OK`` but ``|dx|,|dy|`` ~ 0 and ``|dtheta|`` not ⇒ NDT aligns rotation only
  (geometry / extrinsic / degenerate map) — next: rosout NDT fitness / convergence, livox extrinsic.
- ``at_cloud_stamp=OK`` and ``|dx|,|dy|`` non-zero but /lidar/odom still flat ⇒ check EMA on
  ``/lidar/odom`` vs ``/lidar/odom_raw``.

### top

!/usr/bin/env python3

## `src/ros_project_bringup/ros_project_bringup/pipeline_translation_debug.py`

### module docstring

Trace planar translation: LiDAR odom (default /lidar/odom = LIO relay or smoothed NDT) vs EKF vs TF.

Run beside launch + bag (same ROS_DOMAIN_ID, ROS_USE_SIM_TIME=true).

How to read
-----------
- **step_lidar**: |Δxy| between consecutive **NDT** odometry messages (~10 Hz).
- **step_ekf**: |Δxy| of **EKF** state sampled at those same LiDAR times (snapshots).
- **ratio_window** (rolling): sum(step_ekf) / sum(step_lidar). Near **1** ⇒ EKF tracks NDT translation.
  Near **0** with nonzero step_lidar ⇒ translation dies **between NDT output and EKF/TF** (fusion / prediction).
- **step_tf**: |Δxy| from TF **odom→base_link** at each LiDAR message stamp (falls back to latest if lookup fails).

If **step_lidar ≈ 0** always ⇒ problem is **upstream of EKF** (NDT / topics / replay).

If **step_lidar >> 0** but **step_ekf ≈ 0** ⇒ **EKF** is not integrating LiDAR translation (or IMU-only prediction dominates).

If **step_ekf >> 0** but **step_tf ≈ 0** ⇒ **TF publish** / stamp / RViz frame issue.

### _tf_xy_for_stamp

TF odom→base at cloud/odom stamp; fallback to latest if buffer cannot extrapolate.

### top

!/usr/bin/env python3

### __init__

Odometry

### _on_summary

Interpretation hint

## `src/ros_project_bringup/setup.py`

### top

launch_ros Node() resolves executables under lib/<pkg>/; setuptools puts the real script in bin/.
