# Scripts

Utilities for bag replay, odometry comparison, and SLAM diagnostics. Run from the **workspace root** after sourcing ROS and the overlay:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
cd ~/ROS_Deploy_Orin   # or your clone path
```

Python scripts need **`rclpy`**, **`matplotlib`** (plots in `odom_trajectory_tools.py`), and a built workspace. Bash scripts source Humble + `install/setup.bash` themselves.

---

## Python scripts (root)

### `odom_trajectory_tools.py`

Record **`nav_msgs/Odometry`** to CSV and compare FAST-LIO vs full SLAM (EKF) on the **same bag** and time window.

| Subcommand | What it does |
|------------|----------------|
| **`record`** | Subscribe to a topic; write CSV (`t_sec`, `x`, `y`, `yaw`, …). Use **`--use-sim-time`** with `ros2 bag play --clock`. |
| **`compare`** | Overlay two CSVs; PNG + text report (`--out-prefix`). |
| **`align_xy`** | SE(2) fit between paths (heading/offset vs shape). |
| **`plot`** | Plot one or more CSVs without a full compare report. |
| **`auto_fastlio_vs_slam`** | Orchestrates record/compare for NDT+LIO tuning runs (`--out-dir`, stamp window, optional **`--record-ndt-raw`**). |
| **`tune_rollup`** | Summarize several `auto_fastlio_vs_slam` output directories. |

**Typical bag workflow**

1. Terminal 1 — stack + bag:
   ```bash
   ros2 launch ros_project_bringup launch_slam.launch.py use_sim_time:=true launch_sensors:=false
   ros2 bag play ~/bags/session_01 --clock
   ```
2. Terminal 2 — record FAST-LIO (with TF bridge if needed; see script docstring):
   ```bash
   python3 scripts/odom_trajectory_tools.py record --use-sim-time --topic /Odometry \
     --start-sec 30 --end-sec 80 -o fastlio.csv
   ```
3. Re-run stack (or same pass) and record fused odom:
   ```bash
   python3 scripts/odom_trajectory_tools.py record --use-sim-time --topic /ekf/odom \
     --start-sec 30 --end-sec 80 -o slam.csv
   ```
4. Compare:
   ```bash
   python3 scripts/odom_trajectory_tools.py compare \
     --ref fastlio.csv:FAST-LIO --test slam.csv:SLAM --out-prefix /tmp/run_compare
   ```

Help: `python3 scripts/odom_trajectory_tools.py --help` and `python3 scripts/odom_trajectory_tools.py record --help`.

---

### `slam_symptom_classifier.py`

Maps bag or live evidence to tuning blocks **A–D** (time/TF/extrinsics, deskew, front-end odom/EKF, keyframe/map).

```bash
# Bag only (no ROS graph required beyond ros2 bag info)
python3 scripts/slam_symptom_classifier.py ~/bags/session_01

# Optional symptoms filter
python3 scripts/slam_symptom_classifier.py ~/bags/session_01 --what smeared,drift

# Live graph (source ROS + install in this shell first)
python3 scripts/slam_symptom_classifier.py --live
```

Symptom tokens: `smeared`, `zigzag`, `drift`, `stutter`, `curl`, `startup`, `rotate`.

---

### `compare_imu_gyro_base.py`

Prints Livox and Microstrain angular rates expressed in **`base_link`** while you rotate the robot (checks IMU frame / extrinsic wiring).

```bash
# With stack + bag or live sensors running
python3 scripts/compare_imu_gyro_base.py --duration 15
```

---

### `bag_play_no_recorded_tf.py`

Plays a bag on all topics **except** `/tf` and `/tf_static` (avoids conflicts when **`ekf_node`** and launch static TFs own the tree).

```bash
python3 scripts/bag_play_no_recorded_tf.py ~/bags/session_01 --clock
python3 scripts/bag_play_no_recorded_tf.py ~/bags/session_01 -- --clock -r 1.2
```

---

### `extract_comments_to_md.py`

Developer tool: moves long block comments from `src/` into **`readme/comments.md`**. Not used at runtime.

```bash
python3 scripts/extract_comments_to_md.py
```

---

## Shell scripts (root)

### `slam_replay_diagnostics.sh`

Runs checks **A–G** (clock, topics, TF sample, one LiDAR message, symptom classifier, rosout hints). Optional bag path argument.

```bash
bash scripts/slam_replay_diagnostics.sh
bash scripts/slam_replay_diagnostics.sh ~/bags/session_01
bash scripts/slam_replay_diagnostics.sh --print-terminals   # suggested multi-terminal layout
```

Prereq: built workspace; script sources Humble + `install/setup.bash`.

---

### `record_topic_txts_bag_compare.sh`

Echoes selected topics to text files for ~N seconds (wall clock) during bag replay — quick NDT vs LIO side-by-side logs.

```bash
./scripts/record_topic_txts_bag_compare.sh NDT 30
./scripts/record_topic_txts_bag_compare.sh LIO 30
```

Env: **`OUT_DIR`** (default `/tmp`), **`RECORD_LIVOX_LIDAR=1`** for huge `/livox/lidar` dumps.

---

### `record_live_odom_bench.sh`

Records a short **live** rosbag (odom, IMU, LiDAR, TF, etc.) for benching NDT/EKF/LIO.

```bash
./scripts/record_live_odom_bench.sh 45 /tmp
```

---

## Related docs

| Doc | Content |
|-----|---------|
| **`../README.md`** | Build, launch, default parameters |
| **`../readme/TUNING.md`** | EKF, deskew, NDT, keyframe tuning |
| **`../readme/deployment.md`** | Jetson / field deployment notes |
| **`../readme/autorun.md`** | Autorun / startup notes |
| **`../src/lidar_odometry/README.md`** | NDT node parameters |
| **`../src/LiDAR-Instructions.md`** | FAST-LIO setup and extrinsics |
