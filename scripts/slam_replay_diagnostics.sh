#!/usr/bin/env bash
# Run checks A–G while (or without) SLAM + bag replay.
#
# Usage:
#   bash scripts/slam_replay_diagnostics.sh
#   bash scripts/slam_replay_diagnostics.sh ~/bags/session_01
#
# Prereq: colcon-built workspace. Sources Humble + this repo's install/setup.bash.
# Sets ROS_DISABLE_ROS2_DAEMON=1 so ros2 topic/list/echo avoid flaky daemon !rclpy.ok().
#
# A — /clock
# B — core topics present
# C — tf2_echo odom -> base_link (short sample)
# D — one /livox/lidar message (header)
# E — slam_symptom_classifier.py (bag-only)
# F — slam_symptom_classifier.py --live (skipped if no graph)
# G — sample /rosout (look for NDT / fitness / NIS / skip)
#
# See bottom of this file or run:  bash scripts/slam_replay_diagnostics.sh --print-terminals

set +e
# Do not use `set -u` before sourcing ROS setup.bash — it references vars like
# AMENT_TRACE_SETUP_FILES before defining them, which aborts under nounset.

BAG="${1:-$HOME/bags/session_01}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$SCRIPT_DIR/.." && pwd)"

if [[ "${1:-}" == "--print-terminals" ]]; then
  cat <<'EOF'
================================================================================
Terminal layout (same machine, same ROS_DOMAIN_ID)
================================================================================

--- Order: start Terminal 1, wait until nodes are up, then Terminal 2 ---

Terminal 1 — SLAM (NDT + EKF + keyframe; default)
  cd ~/ROS_Deployment
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  ros2 launch ros_project_bringup launch_slam.launch.py

Terminal 2 — bag (after Terminal 1 is ready)
  source /opt/ros/humble/setup.bash
  source ~/ROS_Deployment/install/setup.bash
  ros2 bag play ~/bags/session_01 --clock

Optional: strip recorded TF if you see TF_OLD_DATA / duplicates
  cd ~/ROS_Deployment
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  python3 scripts/bag_play_no_recorded_tf.py ~/bags/session_01 -- --clock

Terminal 3 — this diagnostics script (while 1 + 2 are running)
  cd ~/ROS_Deployment
  bash scripts/slam_replay_diagnostics.sh ~/bags/session_01

================================================================================
Terminal 4 — auto_fastlio_vs_slam (same runtime as 1 + 2)
================================================================================

The orchestrator needs ALL of these topics at once:
  /Odometry     (FAST-LIO)
  /lidar/odom   (NDT or LIO relay)
  /ekf/odom     (EKF)

Option A — one stack (simplest): FAST-LIO + relay + EKF (no NDT CSV; relay feeds /lidar/odom)
  Terminal 1:
    cd ~/ROS_Deployment
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    ros2 launch ros_project_bringup launch_slam.launch.py use_lio:=true use_lidar_fusion:=false
  Terminal 2: (same bag + --clock as above)
  Terminal 4:
    cd ~/ROS_Deployment
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    export ROS_DISABLE_ROS2_DAEMON=1
    python3 scripts/odom_trajectory_tools.py auto_fastlio_vs_slam --use-sim-time \\
      --out-dir /tmp/slam_tune_$(date +%Y%m%d_%H%M%S) \\
      --warmup-sec 5 --duration-sec 60 --wait-timeout-sec 300

  (--record-ndt-raw only when NDT publishes /lidar/odom_raw; omit for LIO-only.)

Option B — NDT + FAST-LIO together (true NDT vs FAST-LIO; heavier, two LiDAR consumers)
  Terminal 1: default launch_slam (NDT + EKF)
  Terminal 2: bag
  Terminal 3: FAST-LIO mapping + your bag replay overlay (see scripts/odom_trajectory_tools.py header)
  Terminal 3b (if needed for TF projection when recording):
    ros2 launch ros_project_bringup tf_bridge_fastlio_odom_compare.launch.py \\
      use_sim_time:=true bridge_body_to_base_link:=false
  Terminal 4: auto_fastlio_vs_slam command as in Option A, add --record-ndt-raw when /lidar/odom_raw exists

EOF
  exit 0
fi

export ROS_DISABLE_ROS2_DAEMON=1

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "error: /opt/ros/humble/setup.bash not found" >&2
  exit 1
fi
# shellcheck source=/dev/null
source /opt/ros/humble/setup.bash
if [[ ! -f "$REPO/install/setup.bash" ]]; then
  echo "error: $REPO/install/setup.bash not found — run colcon build" >&2
  exit 1
fi
# shellcheck source=/dev/null
source "$REPO/install/setup.bash"

section() {
  echo ""
  echo "########################################################################"
  echo "# $*"
  echo "########################################################################"
}

section "Daemon reset (best effort)"
ros2 daemon stop 2>/dev/null || true
sleep 0.7
ros2 daemon start 2>/dev/null || true
sleep 0.5

section "A — /clock (expect publisher when bag uses --clock)"
if timeout 6 ros2 topic echo /clock --once 2>/dev/null; then
  echo "[OK] received /clock"
else
  echo "[WARN] no /clock in 6s — is the bag playing with --clock?"
fi

section "B — Core topic presence"
TL=""
if TL="$(ros2 topic list 2>/dev/null)"; then
  while IFS= read -r t; do
    [[ -z "$t" ]] && continue
    if echo "$TL" | grep -qx "$t"; then
      echo "[OK] $t"
    else
      echo "[MISS] $t"
    fi
  done <<'TOPICS'
/clock
/livox/lidar
/imu/data
/lidar/odom
/lidar/odom_raw
/ekf/odom
/Odometry
/keyframe_map
TOPICS
else
  echo "[FAIL] ros2 topic list failed even with ROS_DISABLE_ROS2_DAEMON=1"
fi

section "C — tf2_echo odom -> base_link (~8s; Ctrl+C not needed)"
timeout 8 ros2 run tf2_ros tf2_echo odom base_link 2>&1 | tail -28
ec=${PIPESTATUS[0]}
if [[ "$ec" -eq 124 ]]; then
  echo "(timeout — if only 'Waiting for transform', start bag after SLAM or wait longer)"
elif [[ "$ec" -ne 0 ]]; then
  echo "[WARN] tf2_echo exit $ec"
fi

section "D — /livox/lidar first message (header only, ~8s)"
timeout 8 ros2 topic echo /livox/lidar --once 2>&1 | head -45

section "E — Symptom classifier (bag metadata only)"
if [[ ! -e "$BAG" ]]; then
  echo "[SKIP] bag not found: $BAG"
else
  python3 "$REPO/scripts/slam_symptom_classifier.py" "$BAG" --what smeared,rotate,drift
fi

section "F — Symptom classifier (--live; needs SLAM + bag)"
if TL="$(ros2 topic list 2>/dev/null)"; then
  python3 "$REPO/scripts/slam_symptom_classifier.py" "$BAG" --what smeared --live
else
  echo "[SKIP] ros2 topic list failed; cannot run --live"
fi

section "G — /rosout sample (~6s, scan for NDT / fitness / NIS / skip)"
timeout 6 ros2 topic echo /rosout rcl_interfaces/msg/Log --no-arr 2>/dev/null | head -120

section "Done"
echo "Tip: full terminal recipe ->  bash scripts/slam_replay_diagnostics.sh --print-terminals"
