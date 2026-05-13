#!/usr/bin/env bash
set -eo pipefail

# Capture runtime ROS assets for dissertation figures.
# Run while your stack is active (and while bag is playing for replay captures).

WS_DIR="${WS_DIR:-$HOME/ROS_Deployment}"
OUT_ROOT="${OUT_ROOT:-$WS_DIR/docs/dissertation_assets/runtime}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="$OUT_ROOT/$STAMP"
# Capture tuning knobs:
#   FAST_MODE=1                  -> skip heavy steps (topic info/TF/node details/colcon)
#   HZ_TIMEOUT_SEC=12            -> max wall-time per topic hz probe
#   HZ_WINDOW=6                  -> samples for ros2 topic hz -w
#   TOPIC_INFO_ALL=0             -> collect topic info only for key topics
FAST_MODE="${FAST_MODE:-0}"
HZ_TIMEOUT_SEC="${HZ_TIMEOUT_SEC:-20}"
HZ_WINDOW="${HZ_WINDOW:-10}"
TOPIC_INFO_ALL="${TOPIC_INFO_ALL:-1}"

mkdir -p "$OUT_DIR"

# ROS setup scripts can reference unset vars (e.g. AMENT_TRACE_SETUP_FILES).
# Temporarily disable nounset while sourcing to avoid false failures.
set +u
source /opt/ros/humble/setup.bash
source "$WS_DIR/install/setup.bash"
set -u

{
  echo "timestamp: $(date -Iseconds)"
  echo "hostname: $(hostname)"
  echo "ros_distro: ${ROS_DISTRO:-unknown}"
  echo "ros_domain_id: ${ROS_DOMAIN_ID:-unset}"
  echo "ros_use_sim_time_env: ${ROS_USE_SIM_TIME:-unset}"
} > "$OUT_DIR/session_info.txt"

echo "[1/7] Capturing nodes and topics..."
ros2 node list > "$OUT_DIR/node_list.txt"
ros2 topic list -t > "$OUT_DIR/topic_list_types.txt"
ros2 topic list > "$OUT_DIR/topic_names_snapshot.txt"

echo "[2/7] Capturing topic rates for key topics (10s each)..."
KEY_TOPICS=(
  "/livox/lidar"
  "/livox/imu"
  "/imu/data"
  "/lidar/odom_raw"
  "/lidar/odom"
  "/ekf/odom"
  "/tf"
)

for topic in "${KEY_TOPICS[@]}"; do
  :
done

# Run hz probes in parallel so one slow topic does not block all others.
# Per-topic timeout keeps this stage bounded even if a topic is idle.
HZ_PIDS=()
for topic in "${KEY_TOPICS[@]}"; do
  if grep -Fxq -- "$topic" "$OUT_DIR/topic_names_snapshot.txt"; then
    out_file="$OUT_DIR/hz_${topic//\//_}.txt"
    (
      timeout "${HZ_TIMEOUT_SEC}s" ros2 topic hz "$topic" -w "$HZ_WINDOW" > "$out_file" 2>&1 || true
    ) &
    HZ_PIDS+=("$!")
  fi
done
for pid in "${HZ_PIDS[@]}"; do
  wait "$pid" || true
done

echo "[3/7] Capturing verbose topic info (QoS + endpoints)..."
TOPIC_FILE="$OUT_DIR/topic_names.txt"
if [[ "$FAST_MODE" == "1" ]]; then
  echo "FAST_MODE=1 -> skipping topic info sweep" > "$OUT_DIR/topic_info_skipped.txt"
else
  if [[ "$TOPIC_INFO_ALL" == "1" ]]; then
    ros2 topic list > "$TOPIC_FILE"
  else
    : > "$TOPIC_FILE"
    for t in "${KEY_TOPICS[@]}"; do
      if grep -Fxq -- "$t" "$OUT_DIR/topic_names_snapshot.txt"; then
        echo "$t" >> "$TOPIC_FILE"
      fi
    done
  fi
  while IFS= read -r topic; do
    [[ -z "$topic" ]] && continue
    safe_name="${topic//\//_}"
    ros2 topic info -v "$topic" > "$OUT_DIR/topic_info_${safe_name}.txt" 2>&1 || true
  done < "$TOPIC_FILE"
fi

echo "[4/7] Capturing TF tree..."
if [[ "$FAST_MODE" == "1" ]]; then
  echo "FAST_MODE=1 -> skipping TF tree capture" > "$OUT_DIR/view_frames.log"
elif command -v ros2 >/dev/null 2>&1; then
  # view_frames writes frames_<timestamp>.{gv,pdf} in cwd.
  (
    cd "$OUT_DIR"
    ros2 run tf2_tools view_frames > view_frames.log 2>&1 || true
  )
fi

echo "[5/7] Capturing node details for major nodes..."
MAJOR_NODES=(
  "/ekf_node"
  "/lidar_odometry_node"
  "/keyframe_map_node"
  "/pose_graph_node"
  "/microstrain_inertial_driver"
)
NODE_FILE="$OUT_DIR/node_names_snapshot.txt"
ros2 node list > "$NODE_FILE"
if [[ "$FAST_MODE" == "1" ]]; then
  echo "FAST_MODE=1 -> skipping node details" > "$OUT_DIR/node_details_skipped.txt"
else
  for node in "${MAJOR_NODES[@]}"; do
    if grep -Fxq -- "$node" "$NODE_FILE"; then
      ros2 node info "$node" > "$OUT_DIR/node_info_${node//\//_}.txt" 2>&1 || true
      ros2 param dump "$node" > "$OUT_DIR/params_${node//\//_}.yaml" 2>/dev/null || true
    fi
  done
fi

echo "[6/7] Capturing launch/package graph text..."
if [[ "$FAST_MODE" == "1" ]]; then
  echo "FAST_MODE=1 -> skipping colcon graph capture" > "$OUT_DIR/colcon_capture_skipped.txt"
else
  colcon list --names-only > "$OUT_DIR/colcon_packages.txt" 2>/dev/null || true
  colcon graph > "$OUT_DIR/colcon_graph.txt" 2>/dev/null || true
fi

echo "[7/7] Creating quick index..."
{
  echo "# Runtime capture index"
  echo
  echo "- Output directory: \`$OUT_DIR\`"
  echo "- Node list: \`node_list.txt\`"
  echo "- Topic list/types: \`topic_list_types.txt\`"
  echo "- Topic QoS/endpoints: \`topic_info_*.txt\`"
  echo "- TF tree: \`frames_*.gv\`, \`frames_*.pdf\` (if generated)"
  echo "- Per-node info: \`node_info_*.txt\`"
  echo "- Per-node params: \`params_*.yaml\`"
  echo "- Topic rates: \`hz_*.txt\`"
} > "$OUT_DIR/INDEX.md"

echo "[ok] Runtime assets written to: $OUT_DIR"
