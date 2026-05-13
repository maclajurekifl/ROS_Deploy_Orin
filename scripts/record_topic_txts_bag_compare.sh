#!/usr/bin/env bash
# Record selected ROS 2 topics to text files for ~30 s (wall clock), for bag replay comparisons.
#
# Usage (after: bag play --clock + launch NDT or FAST-LIO + source install):
#   ./scripts/record_topic_txts_bag_compare.sh NDT 30
#   ./scripts/record_topic_txts_bag_compare.sh LIO 30
#
# Outputs (example PREFIX=LIO, DURATION=30):
#   /tmp/LIO_lidar_odom.txt
#   /tmp/LIO_lidar_odom_raw.txt
#   /tmp/LIO_ekf_odom.txt
#   /tmp/LIO_Odometry.txt
#   /tmp/LIO_livox_imu.txt
#   /tmp/LIO_tf.txt
#   /tmp/LIO_tf_static.txt
#   optionally /tmp/LIO_livox_lidar.txt if RECORD_LIVOX_LIDAR=1
#
# Env:
#   OUT_DIR=/tmp           output directory
#   RECORD_LIVOX_LIDAR=1   also echo /livox/lidar (very large files)
#
set -eo pipefail
# Do not enable -u before sourcing ROS setup.bash: it tests $AMENT_TRACE_SETUP_FILES unset.

PREFIX="${1:?Usage: $0 <PREFIX e.g. NDT or LIO> [duration_sec]}"
DURATION="${2:-30}"
OUT_DIR="${OUT_DIR:-/tmp}"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ERROR: source ROS 2 Humble first or fix path" >&2
  exit 1
fi
# shellcheck source=/dev/null
source /opt/ros/humble/setup.bash
WS="${ROS_DEPLOY_WS:-/home/macla/ROS_Deployment}"
if [[ -f "${WS}/install/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "${WS}/install/setup.bash"
fi
set -u

mkdir -p "${OUT_DIR}"

topic_to_fname() {
  local t="$1"
  echo "${t#/}" | tr '/' '_'
}

declare -a TOPICS=(
  /lidar/odom
  /lidar/odom_raw
  /ekf/odom
  /Odometry
  /livox/imu
  /tf
  /tf_static
)

if [[ "${RECORD_LIVOX_LIDAR:-0}" == "1" ]]; then
  TOPICS+=(/livox/lidar)
fi

META="${OUT_DIR}/${PREFIX}_recording_meta.txt"
{
  echo "prefix=${PREFIX}"
  echo "duration_sec=${DURATION}"
  echo "wall_start=$(date -Iseconds)"
  echo "out_dir=${OUT_DIR}"
  echo "topics=${TOPICS[*]}"
} | tee "${META}"

declare -a PIDS=()
for t in "${TOPICS[@]}"; do
  fn="${OUT_DIR}/${PREFIX}_$(topic_to_fname "$t").txt"
  echo "Recording $t -> $fn"
  : >"$fn"
  ros2 topic echo "$t" >>"$fn" &
  PIDS+=($!)
done

sleep "${DURATION}"

for pid in "${PIDS[@]}"; do
  kill "$pid" 2>/dev/null || true
done
wait 2>/dev/null || true

{
  echo "wall_end=$(date -Iseconds)"
} >>"${META}"

echo ""
echo "Done. Files in ${OUT_DIR}/${PREFIX}_*.txt"
ls -lh "${OUT_DIR}/${PREFIX}"_*.txt 2>/dev/null || true
