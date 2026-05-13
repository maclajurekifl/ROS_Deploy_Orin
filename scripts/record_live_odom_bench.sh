#!/usr/bin/env bash
# Record a short live bag for NDT / EKF / TF / FAST-LIO diagnosis (30–60 s typical).
#
# Usage:
#   ./scripts/record_live_odom_bench.sh [duration_sec] [output_parent_dir]
# Example:
#   ./scripts/record_live_odom_bench.sh 45 /tmp
#
# Prerequisites: workspace sourced (or this script sources install/setup.bash below).
# Topics are best-effort: missing topics only warn; trim the list if your robot has no FAST-LIO.
set -euo pipefail

DURATION="${1:-45}"
OUT_PARENT="${2:-/tmp}"
WS="${ROS_DEPLOY_WS:-/home/macla/ROS_Deployment}"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ERROR: /opt/ros/humble/setup.bash not found" >&2
  exit 1
fi
# shellcheck source=/dev/null
source /opt/ros/humble/setup.bash
if [[ -f "${WS}/install/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "${WS}/install/setup.bash"
fi

STAMP="$(date +%Y%m%d_%H%M%S)"
BAG_DIR="${OUT_PARENT%/}/odom_bench_${STAMP}"
mkdir -p "${BAG_DIR}"

echo "Recording ${DURATION}s → ${BAG_DIR}"
echo "  (Ctrl+C stops early; timeout ends cleanly at ${DURATION}s)"
echo ""

# Live robot: do NOT require /clock.
# Add/remove topics to match your stack (FAST-LIO must be running for /Odometry).
# Exit 124 = timeout reached (normal). Other non-zero = check ros2 bag stderr.
timeout --signal=INT "${DURATION}" ros2 bag record -o "${BAG_DIR}" \
  /livox/lidar \
  /livox/imu \
  /imu/data \
  /tf \
  /tf_static \
  /lidar/odom \
  /lidar/odom_raw \
  /Odometry \
  /ekf/odom \
  /lidar/relative_motion \
  /lidar/pose_correction

META="${BAG_DIR}/RECORDING_META.txt"
{
  echo "duration_sec_requested=${DURATION}"
  echo "wall_clock_end=$(date -Iseconds)"
  echo "bag_directory=${BAG_DIR}"
  echo "ros_distro=humble"
} >"${META}"

echo ""
echo "Done. Bag directory: ${BAG_DIR}"
echo "List: ls -la ${BAG_DIR}"
echo "Info: ros2 bag info ${BAG_DIR}"
