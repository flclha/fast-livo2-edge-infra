#!/usr/bin/env bash
set -euo pipefail

MODE="${1:?usage: run_offline_prediction_ground_ab.sh off|on [duration_seconds] [sigma] [trigger] [max_abs] [max_correction] [correction_gain]}"
DURATION="${2:-0}"
SIGMA="${3:-0.010}"
TRIGGER="${4:-0.010}"
MAX_ABS="${5:-0.060}"
MAX_CORRECTION="${6:-0.015}"
CORRECTION_GAIN="${7:-0.50}"
BAG="/home/nuc/versions/version5/sync_validation/version5_relative_20260902_173403/version5_relative_motion_120s.bag"
VERSION_ROOT="/home/nuc/versions/version5"
RESULT_ROOT="$VERSION_ROOT/offline_results/prediction_ground_constraint_20260929"

case "$MODE" in
  off) ENABLED=false ;;
  on) ENABLED=true ;;
  *) echo "ERROR: mode must be off or on" >&2; exit 2 ;;
esac

STAMP="$(date +%Y%m%d_%H%M%S)"
RUN_DIR="$RESULT_ROOT/${MODE}_sigma_${SIGMA//./p}_trigger_${TRIGGER//./p}_$STAMP"
mkdir -p "$RUN_DIR/pcd"
INNOVATION_CSV="$RUN_DIR/lio_innovation.csv"
ROSCORE_LOG="$RUN_DIR/roscore.log"
MAPPING_LOG="$RUN_DIR/mapping.log"
PLAY_LOG="$RUN_DIR/rosbag_play.log"

set +u
source "$VERSION_ROOT/ros_env.sh"
set -u

ROSCORE_PID=""
LAUNCH_PID=""
cleanup() {
  set +e
  if rosnode list 2>/dev/null | grep -qx /laserMapping; then
    rosnode kill /laserMapping >/dev/null 2>&1
    sleep 2
  fi
  if [ -n "$LAUNCH_PID" ]; then kill -INT "$LAUNCH_PID" >/dev/null 2>&1; wait "$LAUNCH_PID" >/dev/null 2>&1; fi
  if [ -n "$ROSCORE_PID" ]; then kill -INT "$ROSCORE_PID" >/dev/null 2>&1; wait "$ROSCORE_PID" >/dev/null 2>&1; fi
}
trap cleanup EXIT INT TERM

if pgrep -af '[r]oscore|[r]osmaster|[f]astlivo_mapping|[r]osbag play' | grep -v "$$" | grep -q .; then
  echo "ERROR: another ROS/offline process is already running" >&2
  pgrep -af '[r]oscore|[r]osmaster|[f]astlivo_mapping|[r]osbag play' >&2 || true
  exit 3
fi

roscore >"$ROSCORE_LOG" 2>&1 &
ROSCORE_PID=$!
for _ in $(seq 1 40); do
  if rosparam list >/dev/null 2>&1; then break; fi
  sleep 0.25
done
rosparam set /use_sim_time true

roslaunch fast_livo mapping_uvc_mid360_offline.launch \
  pcd_output_dir:="$RUN_DIR/pcd/" \
  pcd_interval:=0 \
  ground_velocity_constraint_en:=false \
  ground_velocity_constraint_post_vio_en:=false \
  prediction_ground_displacement_constraint_en:="$ENABLED" \
  prediction_ground_displacement_sigma:="$SIGMA" \
  prediction_ground_displacement_trigger:="$TRIGGER" \
  prediction_ground_displacement_max_abs:="$MAX_ABS" \
  prediction_ground_displacement_max_correction:="$MAX_CORRECTION" \
  prediction_ground_displacement_correction_gain:="$CORRECTION_GAIN" \
  innovation_csv_path:="$INNOVATION_CSV" \
  >"$MAPPING_LOG" 2>&1 &
LAUNCH_PID=$!

for _ in $(seq 1 80); do
  if rosnode list 2>/dev/null | grep -qx /laserMapping; then break; fi
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "ERROR: mapping launch exited early" >&2
    tail -n 120 "$MAPPING_LOG" >&2
    exit 4
  fi
  sleep 0.25
done

PLAY_ARGS=("$BAG" --clock -r 1.0 --topics /livox/lidar /livox/imu /uvc_camera/image_sync)
if [ "$DURATION" != "0" ]; then PLAY_ARGS+=(--duration="$DURATION"); fi
rosbag play "${PLAY_ARGS[@]}" >"$PLAY_LOG" 2>&1
sleep 4

echo "MODE=$MODE"
echo "PREDICTION_GROUND_ENABLED=$ENABLED"
echo "SIGMA=$SIGMA"
echo "TRIGGER=$TRIGGER"
echo "MAX_ABS=$MAX_ABS"
echo "MAX_CORRECTION=$MAX_CORRECTION"
echo "CORRECTION_GAIN=$CORRECTION_GAIN"
echo "RUN_DIR=$RUN_DIR"
echo "INNOVATION_CSV=$INNOVATION_CSV"
if [ -s "$INNOVATION_CSV" ]; then
  echo "CSV_ROWS=$(($(wc -l < "$INNOVATION_CSV") - 1))"
else
  echo "ERROR: innovation CSV missing or empty" >&2
  exit 5
fi
