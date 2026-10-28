#!/usr/bin/env bash
set -euo pipefail

MODE="${1:?usage: run_offline_vio_velocity_delta_ab.sh off|on BAG LABEL [trigger] [limit] [result_root]}"
BAG="${2:?missing input bag}"
LABEL="${3:?missing run label}"
TRIGGER="${4:-0.040}"
LIMIT="${5:-$TRIGGER}"
VERSION_ROOT="/home/nuc/versions/version5"
RESULT_ROOT="${6:-$VERSION_ROOT/offline_results/vio_velocity_delta_guard_20261009}"
STAGE="${7:-vio}"
PLAY_RATE="${8:-1.0}"

case "$MODE" in
  off) ENABLED=false ;;
  on) ENABLED=true ;;
  *) echo "ERROR: mode must be off or on" >&2; exit 2 ;;
esac

case "$STAGE" in
  vio)
    VIO_DIAG=true; VIO_GUARD="$ENABLED"
    PREDICTION_START_DIAG=false; PREDICTION_START_GUARD=false
    ;;
  prediction_start)
    VIO_DIAG=false; VIO_GUARD=false
    PREDICTION_START_DIAG=true; PREDICTION_START_GUARD="$ENABLED"
    ;;
  *) echo "ERROR: stage must be vio or prediction_start" >&2; exit 2 ;;
esac

if [ ! -r "$BAG" ]; then
  echo "ERROR: bag not readable: $BAG" >&2
  exit 2
fi

STAMP="$(date +%Y%m%d_%H%M%S)"
RUN_DIR="$RESULT_ROOT/${LABEL}_${STAGE}_${MODE}_trigger_${TRIGGER//./p}_limit_${LIMIT//./p}_$STAMP"
mkdir -p "$RUN_DIR/pcd"
INNOVATION_CSV="$RUN_DIR/lio_innovation.csv"
OUTPUT_PREFIX="$RUN_DIR/derived"
OUTPUT_BAG="$OUTPUT_PREFIX.bag"
ROSCORE_LOG="$RUN_DIR/roscore.log"
MAPPING_LOG="$RUN_DIR/mapping.log"
RECORD_LOG="$RUN_DIR/rosbag_record.log"
PLAY_LOG="$RUN_DIR/rosbag_play.log"
META="$RUN_DIR/run_meta.txt"

set +u
source "$VERSION_ROOT/ros_env.sh"
set -u

ROSCORE_PID=""
LAUNCH_PID=""
RECORD_PID=""
cleanup() {
  set +e
  if [ -n "$RECORD_PID" ] && kill -0 "$RECORD_PID" 2>/dev/null; then
    kill -INT "$RECORD_PID" >/dev/null 2>&1
    wait "$RECORD_PID" >/dev/null 2>&1
  fi
  if rosnode list 2>/dev/null | grep -qx /laserMapping; then
    rosnode kill /laserMapping >/dev/null 2>&1
    sleep 1
  fi
  if [ -n "$LAUNCH_PID" ] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
    kill -INT "$LAUNCH_PID" >/dev/null 2>&1
    wait "$LAUNCH_PID" >/dev/null 2>&1
  fi
  if [ -n "$ROSCORE_PID" ] && kill -0 "$ROSCORE_PID" 2>/dev/null; then
    kill -INT "$ROSCORE_PID" >/dev/null 2>&1
    wait "$ROSCORE_PID" >/dev/null 2>&1
  fi
}
trap cleanup EXIT INT TERM

if pgrep -af '[r]oscore|[r]osmaster|[f]astlivo_mapping|[r]osbag play|[r]osbag record' | grep -v "$$" | grep -q .; then
  echo "ERROR: another ROS/offline process is already running" >&2
  pgrep -af '[r]oscore|[r]osmaster|[f]astlivo_mapping|[r]osbag play|[r]osbag record' >&2 || true
  exit 3
fi

BAG_START="$(rosbag info -y "$BAG" | awk '/^start:/{print $2; exit}')"
BAG_DURATION="$(rosbag info -y "$BAG" | awk '/^duration:/{print $2; exit}')"

{
  echo "mode=$MODE"
  echo "stage=$STAGE"
  echo "enabled=$ENABLED"
  echo "bag=$BAG"
  echo "label=$LABEL"
  echo "bag_start=$BAG_START"
  echo "bag_duration=$BAG_DURATION"
  echo "trigger=$TRIGGER"
  echo "limit=$LIMIT"
  echo "stable_frames=3"
  echo "play_rate=$PLAY_RATE"
  echo "created_at=$(date --iso-8601=seconds)"
} >"$META"

roscore >"$ROSCORE_LOG" 2>&1 &
ROSCORE_PID=$!
for _ in $(seq 1 60); do
  if rosparam list >/dev/null 2>&1; then break; fi
  sleep 0.25
done
rosparam set /use_sim_time true

roslaunch fast_livo mapping_uvc_mid360_offline.launch \
  pcd_output_dir:="$RUN_DIR/pcd/" \
  pcd_interval:=0 \
  ground_velocity_constraint_en:=false \
  ground_velocity_constraint_post_vio_en:=false \
  prediction_ground_displacement_constraint_en:=false \
  vio_velocity_delta_diag_en:="$VIO_DIAG" \
  vio_velocity_delta_guard_en:="$VIO_GUARD" \
  vio_velocity_delta_guard_trigger:="$TRIGGER" \
  vio_velocity_delta_guard_limit:="$LIMIT" \
  vio_velocity_delta_guard_min_stable_frames:=3 \
  prediction_start_velocity_delta_diag_en:="$PREDICTION_START_DIAG" \
  prediction_start_velocity_delta_guard_en:="$PREDICTION_START_GUARD" \
  prediction_start_velocity_delta_guard_trigger:="$TRIGGER" \
  prediction_start_velocity_delta_guard_limit:="$LIMIT" \
  prediction_start_velocity_delta_guard_min_stable_frames:=3 \
  innovation_csv_path:="$INNOVATION_CSV" \
  >"$MAPPING_LOG" 2>&1 &
LAUNCH_PID=$!

for _ in $(seq 1 100); do
  if rosnode list 2>/dev/null | grep -qx /laserMapping; then break; fi
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "ERROR: mapping launch exited early" >&2
    tail -n 120 "$MAPPING_LOG" >&2
    exit 4
  fi
  sleep 0.25
done

rosbag record -O "$OUTPUT_PREFIX" \
  /aft_mapped_to_init \
  /LIVO2/lio_prediction \
  /LIVO2/lio_update \
  /LIVO2/lio_innovation \
  >"$RECORD_LOG" 2>&1 &
RECORD_PID=$!
sleep 2

rosbag play "$BAG" --clock -r "$PLAY_RATE" --topics \
  /livox/lidar /livox/imu /uvc_camera/image_sync \
  >"$PLAY_LOG" 2>&1
sleep 4

kill -INT "$RECORD_PID" >/dev/null 2>&1 || true
wait "$RECORD_PID" >/dev/null 2>&1 || true
RECORD_PID=""

if [ ! -s "$INNOVATION_CSV" ]; then
  echo "ERROR: innovation CSV missing or empty" >&2
  exit 5
fi
if [ ! -s "$OUTPUT_BAG" ]; then
  echo "ERROR: derived output bag missing or empty" >&2
  exit 6
fi

echo "MODE=$MODE"
echo "STAGE=$STAGE"
echo "PLAY_RATE=$PLAY_RATE"
echo "BAG=$BAG"
echo "RUN_DIR=$RUN_DIR"
echo "INNOVATION_CSV=$INNOVATION_CSV"
echo "OUTPUT_BAG=$OUTPUT_BAG"
echo "CSV_ROWS=$(($(wc -l < "$INNOVATION_CSV") - 1))"
