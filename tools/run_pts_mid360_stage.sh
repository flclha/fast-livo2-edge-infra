#!/usr/bin/env bash
set -euo pipefail

if [ "$#" -lt 1 ] || [ "$#" -gt 2 ]; then
  echo "usage: $0 calibration|static|motion [duration_seconds]" >&2
  exit 2
fi

STAGE="$1"
case "$STAGE" in
  calibration) DEFAULT_DURATION=600 ;;
  static) DEFAULT_DURATION=120 ;;
  motion) DEFAULT_DURATION=180 ;;
  *) echo "unknown stage: $STAGE" >&2; exit 2 ;;
esac
DURATION="${2:-$DEFAULT_DURATION}"
if ! [[ "$DURATION" =~ ^[0-9]+$ ]] || [ "$DURATION" -lt 1 ]; then
  echo "duration must be a positive integer" >&2
  exit 2
fi

VERSION_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONFIG="$VERSION_ROOT/catkin_ws/src/mvs_ros_driver/config/uvc_camera.yaml"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="$VERSION_ROOT/logs/pts_mid360/${STAMP}_${STAGE}"
CSV="$OUT/frames.csv"
mkdir -p "$OUT"

if [ "$STAGE" = calibration ]; then
  grep -q '^TimestampSource:[[:space:]]*"v4l2_soe_lidar_clock"' "$CONFIG" || {
    echo "calibration requires TimestampSource=v4l2_soe_lidar_clock" >&2
    echo "restore acquisition mode before fitting a replacement calibration" >&2
    exit 3
  }
else
  grep -q '^TimestampSource:[[:space:]]*"pts_mid360_session_anchor"' "$CONFIG" || {
    echo "$STAGE validation requires automatic PTS session anchoring" >&2
    exit 3
  }
  grep -q '^CameraClockCalibrationValid:[[:space:]]*1' "$CONFIG" || {
    echo "CameraClockCalibrationValid must be 1" >&2
    exit 3
  }
fi

cp "$CONFIG" "$OUT/config_used.yaml"
stopped=0
stop_stack() {
  if [ "$stopped" -eq 0 ]; then
    bash "$VERSION_ROOT/stop_slam" || true
    stopped=1
  fi
}
trap stop_stack EXIT INT TERM

echo "Starting $STAGE acquisition for ${DURATION}s"
CAMERA_TIME_DIAG_CSV="$CSV" START_RVIZ="${START_RVIZ:-0}" \
  bash "$VERSION_ROOT/start_slam1" | tee "$OUT/startup.log"
sleep "$DURATION"
stop_stack
trap - EXIT INT TERM

cp "$VERSION_ROOT/logs/camera_driver.log" "$OUT/camera_driver.log" 2>/dev/null || true
cp "$VERSION_ROOT/logs/fast_livo_mapping.log" "$OUT/fast_livo_mapping.log" 2>/dev/null || true
test -s "$CSV" || { echo "missing camera diagnostic CSV: $CSV" >&2; exit 4; }

if [ "$STAGE" = calibration ]; then
  python3 "$VERSION_ROOT/tools/fit_pts_mid360_clock.py" "$CSV" \
    --mode fit --label calibration \
    --output-json "$OUT/calibration.json" \
    --output-pairs "$OUT/paired_samples.csv" \
    --write-config "$CONFIG" --activate \
    | tee "$OUT/calibration.txt"
  echo "Fixed a written to $CONFIG; every future STREAMON re-anchors b_session."
else
  python3 "$VERSION_ROOT/tools/fit_pts_mid360_clock.py" "$CSV" \
    --mode validate --use-recorded-session-map --label "$STAGE" \
    --fast-livo-log "$OUT/fast_livo_mapping.log" \
    --output-json "$OUT/${STAGE}.json" \
    --output-pairs "$OUT/paired_samples.csv" \
    | tee "$OUT/${STAGE}.txt"
fi

echo "Stage output: $OUT"
