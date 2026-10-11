#!/bin/sh
set -eu

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
cd "$SCRIPT_DIR"

DEBUG_RECORD=${CYBERCAM_DEBUG_RECORD:-off}
DEBUG_RECORD_DIR=${CYBERCAM_DEBUG_RECORD_DIR:-"$SCRIPT_DIR/debug_frames"}
DEBUG_RECORD_INTERVAL=${CYBERCAM_DEBUG_RECORD_INTERVAL:-1.0}
SERIAL_PORT=${CYBERCAM_SERIAL_PORT:-/dev/ttyS2}
TARGET=${CYBERCAM_TARGET:-ring_cross}
CAMERA_BACKEND=${CYBERCAM_BACKEND:-opencv}
CAMERA_SOURCE=${CYBERCAM_CAMERA:-/dev/video1}
CAMERA_WIDTH=${CYBERCAM_WIDTH:-640}
CAMERA_HEIGHT=${CYBERCAM_HEIGHT:-480}
CAMERA_FPS=${CYBERCAM_FPS:-30}

if [ "${CYBERCAM_MANAGE_DESKTOP_SERVICE:-on}" = "on" ]; then
  sudo systemctl stop cybercam-desktop.service
fi

if [ "$CAMERA_BACKEND" = "opencv" ] && command -v v4l2-ctl >/dev/null 2>&1; then
  v4l2-ctl -d "$CAMERA_SOURCE" \
    --set-fmt-video="width=$CAMERA_WIDTH,height=$CAMERA_HEIGHT,pixelformat=MJPG"
  v4l2-ctl -d "$CAMERA_SOURCE" --set-parm="$CAMERA_FPS"
fi

exec python3 -u main.py \
  --backend "$CAMERA_BACKEND" \
  --camera "$CAMERA_SOURCE" \
  --width "$CAMERA_WIDTH" \
  --height "$CAMERA_HEIGHT" \
  --display off \
  --serial "$SERIAL_PORT" \
  --target "$TARGET" \
  --debug-record "$DEBUG_RECORD" \
  --debug-record-dir "$DEBUG_RECORD_DIR" \
  --debug-record-interval "$DEBUG_RECORD_INTERVAL" \
  "$@"
