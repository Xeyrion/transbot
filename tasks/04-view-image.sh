#!/bin/bash
# 04-view-image: quick camera check, USB camera or Astra RGB.
# Usage: bash tasks/04-view-image.sh [usb|astra]   (default: astra)
DIR="$(cd "$(dirname "$0")" && pwd)"
source "$DIR/../env.sh" || exit 1
MODE="${1:-astra}"
if [ "$MODE" = "usb" ]; then
  echo "[view] /usb_cam/image_raw"
  exec rqt_image_view /usb_cam/image_raw
else
  echo "[view] /camera/rgb/image_raw/compressed (+ depth: /camera/depth/image_raw)"
  echo "Depth pseudo-color is also available in the GUI. Pass 'usb' for the USB camera."
  exec rqt_image_view /camera/rgb/image_raw/compressed
fi
