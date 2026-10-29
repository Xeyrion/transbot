#!/bin/bash
# 01-keyboard-drive: teleop drive check (needs 00-health-check voltage/odom OK).
# This launches the vendor teleop_twist_keyboard (full keymap):
#   u/i/o  forward-left/forward/forward-right     j/k/l  left/stop/right
#   m/,/.  back-left/back/back-right              q/z speed +/- 10% (w/x linear, e/c angular)
# NOTE: scripts/arm_teleop.py uses a simplified subset (i/,/j/l/k, u/o speed).
DIR="$(cd "$(dirname "$0")" && pwd)"
source "$DIR/../env.sh" || exit 1
exec rosrun teleop_twist_keyboard teleop_twist_keyboard.py
