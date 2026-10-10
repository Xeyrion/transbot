#!/bin/bash
# 01-keyboard-drive: teleop drive check (needs 00-health-check voltage/odom OK).
# Keys: i forward, , back, j/l turn, k stop, q/z speed +/-  (teleop_twist_keyboard)
DIR="$(cd "$(dirname "$0")" && pwd)"
source "$DIR/../env.sh" || exit 1
exec rosrun teleop_twist_keyboard teleop_twist_keyboard.py
