#!/bin/bash
# 07-arm-teleop: arm + gimbal + base keyboard teleop.
# Keys: q/z j7, w/x j8, e/c j9, r/v gimbal, s home; base keys printed on start.
# Usage: bash tasks/07-arm-teleop.sh
DIR="$(cd "$(dirname "$0")" && pwd)"
source "$DIR/../env.sh" || exit 1
exec python3 "$DIR/../scripts/arm_teleop.py"
