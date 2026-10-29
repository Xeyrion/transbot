#!/bin/bash
# 08-demo-actions: one-shot showroom demo (buzzer -> RGB -> gimbal -> arm).
# The mobile base never moves. Robot must be online.
# Usage: bash tasks/08-demo-actions.sh
DIR="$(cd "$(dirname "$0")" && pwd)"
source "$DIR/../env.sh" || exit 1
exec python3 "$DIR/../scripts/demo_actions.py"
