#!/bin/bash
# 00-health-check: run ~1 min after power-on. All green before continuing.
# Usage: bash tasks/00-health-check.sh
DIR="$(cd "$(dirname "$0")" && pwd)"
source "$DIR/../env.sh" || exit 1
bash "$DIR/../check_robot.sh"
