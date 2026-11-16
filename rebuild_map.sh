#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/melodic/setup.bash
cd "$(dirname "$0")"
exec python3 rebuild_map.py "$@"
