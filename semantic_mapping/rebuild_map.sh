#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/../runtime/ros_env.sh"
cd "$(dirname "$0")"
exec python3 mapping_job.py "$@"
