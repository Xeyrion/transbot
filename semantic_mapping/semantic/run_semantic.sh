#!/usr/bin/env bash
# One isolated offline run. Optional second argument must be a new directory.
set -eo pipefail
set +u
source "$(dirname "${BASH_SOURCE[0]}")/../../runtime/ros_env.sh"
set -u
export PYTHONIOENCODING=utf-8
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec python3 "$HERE/run_pipeline.py" "$@"
