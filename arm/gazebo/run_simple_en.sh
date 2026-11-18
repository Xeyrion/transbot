#!/usr/bin/env bash
# English demo panel.  Same Gazebo scene and same behaviour as
# run_simple.sh; only the panel labels/messages are English.
#
#   ./gazebo/run_simple_en.sh
#   ./gazebo/run_simple_en.sh --real 192.168.1.11
#
# Extra arguments are passed to the GUI (after run_simple.sh's "--").
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
exec "$HERE/run_simple.sh" -- --lang en "$@"
