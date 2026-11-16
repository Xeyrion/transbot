#!/usr/bin/env bash
# Record a reproducible session bag for the Transbot research work.
#
# Why: every analysis on 2026-11-03 was an ad-hoc script plus a text log, so
# nothing could be re-examined without moving the robot again. One bag per
# session lets you re-run ICP, re-fit the odometry scale, or rebuild a map
# offline. Given the battery sits at 11.2 V and every motion test needs a
# spotter, this is the cheapest process change available.
#
# Usage
#   ./record_session.sh [tag] [duration_seconds]
#   ./record_session.sh clean_map 240
#
# Run this BEFORE the motion script, in a second terminal. Ctrl-C stops early.
# Works on the control machine or on the robot; on the robot it writes to the
# same path but eats the Nano's SD card, so prefer the control machine.
set -euo pipefail

TAG="${1:-session}"
DURATION="${2:-300}"
OUT_DIR="${BAG_DIR:-$HOME/robot_research/bagtools/bags}"

if ! command -v rosbag >/dev/null 2>&1; then
    echo "ERROR: rosbag not found. Source ROS first:" >&2
    echo "  source /opt/ros/melodic/setup.bash" >&2
    exit 1
fi

if [ -z "${ROS_MASTER_URI:-}" ]; then
    echo "WARNING: ROS_MASTER_URI is empty; rosbag will talk to localhost:11311." >&2
    echo "  For the robot over WiFi: export ROS_MASTER_URI=http://192.168.1.11:11311" >&2
fi

mkdir -p "$OUT_DIR"

AVAIL_KB=$(df -Pk "$OUT_DIR" | awk 'NR==2 {print $4}')
if [ -n "$AVAIL_KB" ] && [ "$AVAIL_KB" -lt 2097152 ]; then
    echo "ERROR: less than 2 GB free in $OUT_DIR (${AVAIL_KB} KB)." >&2
    exit 1
fi

STAMP=$(date +%Y%m%d_%H%M%S)
BAG="$OUT_DIR/${TAG}_${STAMP}.bag"

# /map and /amcl_pose are optional: rosbag warns and skips absent topics, which
# is fine for a mapping run where map_server is not up yet.
TOPICS=(
    /scan
    /odom
    /odom_raw
    /tf
    /tf_static
    /imu/data
    /imu/data_raw
    /cmd_vel
    /map
    /amcl_pose
)

echo "Recording $DURATION s -> $BAG"
echo "Topics: ${TOPICS[*]}"
echo

set +e
rosbag record -O "$BAG" --duration="$DURATION" "${TOPICS[@]}"
RC=$?
set -e

echo
if [ -f "$BAG" ]; then
    echo "=== bag written ==="
    rosbag info "$BAG" || true
    echo
    echo "Re-analyse offline:"
    echo "  python3 scripts/bag_replay.py $BAG"
else
    echo "WARNING: no bag file at $BAG (rosbag exit $RC)" >&2
fi
