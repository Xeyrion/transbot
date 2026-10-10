#!/bin/bash
# 05-lidar-demo: avoidance / tracking / warning (needs /scan data, see 00-health-check).
# Usage: bash tasks/05-lidar-demo.sh [avoid|track|warn]   (default: avoid)
# Requires the vendor package transbot_laser on the ROS path (robot-side stack).
DIR="$(cd "$(dirname "$0")" && pwd)"
source "$DIR/../env.sh" || exit 1
MODE="${1:-avoid}"
case "$MODE" in
  avoid)  exec roslaunch transbot_laser laser_Avoidance.launch ;;
  track)  exec roslaunch transbot_laser laser_Tracker.launch ;;
  warn)   exec roslaunch transbot_laser laser_Warning.launch ;;
  *) echo "Usage: $0 [avoid|track|warn]"; exit 1 ;;
esac
