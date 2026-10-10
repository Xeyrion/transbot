#!/bin/bash
# 06-line-follow: place the robot on a black line with the camera facing down.
# Prereq: camera view OK (run 04-view-image.sh first).
# Requires the vendor package transbot_linefollow on the ROS path.
DIR="$(cd "$(dirname "$0")" && pwd)"
source "$DIR/../env.sh" || exit 1
exec roslaunch transbot_linefollow follow_line.launch
