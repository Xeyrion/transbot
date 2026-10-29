#!/bin/bash
# 10-mapping-ssh: print (do not execute) mainboard-side mapping/navigation commands.
#
# Two machines are involved and they are NOT interchangeable:
#   control PC (this repo)  --publishes /cmd_vel, visualizes in rviz
#   robot mainboard (Jetson) --owns /dev/ttyTHS1, /dev/rplidar, /scan, /odom
#
# Mapping/navigation must run on the MAINBOARD, not here: it needs the serial
# devices and a lossless local /scan + /odom. The SSH user there is `jetson`
# (the control PC's own user is a different account -- do not mix them up).
#
# Usage: bash tasks/10-mapping-ssh.sh
DIR="$(cd "$(dirname "$0")" && pwd)"
source "$DIR/../env.sh" >/dev/null 2>&1 || true
# ROBOT_IP is the mainboard's address (the ROS master), not this PC's address.
ROBOT="${ROBOT_IP:-192.168.1.11}"

cat <<EOF
# --- SSH into the robot mainboard (user jetson, default password yahboom) ---
ssh jetson@${ROBOT}

# --- On the mainboard: choose one ---

# A. Mapping (drive around; view from the PC with tasks/03-view-mapping.sh)
roslaunch transbot_nav transbot_map.launch map_type:=gmapping open_rviz:=false
# Save the map when done:
rosrun map_server map_saver -f ~/my_map

# B. Navigation (uses an existing map, e.g. house)
roslaunch transbot_nav transbot_navigation.launch map:=house open_rviz:=false

# --- Back on the control PC: visualization only ---
# bash tasks/03-view-mapping.sh
EOF
