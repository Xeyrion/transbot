#!/bin/bash
# 10-mapping-ssh: print (do not execute) robot-side mapping/navigation commands.
# Mapping must run on the robot: it needs stable local /scan + /odom.
# Usage: bash tasks/10-mapping-ssh.sh
DIR="$(cd "$(dirname "$0")" && pwd)"
source "$DIR/../env.sh" >/dev/null 2>&1 || true
ROBOT="${ROBOT_IP:-192.168.1.11}"

cat <<EOF
# --- SSH into the robot (default user jetson, password yahboom) ---
ssh yahboom@${ROBOT}

# --- On the robot: choose one ---

# A. Mapping (drive around; view from the PC with tasks/03-view-mapping.sh)
roslaunch transbot_nav transbot_map.launch map_type:=gmapping open_rviz:=false
# Save the map when done:
rosrun map_server map_saver -f ~/my_map

# B. Navigation (uses an existing map, e.g. house)
roslaunch transbot_nav transbot_navigation.launch map:=house open_rviz:=false

# --- On the PC: visualization only ---
# bash tasks/03-view-mapping.sh
EOF
