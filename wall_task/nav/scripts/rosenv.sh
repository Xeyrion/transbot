#!/usr/bin/env bash
# Source this before running anything ROS-related on the control machine.
#
#   source transbot/wall_task/nav/scripts/rosenv.sh
#
# Two things it fixes, both of which have silently broken tests before:
#   1. ROS_MASTER_URI left as 127.0.0.1 by a setup.bash somewhere.
#   2. ROS_IP inherited from an old shell. The WiFi dongle gets a DHCP address
#      that changes (137 -> 136 ...), and a stale ROS_IP makes the publisher
#      advertise an address the robot cannot dial back to. Service calls still
#      work, so the robot beeps and looks alive while no /cmd_vel ever arrives.
# The address is therefore always derived from the route to the master.

export ROS_MASTER_URI="http://${ROBOT_IP:-192.168.1.11}:11311"

_rosenv_ip=$(ip route get "${ROBOT_IP:-192.168.1.11}" 2>/dev/null \
    | awk '{for(i=1;i<=NF;i++) if($i=="src"){print $(i+1); exit}}')
if [ -z "$_rosenv_ip" ]; then
    echo "rosenv: no route to 192.168.1.11 -- is the robot hotspot up?" >&2
else
    if [ -n "${ROS_IP:-}" ] && [ "$ROS_IP" != "$_rosenv_ip" ]; then
        echo "rosenv: overriding stale ROS_IP=$ROS_IP with $_rosenv_ip" >&2
    fi
    export ROS_IP="$_rosenv_ip"
fi
unset _rosenv_ip

source "$(dirname "${BASH_SOURCE[0]}")/../../../runtime/ros_env.sh" || return $?
if [ -f "$HOME/transbot_ws/devel/setup.bash" ]; then
    source "$HOME/transbot_ws/devel/setup.bash"
fi
