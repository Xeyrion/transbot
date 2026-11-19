#!/usr/bin/env bash
# Source exactly one ROS 1 installation without changing the master endpoint.
if [ -z "${TRANSBOT_ROS_SETUP:-}" ]; then
  if [ -n "${ROS_DISTRO:-}" ] && [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
    TRANSBOT_ROS_SETUP="/opt/ros/$ROS_DISTRO/setup.bash"
  elif [ -f /opt/ros/melodic/setup.bash ]; then
    TRANSBOT_ROS_SETUP=/opt/ros/melodic/setup.bash
  elif [ -f /opt/ros/noetic/setup.bash ]; then
    TRANSBOT_ROS_SETUP=/opt/ros/noetic/setup.bash
  else
    echo 'ROS 1 setup not found; set TRANSBOT_ROS_SETUP to its setup.bash.' >&2
    return 1
  fi
fi
if [ ! -f "$TRANSBOT_ROS_SETUP" ]; then
  echo "ROS setup is missing: $TRANSBOT_ROS_SETUP" >&2
  return 1
fi
export TRANSBOT_ROS_SETUP
_transbot_shell_flags=$-
set +eu
source "$TRANSBOT_ROS_SETUP"
_transbot_ros_result=$?
if [ "$_transbot_ros_result" -eq 0 ] && [ -n "${ROS_DISTRO:-}" ] &&
   [ -f "$HOME/.local/share/transbot/ros/$ROS_DISTRO/setup.bash" ]; then
  source "$HOME/.local/share/transbot/ros/$ROS_DISTRO/setup.bash"
  _transbot_ros_result=$?
fi
case "$_transbot_shell_flags" in *u*) set -u ;; esac
case "$_transbot_shell_flags" in *e*) set -e ;; esac
unset _transbot_shell_flags
return "$_transbot_ros_result"
