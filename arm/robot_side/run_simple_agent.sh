#!/usr/bin/env bash
# Run the UDP agent against the REAL robot's ROS master, for the simple panel.
#
# The agent keeps one rospy connection to the robot master, so the panel can
# stay on the local Gazebo master and still command / read the real arm.
#
#   ./robot_side/run_simple_agent.sh              # enabled, box 0 (servo limits only)
#   ./robot_side/run_simple_agent.sh --read-only  # read angles only, never moves
#   ./robot_side/run_simple_agent.sh --box 30     # start pose +/- 30 deg envelope
#
# WARNING with the default box 0: the only protection is the servo hard
# limits (j7 0..225, j8 15..240).  The panel sends whatever the sliders say.
# Prefer --read-only first to check the state packets before enabling motion.
set -e

HERE="$(cd "$(dirname "$0")" && pwd)"

ROBOT_MASTER="${ROBOT_MASTER:-http://192.168.1.11:11311}"
# The robot must see us at an address it can reach back to.
# Prefer an explicit ROBOT_IP; otherwise auto-pick the local NIC on the robot subnet.
if [ -z "${ROBOT_IP:-}" ]; then
  ROBOT_HOST="$(printf '%s' "$ROBOT_MASTER" | sed -E 's#^https?://##; s#:.*$##')"
  ROBOT_SUB="$(printf '%s' "$ROBOT_HOST" | cut -d. -f1-3)"
  ROBOT_IP="$(ip -4 -o addr | awk '{print $4}' | cut -d/ -f1 | grep -E "^${ROBOT_SUB}\." | head -1)"
  if [ -z "$ROBOT_IP" ]; then
    ROBOT_IP="$(ip -4 -o route get 1.1.1.1 2>/dev/null | sed -nE 's/.*src ([0-9.]+).*/\1/p' | head -1)"
  fi
  unset ROBOT_HOST ROBOT_SUB
fi
ROBOT_IP="${ROBOT_IP:-127.0.0.1}"

source /opt/ros/melodic/setup.bash
if [ -f "$HOME/transbot_ws/devel/setup.bash" ]; then
  source "$HOME/transbot_ws/devel/setup.bash"
fi

export ROS_MASTER_URI="$ROBOT_MASTER"
export ROS_IP="$ROBOT_IP"
export ROS_HOSTNAME="$ROBOT_IP"

# Default envelope is box 0 = servo hard limits only, unless --box is given.
BOX_GIVEN=0
for a in "$@"; do
  if [ "$a" = "--box" ]; then BOX_GIVEN=1; break; fi
done
if [ "$BOX_GIVEN" = 0 ]; then
  set -- --box "${BOX:-0}" "$@"
fi

echo "[simple-agent] master=$ROS_MASTER_URI  ip=$ROS_IP  args: $*"
exec python2 "$HERE/arm_udp_agent.py" "$@"
