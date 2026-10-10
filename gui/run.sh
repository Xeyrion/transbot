#!/bin/bash
# ============================================================
# transbot-remote-console / gui launcher
#   1. Sources ../env.sh (ROS_MASTER_URI / ROS_IP)
#   2. Starts the tkinter console (gui/app.py)
#
# Usage: bash gui/run.sh   (or ./gui/run.sh)
# ============================================================
cd "$(dirname "$0")"
DIR=$(pwd)

# 1) ROS environment: repo-local env.sh first, system ROS as offline fallback.
if [ -f "$DIR/../env.sh" ]; then
    source "$DIR/../env.sh"
elif [ -f /opt/ros/melodic/setup.bash ]; then
    source /opt/ros/melodic/setup.bash
    [ -f ~/transbot_ws/devel/setup.bash ] && source ~/transbot_ws/devel/setup.bash --extend
fi

# 2) Warn if the robot master is unreachable; the GUI still opens for debugging.
if ! timeout 3 bash -c "echo > /dev/tcp/${ROBOT_IP:-192.168.1.11}/11311" 2>/dev/null; then
    echo "[hint] robot ROS master unreachable (${ROBOT_IP:-192.168.1.11})."
    echo "       The GUI will still open, but motion/light actions need the robot online."
fi

echo "[start] launching Transbot console..."
exec python3 "$DIR/app.py"
