#!/bin/bash
# ======================================================================
# run_gui.sh — launcher for the Transbot GUI console (tkinter).
#
# Behavior:
#   1. Sources ./env.sh (same directory first) to configure the ROS
#      endpoint; falls back to ../env.sh, then plain ROS Melodic setup
#      when env.sh is absent.
#   2. Verifies the Python dependencies (rospy / transbot_msgs) and
#      prints an actionable message when something is missing.
#   3. Probes the ROS master; warns (non-fatal) if unreachable. The GUI
#      still opens for offline inspection; motion/effect commands require
#      the robot online.
#   4. Replaces itself with the GUI process (exec).
#
# Usage: bash run_gui.sh   (or ./run_gui.sh)
# Requires: python3-tk, ROS Melodic.
#           transbot_msgs ships in vendor/python (no build required).
# ======================================================================
cd "$(dirname "$0")"
DIR=$(pwd)

# 1) ROS endpoint configuration (relative paths first; no fixed location assumed).
if [ -f "$DIR/env.sh" ]; then
    source "$DIR/env.sh"
elif [ -f "$DIR/../env.sh" ]; then
    source "$DIR/../env.sh"
elif [ -f /opt/ros/melodic/setup.bash ]; then
    source /opt/ros/melodic/setup.bash
fi

# 2) Dependency preflight — fail early with a precise diagnosis instead
#    of a bare "ModuleNotFoundError" traceback.
if ! python3 -c "import rospy" >/dev/null 2>&1; then
    echo "Error: the ROS Melodic Python modules are not importable."
    echo "  -> 'rospy' not found. Install ROS Melodic and/or source its setup.bash:"
    echo "       source /opt/ros/melodic/setup.bash"
    exit 1
fi
if ! python3 -c "import transbot_msgs" >/dev/null 2>&1; then
    echo "Error: the 'transbot_msgs' package is not importable."
    echo "  -> Expected one of:"
    echo "       a) a built catkin workspace on PYTHONPATH (transbot_ws/devel)"
    echo "       b) the bundled copy in $DIR/vendor/python"
    echo "  -> Re-sourcing env.sh should fix this:  source $DIR/env.sh"
    exit 1
fi

# 3) Master reachability check (advisory only; exit status ignored by design).
if ! timeout 3 bash -c "echo > /dev/tcp/${ROBOT_IP:-192.168.1.11}/11311" 2>/dev/null; then
    echo "Warning: ROS master unreachable at ${ROBOT_IP:-192.168.1.11}."
    echo "GUI will open; motion and effect commands take effect after the robot is online."
fi

echo "Starting Transbot console..."
exec python3 "$DIR/transbot_gui.py"
