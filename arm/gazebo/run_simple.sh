#!/usr/bin/env bash
# Start a FRESH simplest Gazebo scene (one 2R arm, nothing else) plus the panel.
# If a Gazebo scene is already running it is stopped first; a new one always
# starts.  Two scenes cannot share the local ROS master, so there is no reuse.
#
#   ./gazebo/run_simple.sh              # fresh Gazebo + panel
#   ./gazebo/run_simple.sh gui:=false   # headless Gazebo + panel
#   ./gazebo/run_simple.sh init_pose:=false
#   ./gazebo/run_simple.sh -- --j7 95 --j8 180   # panel initial pose
#   ./gazebo/run_simple.sh -- --lang en          # English panel
#
# Arguments before "--" go to roslaunch, after "--" go to arm_simple_gui.py.
# Ctrl-C stops the Gazebo scene and the panel started by this script.
set -e

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"

LAUNCH_ARGS=()
GUI_ARGS=()
after_sep=0
for a in "$@"; do
  if [ "$a" = "--" ]; then
    after_sep=1
    continue
  fi
  if [ "$after_sep" -eq 1 ]; then
    GUI_ARGS+=("$a")
  else
    LAUNCH_ARGS+=("$a")
  fi
done

source /opt/ros/melodic/setup.bash
if [ -f "$HOME/transbot_ws/devel/setup.bash" ]; then
  source "$HOME/transbot_ws/devel/setup.bash"
fi
export ROS_PACKAGE_PATH="$ROOT/gazebo:$ROS_PACKAGE_PATH"

# Keep the simulation on a LOCAL master.  If the real robot is online,
# ~/.bashrc / transbot/console/env.sh point ROS_MASTER_URI at the robot; running
# Gazebo there would flip /use_sim_time on the robot and merge its topics.
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
export ROS_HOSTNAME=127.0.0.1
export GAZEBO_MODEL_DATABASE_URI=
echo "[simple] local ROS master: $ROS_MASTER_URI (robot master is not touched)"

# ------------------------------------------------- stop any existing scene
# One scene at a time: stop whatever is running, then start fresh.
"$HERE/stop_simple.sh" || exit 1

# --------------------------------------------------------- start new scene
LOG_DIR="${TMPDIR:-/tmp}/transbot_simple"
mkdir -p "$LOG_DIR"
setsid nohup roslaunch transbot_arm_simple arm_simple.launch "${LAUNCH_ARGS[@]}" \
  > "$LOG_DIR/gazebo.log" 2>&1 < /dev/null &
disown || true
echo "[simple] 场景已 detached：关面板/关终端不会停场景；"
echo "[simple] 重跑本脚本起全新场景，或 ./gazebo/stop_simple.sh 彻底停止"

controllers_ok() {
  local out
  out="$(timeout 4 rosservice call /controller_manager/list_controllers 2>/dev/null)" || return 1
  printf '%s\n' "$out" | grep -A1 'name: "joint_state_controller"' \
    | grep -q 'state: "running"' || return 1
  printf '%s\n' "$out" | grep -A1 'name: "arm_controller"' \
    | grep -q 'state: "running"' || return 1
}

# Wait for the arm controller and live joint states before opening the panel.
ready=0
for _ in $(seq 1 40); do
  if controllers_ok && timeout 2 rostopic echo -n1 /joint_states >/dev/null 2>&1; then
    ready=1
    break
  fi
  sleep 0.5
done
if [ "$ready" != 1 ]; then
  echo "[simple] 新场景没有起来：/arm_controller 或 /joint_states 不可用" >&2
  exit 1
fi
sleep 1

rosrun transbot_arm_simple arm_simple_gui.py "${GUI_ARGS[@]}"
