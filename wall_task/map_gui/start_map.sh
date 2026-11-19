#!/bin/bash
# ===========================================================================
# start_map.sh -- one command for mapping: scan filter + gmapping + map_gui.
#
# The robot only publishes /scan and the TF tree; gmapping runs here on the
# control machine so the parameters are easy to change and nothing on the
# robot has to be touched.
#
#   ./start_map.sh                     # maps go to wall_task/maps
#   MAP_DIR=~/maps ./start_map.sh      # somewhere else
#   ./start_map.sh --dark              # rviz-like dark palette
#
# Quitting the window also stops gmapping.  If you want to keep the map alive
# (e.g. to save it from another terminal), kill only the GUI.
# ===========================================================================
set +u

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPTS="$HERE/../nav/scripts"
MAP_DIR="${MAP_DIR:-$HERE/../maps}"

step() { printf '\n=== %s ===\n' "$*"; }
ok()   { printf '  ok    %s\n' "$*"; }
warn() { printf '  warn  %s\n' "$*"; }

step "1/5 ROS environment"
if [ -f "$HERE/../../console/env.sh" ]; then
    source "$HERE/../../console/env.sh" >/dev/null 2>&1
    ok "sourced console/env.sh  (master $ROS_MASTER_URI, host $ROS_IP)"
else
    source "$HERE/../../runtime/ros_env.sh"
    warn "console/env.sh not found; using bare ROS"
fi
if ! timeout 3 bash -c "echo > /dev/tcp/$(echo "$ROS_MASTER_URI" | sed -E 's#.*://([^:/]+).*#\1#')/11311" 2>/dev/null; then
    echo "  FAIL  no ROS master at $ROS_MASTER_URI -- is the robot on?"
    exit 1
fi
ok "master reachable"

step "2/5 stop leftovers"
pkill -f '[s]can_filter.py' 2>/dev/null
pkill -f '[s]lam_test.launch' 2>/dev/null
pkill -f '[s]lam_gmapping' 2>/dev/null
sleep 2
ok "clean"

step "3/5 scan filter (drops the robot's own body from /scan)"
setsid nohup python3 "$SCRIPTS/scan_filter.py" \
    > /tmp/scan_filter.log 2>&1 < /dev/null &
sleep 3
HZ=$(timeout 6 rostopic hz /scan_filtered 2>&1 | grep -o 'average rate: [0-9.]*' | tail -1)
if [ -n "$HZ" ]; then
    ok "/scan_filtered $HZ"
else
    warn "/scan_filtered silent, see /tmp/scan_filter.log"
fi

step "4/5 gmapping"
: > /tmp/slam_test.log
setsid nohup roslaunch "$SCRIPTS/slam_test.launch" \
    > /tmp/slam_test.log 2>&1 < /dev/null &
for _ in $(seq 1 25); do
    sleep 1
    if timeout 3 rostopic list 2>/dev/null | grep -qx '/map'; then
        break
    fi
done
if timeout 3 rostopic list 2>/dev/null | grep -qx '/map'; then
    ok "/map is up"
else
    warn "/map did not appear, see /tmp/slam_test.log"
fi

step "5/5 window"
printf '  exec map_gui.py --maps-dir %s %s\n\n' "$MAP_DIR" "$*"
python3 "$HERE/map_gui.py" --maps-dir "$MAP_DIR" "$@"

step "cleanup"
pkill -f '[s]lam_test.launch' 2>/dev/null
pkill -f '[s]can_filter.py' 2>/dev/null
ok "gmapping stopped"
