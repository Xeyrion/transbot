#!/bin/bash
# ===========================================================================
# start_slam_nav.sh -- map and navigate at the same time (self-contained).
#
# Everything this feature needs lives in this folder:
#   slam_nav.launch   gmapping + move_base on the live /map
#   slam_nav_gui.py   live map + click to send a goal
#
# It does NOT touch map_gui (mapping only) or task_gui (the wall task).
# It does stop a running nav stack first: map_server and amcl would publish a
# second /map and a second map->odom, which breaks both.
#
#   ./start_slam_nav.sh              # light
#   ./start_slam_nav.sh --dark       # rviz-like
#
# Left click = goal (0.30 m clearance required), right click = cancel.
# Quitting the window stops gmapping, move_base and the relay.
# ===========================================================================
set +u

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPTS="$HERE/../nav/scripts"

step() { printf '\n=== %s ===\n' "$*"; }
ok()   { printf '  ok    %s\n' "$*"; }
warn() { printf '  warn  %s\n' "$*"; }

step "1/6 ROS environment"
if [ -f "$HERE/../../console/env.sh" ]; then
    source "$HERE/../../console/env.sh" >/dev/null 2>&1
    ok "master $ROS_MASTER_URI, host $ROS_IP"
else
    source "$HERE/../../runtime/ros_env.sh"
    warn "console/env.sh not found"
fi

step "2/6 stop what would conflict"
# map_server / amcl publish a second /map and map->odom
pkill -f '[m]ap_server' 2>/dev/null
pkill -f '[a]mcl' 2>/dev/null
pkill -f '[m]ove_base' 2>/dev/null
pkill -f '[s]lam_nav.launch' 2>/dev/null
pkill -f '[s]lam_test.launch' 2>/dev/null
pkill -f '[s]can_filter.py' 2>/dev/null
pkill -f '[c]md_vel_scale.py' 2>/dev/null
sleep 2
ok "clean"

step "3/6 scan filter (shared with the mapping tool)"
setsid nohup python3 "$SCRIPTS/scan_filter.py" \
    > /tmp/scan_filter.log 2>&1 < /dev/null &
sleep 3
if timeout 5 rostopic list 2>/dev/null | grep -qx '/scan_filtered'; then
    ok "/scan_filtered is up"
else
    warn "/scan_filtered missing, see /tmp/scan_filter.log"
fi

step "4/6 gmapping + move_base (live map)"
: > /tmp/slam_nav.log
setsid nohup roslaunch "$HERE/slam_nav.launch" \
    > /tmp/slam_nav.log 2>&1 < /dev/null &
for _ in $(seq 1 25); do
    sleep 1
    if timeout 3 rosnode list 2>/dev/null | grep -qx '/move_base'; then
        break
    fi
done
if timeout 3 rosnode list 2>/dev/null | grep -qx '/move_base'; then
    ok "move_base is up"
else
    warn "move_base did not start, see /tmp/slam_nav.log"
fi

step "5/6 angular gain relay"
setsid nohup python3 "$SCRIPTS/cmd_vel_scale.py" \
    --in-topic /cmd_vel_raw --out-topic /cmd_vel \
    --angular-scale 0.709 --angular-deadzone 0.0 \
    > /tmp/cmd_vel_scale.log 2>&1 < /dev/null &
sleep 2
if pgrep -f '[c]md_vel_scale.py' > /dev/null; then
    ok "running (angular x0.709)"
else
    warn "relay did not start -- the robot will not move"
fi

step "6/6 window"
python3 "$HERE/slam_nav_gui.py" "$@"

step "cleanup"
pkill -f '[s]lam_nav.launch' 2>/dev/null
pkill -f '[s]can_filter.py' 2>/dev/null
pkill -f '[c]md_vel_scale.py' 2>/dev/null
ok "stopped"
