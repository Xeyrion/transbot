#!/bin/bash
# One command: clean up, localise, bring up the navigation stack, open rviz.
#
#   ./start_nav.sh                 localise from the current scan, then start
#   ./start_nav.sh --no-rviz       same without the GUI
#   ./start_nav.sh --pose 0 0 0    skip localisation, use this pose
#
# Why it localises first: after the robot has been picked up and put down,
# AMCL has no idea where it is (odometry only counts wheel rotation, and the
# wheels did not turn). coarse_localize matches one scan against the map and
# gives a pose good to a few centimetres, which is what AMCL needs to start.
#
# Once rviz is up:
#   1. click "2D Nav Goal" in the toolbar (or press G)
#   2. click a free spot on the map, drag to set the heading, release
#   3. the green line is the global plan, the blue line is what DWA is
#      actually doing; the yellow arrow is where AMCL thinks the robot is
#
# Clicking in rviz does NOT check the goal: a goal inside a wall or under the
# table is accepted and then fails as "Failed to get a plan". To check first,
# run:  nav_goal.py --check <x> <y>

set -o pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
# Which map to load: the first line of maps/current, else fig8.
# Switching maps means editing that one line -- no renaming, and no
# chance of breaking the "image:" path inside the yaml.
MAP_NAME=$(head -1 "$HERE/../../maps/current" 2>/dev/null | tr -d "[:space:]")
[ -z "$MAP_NAME" ] && MAP_NAME=fig8
MAP="$HERE/../../maps/$MAP_NAME.yaml"
ROBOT="${ROBOT_USER:-jetson}@${ROBOT_IP:-192.168.1.11}"
SSH_OPTS="-n -o StrictHostKeyChecking=no -o ConnectTimeout=8"

RVIZ=1
POSE=""
while [ $# -gt 0 ]; do
    case "$1" in
        --no-rviz) RVIZ=0; shift ;;
        --pose)    POSE="$2 $3 $4"; shift 4 ;;
        -h|--help) sed -n '2,22p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "unknown option: $1"; exit 2 ;;
    esac
done

export SSHPASS="${ROBOT_SSH_PASS:-yahboom}"
source "$HERE/rosenv.sh" || { echo "ABORT: ROS environment"; exit 1; }
cd "$HERE" || exit 1

step() { echo; echo "=== $* ==="; }

step "1/6 clock"
NOW=$(date '+%Y-%m-%d %H:%M:%S.%N')
sshpass -e ssh $SSH_OPTS "$ROBOT" "echo "$SSHPASS" | sudo -S date -s '$NOW'" > /dev/null 2>&1
sleep 1
C=$(date +%s.%N)
R=$(sshpass -e ssh $SSH_OPTS "$ROBOT" 'date +%s.%N' | tail -1)
python3 -c "print('  robot - control = %+.3f s' % (float('$R') - float('$C')))"

if [ ! -f "$MAP" ]; then
    echo "  ABORT: $MAP not found"
    echo "         maps/current says: $MAP_NAME"
    exit 1
fi

step "2/6 stop leftovers"
for n in /move_base /amcl /map_server; do
    timeout 6 rosnode kill "$n" > /dev/null 2>&1
done
# keepalive nodes are anonymous, so they cannot be killed by name
pkill -f '[a]mcl_keepalive' 2>/dev/null
pkill -f '[c]md_vel_scale.py' 2>/dev/null
pkill -f '[n]av_test.launch' 2>/dev/null
sleep 4
LEFT=$(timeout 8 rosnode list 2>/dev/null | grep -cE '/move_base$|/amcl$|/map_server$')
echo "  navigation nodes left: $LEFT"

step "3/6 localise"
if [ -n "$POSE" ]; then
    echo "  using the pose from the command line: $POSE"
else
    POSE=$(timeout 200 python3 coarse_localize.py --map "$MAP" --pose-only 2>/dev/null | tail -1)
    if [ -z "$POSE" ]; then
        echo "  ABORT: coarse_localize produced nothing (is /scan alive?)"
        exit 1
    fi
    echo "  coarse pose: $POSE"
fi
# shellcheck disable=SC2086
set -- $POSE
PX=$1; PY=$2; PA=$3
python3 - "$PX" "$PY" "$PA" <<'PY'
import sys
print('  x=%.2f  y=%.2f  yaw=%.1f deg' % (float(sys.argv[1]),
                                          float(sys.argv[2]),
                                          float(sys.argv[3]) * 57.2958))
PY

step "4/6 navigation stack"
: > /tmp/nav_test.log
setsid nohup roslaunch nav_test.launch map_file:=$MAP \
    initial_pose_x:=$PX initial_pose_y:=$PY initial_pose_a:=$PA \
    > /tmp/nav_test.log 2>&1 < /dev/null &
sleep 18
NODES=$(timeout 8 rosnode list 2>/dev/null | grep -cE '/move_base$|/amcl$|/map_server$')
echo "  nodes up: $NODES / 3"
if [ "$NODES" -lt 3 ]; then
    echo "  ABORT: the stack did not come up, see /tmp/nav_test.log"
    tail -8 /tmp/nav_test.log
    exit 1
fi

step "5/6 amcl keepalive"
# Without this AMCL stops updating while the robot stands still, its
# map->odom stamp freezes, and every costmap logs "transform timeout" and
# refuses to plan. One instance only: two of them fight over the service.
setsid nohup python3 amcl_keepalive.py --quiet > /tmp/keepalive.log 2>&1 < /dev/null &
sleep 3
if pgrep -f '[a]mcl_keepalive' > /dev/null; then
    echo "  running"
else
    echo "  WARNING: keepalive did not start, planning will stall when idle"
fi

step "5b/6 angular gain"
# The chassis turns 1.41x faster than commanded (measured 2026-11-04,
# n=486).  Divide the request so DWA's prediction matches reality.
# Gain only: the dead-zone compensator made the gait worse.
setsid nohup python3 cmd_vel_scale.py --in-topic /cmd_vel_raw \
    --out-topic /cmd_vel --angular-scale 0.709 --angular-deadzone 0.0 \
    > /tmp/cmd_vel_scale.log 2>&1 < /dev/null &
sleep 2
if pgrep -f '[c]md_vel_scale.py' > /dev/null; then
    echo "  running (angular x0.709)"
else
    echo "  WARNING: scaler did not start, see /tmp/cmd_vel_scale.log"
fi

step "6/6 rviz"
if [ "$RVIZ" -eq 1 ]; then
    if [ -z "${DISPLAY:-}" ]; then
        export DISPLAY=:0
    fi
    if [ -f "$HOME/.Xauthority" ]; then
        export XAUTHORITY="$HOME/.Xauthority"
    fi
    # No xdpyinfo probe: it is not guaranteed to be installed, and a missing
    # probe would look exactly like a missing display. Start rviz and check
    # whether it survives instead.
    setsid nohup rviz -d "$HERE/nav.rviz" > /tmp/rviz.log 2>&1 < /dev/null &
    sleep 6
    if pgrep -f '[r]viz' > /dev/null; then
        echo "  rviz window opened on $DISPLAY"
    else
        echo "  rviz did not start, see /tmp/rviz.log"
    fi
else
    echo "  skipped (--no-rviz)"
fi

echo
echo "======================================================"
echo " CLICK TO NAVIGATE"
echo "   1. toolbar: 2D Nav Goal   (or press g)"
echo "   2. click a free spot, drag to set the heading, release"
echo ""
echo "   green  = global plan      blue = DWA local plan"
echo "   yellow = where AMCL thinks the robot is"
echo "   red dots = live laser returns"
echo ""
echo " to check a point first:  nav_goal.py --check <x> <y>"
echo " to stop:                 rosnode kill /move_base"
echo "======================================================"
