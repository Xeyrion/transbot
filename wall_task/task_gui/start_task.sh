#!/bin/bash
# ===========================================================================
# start_task.sh -- one command for the whole wall task.
#
# Everything that used to be "remember to do this first" lives here, in the
# order that matters.  Each step prints what it found and fixes what it can;
# nothing is silent.
#
#   1. find the robot        config hint -> mDNS -> subnet scan -> default
#   2. ROS environment       console/env.sh, then the discovered endpoint
#   3. robot self-check      old window killed, one writer on /dev/ttyTHS1,
#                            factory /cmd_vel publishers stopped, battery
#   4. clock                 one RTT-compensated sync, then the guard daemon
#   5. nav stack             start if missing, restart if on a stale address,
#                            then verify /cmd_vel has a listener
#   6. window                exec task_gui.py
#
# Why step 3 exists: the vendor transbot_main.pyc also opens /dev/ttyTHS1.
# With two writers the MCU gets interleaved frames and silently drops
# commands -- the robot looks alive, services answer, nothing moves.
#
# Why step 4 exists: TF stamps come from the robot, lookups happen here.  A
# gap past transform_tolerance (1.5 s) stops AMCL publishing, which looks
# like "the arrow froze".
#
# Usage:
#   ./start_task.sh                 # everything, GUI on this display
#   ./start_task.sh --stage align   # open straight into a stage
#   ./start_task.sh --no-nav        # do not start the nav stack
#   ./start_task.sh --offline       # skip the robot, just open the window
#   ROBOT_IP=192.168.1.11 ./start_task.sh
# ===========================================================================

# No `set -u`: ROS's own setup.bash reads ROS_DISTRO before setting it.
set +u

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
CONF="$HOME/.config/transbot_console.conf"
ROBOT_USER="${ROBOT_USER:-jetson}"
export SSHPASS="${ROBOT_SSH_PASS:-yahboom}"
SSH_OPTS="-n -o StrictHostKeyChecking=no -o ConnectTimeout=6"

GUI_ARGS=()
START_NAV=1
OFFLINE=0
while [ $# -gt 0 ]; do
    case "$1" in
        --no-nav)  START_NAV=0; shift ;;
        --offline) OFFLINE=1; shift ;;
        --stage)   GUI_ARGS+=(--stage "$2"); shift 2 ;;
        -h|--help) sed -n '2,32p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) GUI_ARGS+=("$1"); shift ;;
    esac
done

step() { printf '\n=== %s ===\n' "$*"; }
ok()   { printf '  ok    %s\n' "$*"; }
warn() { printf '  warn  %s\n' "$*"; }
die()  { printf '  FAIL  %s\n' "$*"; exit 1; }

probe_master() {
    timeout 1 bash -c "echo > /dev/tcp/$1/11311" 2>/dev/null
}

# ---------------------------------------------------------------- 1. robot --
step "1/6 find the robot"
ROBOT=""
if [ "$OFFLINE" -eq 1 ]; then
    ROBOT="${ROBOT_IP:-192.168.1.11}"
    ok "offline: skipping robot discovery"
elif [ -n "${ROBOT_IP:-}" ]; then
    ROBOT="$ROBOT_IP"; ok "from \$ROBOT_IP: $ROBOT"
elif [ -f "$CONF" ]; then
    HINT=$(sed -nE 's/^\s*ROBOT_IP\s*=\s*([0-9.]+).*/\1/p' "$CONF" | tail -1)
    if [ -n "$HINT" ] && probe_master "$HINT"; then
        ROBOT="$HINT"; ok "from config, reachable: $ROBOT"
    elif [ -n "$HINT" ]; then
        warn "config says $HINT but the master is not answering"
    fi
fi
if [ -z "$ROBOT" ]; then
    MDNS=$(getent ahostsv4 transbot.local 2>/dev/null | awk 'NR==1{print $1}')
    if [ -n "$MDNS" ] && probe_master "$MDNS"; then
        ROBOT="$MDNS"; ok "via mDNS: $ROBOT"
    fi
fi
if [ -z "$ROBOT" ]; then
    # Scan the WiFi subnet for an open 11311.  Only 254 hosts, and the VM
    # subnets are excluded so the scan cannot wander into 192.168.240.x.
    LOCAL=$(ip -4 -o addr show 2>/dev/null \
        | awk '$2 ~ /^(wl|wlan)/ {split($4,a,"/"); print a[1]; exit}')
    if [ -n "$LOCAL" ]; then
        BASE=$(echo "$LOCAL" | cut -d. -f1-3)
        printf '  ..    scanning %s.0/24 for :11311\n' "$BASE"
        ROBOT=$(seq 2 254 | xargs -P 48 -I{} sh -c \
            "timeout 0.5 bash -c 'echo > /dev/tcp/$BASE.{}/11311' 2>/dev/null \
             && echo $BASE.{}" 2>/dev/null | head -1)
        [ -n "$ROBOT" ] && ok "found by scan: $ROBOT"
    fi
fi
if [ -z "$ROBOT" ]; then
    ROBOT=192.168.1.11
    warn "nothing found, falling back to the factory hotspot address $ROBOT"
fi

# Remember a discovered address, so the next start is instant.  A stale entry
# is harmless: it is only used when the master actually answers.
if [ "$OFFLINE" -eq 0 ] && { [ ! -f "$CONF" ] || ! grep -q "ROBOT_IP *= *$ROBOT" "$CONF" 2>/dev/null; }; then
    mkdir -p "$(dirname "$CONF")"
    if [ -f "$CONF" ]; then
        sed -i -E "s/^\s*ROBOT_IP\s*=.*/ROBOT_IP = $ROBOT/" "$CONF"
    else
        printf 'ROBOT_IP = %s\n' "$ROBOT" > "$CONF"
    fi
fi
export ROBOT_IP="$ROBOT"

# ------------------------------------------------------------- 2. ROS env --
step "2/6 ROS environment"
ENVSH=""
for c in "$ROOT/console/env.sh"; do
    [ -f "$c" ] && ENVSH="$c" && break
done
if [ -n "$ENVSH" ]; then
    source "$ENVSH" >/dev/null 2>&1
    ok "sourced $ENVSH"
else
    warn "env.sh not found; using a bare ROS environment"
    source "$ROOT/runtime/ros_env.sh"
fi

if [ "$OFFLINE" -eq 0 ] && ! probe_master "$ROBOT"; then
    warn "no ROS master at $ROBOT:11311 -- starting offline"
    OFFLINE=1
fi

if [ "$OFFLINE" -eq 0 ]; then
    export ROS_MASTER_URI="http://$ROBOT:11311"
    LOCAL=$(ip route get "$ROBOT" 2>/dev/null \
        | awk '{for(i=1;i<=NF;i++) if($i=="src"){print $(i+1); exit}}')
    [ -z "$LOCAL" ] && LOCAL=$(ip -4 -o addr show | awk '$2 ~ /^(wl|wlan)/ {
        split($4,a,"/"); print a[1]; exit}')
    export ROS_IP="$LOCAL" ROS_HOSTNAME="$LOCAL"
    ok "master $ROS_MASTER_URI"
    ok "this host $ROS_IP"
else
    ok "offline mode"
fi

ssh_robot() { sshpass -e ssh $SSH_OPTS "$ROBOT_USER@$ROBOT" "$1"; }

# --------------------------------------------------- 3. robot self-check ---
if [ "$OFFLINE" -eq 0 ]; then
    step "3/6 robot self-check"

    # 3a. exactly one writer on the serial port
    OWNERS=$(ssh_robot 'fuser /dev/ttyTHS1 2>/dev/null | wc -w' 2>/dev/null \
        | tr -d '[:space:]')
    if [ "$OWNERS" = "1" ]; then
        ok "/dev/ttyTHS1 has 1 writer"
    elif [ "$OWNERS" = "0" ] || [ -z "$OWNERS" ]; then
        warn "/dev/ttyTHS1 has no writer -- is the robot stack up?"
    else
        warn "/dev/ttyTHS1 has $OWNERS writers; stopping the vendor app"
        ssh_robot 'pkill -TERM -f transbot_main 2>/dev/null; sleep 2; \
                   fuser /dev/ttyTHS1 2>/dev/null | wc -w' >/dev/null 2>&1
        sleep 1
        OWNERS=$(ssh_robot 'fuser /dev/ttyTHS1 2>/dev/null | wc -w' \
            2>/dev/null | tr -d '[:space:]')
        if [ "$OWNERS" = "1" ]; then
            ok "vendor app stopped, 1 writer left"
        else
            warn "still $OWNERS writers -- commands may be ignored"
        fi
    fi

    # 3a-bis. the previous window.  Two GUIs both publish /cmd_vel and both
    # hold subscriptions the driver has to dial; replacing it here also means
    # its registration can be cleaned up in the next block.
    OLDGUI=$(pgrep -f '[t]ask_gui.py' | xargs)
    if [ -n "$OLDGUI" ]; then
        warn "an older task_gui is running (pid $OLDGUI); replacing it"
        pkill -f '[t]ask_gui.py' 2>/dev/null
        sleep 1
    fi

    # A recorder started from the window outlives it when the window is
    # killed instead of closed (SIGTERM never reaches on_close).  It keeps a
    # /cmd_vel subscription, which is one more endpoint the driver has to
    # dial, so stop it here.  SIGTERM makes it write its report first.
    ORPHAN=$(pgrep -f '[n]av_trace.py' | xargs)
    if [ -n "$ORPHAN" ]; then
        warn "leftover recorder (pid $ORPHAN); stopping it"
        pkill -TERM -f '[n]av_trace.py' 2>/dev/null
        sleep 1
    fi

    # 3a-ter. dead /task_gui_* registrations from a previous kill.  Each one
    # is a publisher the robot driver keeps trying to dial; enough of them and
    # it stops accepting new publishers, which looks like "clicks do nothing".
    if command -v rosnode >/dev/null 2>&1; then
        # We just killed the previous window, so every /task_gui_* entry that
        # is still registered is dead by definition.  rospy does not reliably
        # unregister on SIGTERM, so clean them here instead of waiting for a
        # second one to appear.
        STALE=$(timeout 8 rosnode list 2>/dev/null | grep -c '^/task_gui_')
        if [ "$STALE" -gt 0 ]; then
            warn "$STALE dead /task_gui registration(s); cleaning"
            echo y | timeout 40 rosnode cleanup >/dev/null 2>&1
            STALE=$(timeout 8 rosnode list 2>/dev/null | grep -c '^/task_gui_')
            ok "now $STALE"
        fi
    fi

    # 3b. factory nodes that publish /cmd_vel and /TargetAngle themselves
    if command -v rosnode >/dev/null 2>&1; then
        KILLED=""
        for n in /TransbotPatrol /BigTransbot; do
            if timeout 6 rosnode list 2>/dev/null | grep -qx "$n"; then
                timeout 8 rosnode kill "$n" >/dev/null 2>&1 && KILLED="$KILLED $n"
            fi
        done
        if [ -n "$KILLED" ]; then
            ok "stopped$KILLED (they publish /cmd_vel)"
        else
            ok "no factory /cmd_vel publishers running"
        fi
    fi

    # 3c. battery.  There is no low-voltage cut-off in the driver, so a flat
    # pack produces no error at all -- the chassis simply delivers a fraction
    # of the commanded speed and every button looks dead.  Measured: 10.3 V
    # gives ~25 % (0.20 m/s asked, 0.05 m/s actual).
    VOLT=$(timeout 6 rostopic echo -n1 /voltage 2>/dev/null \
        | awk '/Voltage/{printf "%.1f", $2; exit}')
    if [ -z "$VOLT" ]; then
        warn "no /voltage -- is the robot driver up?"
    elif awk "BEGIN{exit !($VOLT < 11.0)}"; then
        warn "battery ${VOLT} V -- about 25% of the commanded speed;"
        warn "  the base will move, just barely. Charge to 12 V before a demo"
    else
        ok "battery ${VOLT} V"
    fi
fi

# --------------------------------------------------------------- 4. clock --
if [ "$OFFLINE" -eq 0 ]; then
    step "4/6 clock"
    GUARD="$HERE/../tools/clock_guard.py"
    if [ -f "$GUARD" ]; then
        # Always sync once: it is two ssh calls, and starting the session with
        # a known-good offset avoids the first TF failure of the day.
        timeout 30 python3 "$GUARD" --sync 2>&1 | tail -1 | sed 's/^/  sync  /'
        if ! pgrep -f '[c]lock_guard.py' >/dev/null 2>&1; then
            setsid nohup python3 "$GUARD" > /tmp/clock_guard.log 2>&1 < /dev/null &
            ok "guard started (corrects past 0.3 s, every 30 s)"
        else
            ok "guard already running"
        fi
    else
        warn "clock_guard.py missing; TF may break when the clocks drift"
    fi
fi

# ----------------------------------------------------------- 5. nav stack --
if [ "$OFFLINE" -eq 0 ] && [ "$START_NAV" -eq 1 ]; then
    step "5/6 navigation stack"
    UP=$(timeout 8 rosnode list 2>/dev/null \
        | grep -cE '/map_server$|/amcl$|/move_base$')

    # A node can be alive and still useless: after the control machine's
    # address changes, old move_base/amcl keep advertising the previous one.
    # The robot driver then dials a dead address for every /scan and /odom
    # connection and stops accepting commands -- which looks exactly like
    # "everything is ok but the robot does not move".  rosnode list only
    # shows the name, so compare the XML-RPC host as well.
    STALE=""
    for n in /map_server /amcl /move_base; do
        H=$(timeout 5 rosnode info "$n" 2>/dev/null \
            | sed -n 's#.*http://\([0-9.]*\):[0-9]*/.*#\1#p' | head -1)
        if [ -n "$H" ] && [ "$H" != "$ROS_IP" ]; then
            STALE="$STALE $n@$H"
        fi
    done
    if [ -n "$STALE" ]; then
        warn "nav nodes on a stale address:$STALE (this host is $ROS_IP)"
        pkill -f '[m]ove_base' 2>/dev/null
        pkill -f '[a]mcl' 2>/dev/null
        pkill -f '[m]ap_server' 2>/dev/null
        sleep 2
        echo y | timeout 30 rosnode cleanup >/dev/null 2>&1
        UP=0
    fi

    if [ "$UP" -ge 3 ]; then
        ok "map_server + amcl + move_base already up"
    else
        warn "only $UP/3 nav nodes; starting the stack (about 40 s)"
        if [ -x "$HERE/../nav/scripts/start_nav.sh" ]; then
            ( cd "$HERE/../nav/scripts" && \
              setsid nohup ./start_nav.sh --no-rviz > /tmp/start_nav.log 2>&1 \
              < /dev/null & )
            for _ in $(seq 1 30); do
                sleep 2
                UP=$(timeout 8 rosnode list 2>/dev/null \
                    | grep -cE '/map_server$|/amcl$|/move_base$')
                [ "$UP" -ge 3 ] && break
            done
            if [ "$UP" -ge 3 ]; then
                POSE=$(grep -m1 'coarse pose' /tmp/start_nav.log 2>/dev/null)
                ok "stack up${POSE:+  ($POSE)}"
                warn "check the arrow on the map; use 设初始位姿 if it is wrong"
            else
                warn "stack did not come up, see /tmp/start_nav.log"
            fi
        else
            warn "start_nav.sh not found; start the stack yourself"
        fi
    fi

    # --- verify what is actually running, not just what is named ----------
    BAD=""
    for n in /map_server /amcl /move_base; do
        H=$(timeout 5 rosnode info "$n" 2>/dev/null \
            | sed -n 's#.*http://\([0-9.]*\):[0-9]*/.*#\1#p' | head -1)
        if [ -n "$H" ] && [ "$H" != "$ROS_IP" ]; then
            BAD="$BAD $n@$H"
        fi
    done
    if [ -n "$BAD" ]; then
        warn "still on a stale address:$BAD"
        warn "  kill them and rerun: pkill -f move_base; pkill -f amcl; rosnode cleanup"
    else
        ok "all three nav nodes on $ROS_IP"
    fi

    SUBS=$(timeout 6 rostopic info /cmd_vel 2>/dev/null \
        | sed -n '/Subscribers/,$p' | grep -c '^ \*')
    if [ "$SUBS" -ge 1 ]; then
        ok "/cmd_vel has $SUBS subscriber(s)"
    else
        warn "/cmd_vel has no subscriber -- the robot driver is not listening"
    fi
    if [ "$SUBS" -gt 1 ]; then
        # More than one listener is not fatal, but it is how a leftover
        # recorder hides: the driver has to dial every endpoint.
        warn "more than one listener; check for leftovers:"
        timeout 6 rostopic info /cmd_vel 2>/dev/null \
            | sed -n '/Subscribers/,$p' | sed 's/^/         /'
    fi

    # The angular gain relay.  move_base publishes /cmd_vel_raw and this node
    # divides by the measured 1.41 gain before /cmd_vel.  If the stack was
    # started without it the robot still drives, just 40 % too fast in turns;
    # if move_base was not remapped the relay does nothing.  Check both.
    if pgrep -f '[c]md_vel_scale.py' > /dev/null; then
        ok "angular gain relay running"
    elif timeout 5 rostopic info /cmd_vel_raw 2>/dev/null | grep -q '\*'; then
        warn "move_base publishes /cmd_vel_raw but the relay is down; starting it"
        setsid nohup python3 "$HERE/../nav/scripts/cmd_vel_scale.py" \
            --in-topic /cmd_vel_raw --out-topic /cmd_vel \
            --angular-scale 0.709 --angular-deadzone 0.0 \
            > /tmp/cmd_vel_scale.log 2>&1 < /dev/null &
        sleep 2
        if pgrep -f '[c]md_vel_scale.py' > /dev/null; then
            ok "relay started"
        else
            warn "relay did not start -- the robot will not move at all"
        fi
    else
        warn "no /cmd_vel_raw publisher: move_base is not remapped, relay idle"
    fi

    # A goal left over from the previous session starts driving the moment
    # the stack comes up, before anyone touches the window.
    ST=$(timeout 5 rostopic echo -n1 /move_base/status 2>/dev/null \
        | grep -m1 'status:' | awk '{print $2}')
    if [ "$ST" = "0" ] || [ "$ST" = "1" ]; then
        timeout 5 rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID \
            '{}' >/dev/null 2>&1
        timeout 5 rostopic pub -1 /cmd_vel geometry_msgs/Twist \
            '{linear: {x: 0.0}, angular: {z: 0.0}}' >/dev/null 2>&1
        warn "cancelled a leftover goal (status $ST) and stopped the base"
    fi
else
    step "5/6 navigation stack"
    ok "skipped"
fi

# ------------------------------------------------------------- 6. window --
step "6/6 window"
[ "$OFFLINE" -eq 1 ] && GUI_ARGS+=(--offline)
printf '  exec task_gui.py %s\n\n' "${GUI_ARGS[*]}"
exec python3 "$HERE/task_gui.py" "${GUI_ARGS[@]}"
