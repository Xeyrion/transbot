#!/bin/bash
# ======================================================================
# env.sh — VM-to-robot ROS environment setup (adaptive IP resolution)
#
# Usage:
#   source ./env.sh                          # auto-resolve robot IP
#   ROBOT_IP=192.168.2.161 source ./env.sh   # explicit override
#
# Resolution precedence for the robot address:
#   1. $ROBOT_IP environment variable (explicit override, highest priority)
#   2. ROBOT_IP entry in ~/.config/transbot_console.conf
#      (legacy ~/.config/transbot-console.conf and ~/.config/transbot_demo.conf
#       are still accepted)
#   3. 192.168.1.11 (factory default in hotspot/AP mode)
#   4. mDNS name Transbot.local (tracks the robot's current address in
#      station/STA mode, where the DHCP-assigned address varies)
#
# Explicit/pinned addresses (1 and 2) skip mDNS and are kept even when the
# robot is offline. Only the default address falls through to mDNS; when
# that also fails, the endpoint is still exported but the function returns
# non-zero so ~/.bashrc can switch to local single-machine mode.
#
# mDNS caveat: if the local host shares the name "Transbot", avahi resolves
# transbot.local to a local interface. Candidates matching a local interface
# address are therefore discarded to prevent self-connection.
#
# ROS setup order matters: the ROS workspaces are loaded FIRST, so that
# `rospy` / `transbot_msgs` stay importable even when the robot is offline
# (the GUI must still open for offline inspection).
#
# Must be sourced (not executed) so exports apply to the calling shell.
# ======================================================================

# Directory of this script (works when sourced from any location).
_ENV_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_VENDOR_PY="$_ENV_DIR/vendor/python"

# ---- 0. Load ROS workspaces (always, before anything can bail out) ----
if [ -f /opt/ros/melodic/setup.bash ]; then
    source /opt/ros/melodic/setup.bash
fi

# Some ROS Melodic images ship the Python modules only under
# lib/python2.7/dist-packages (still importable from python3) but place
# "rospkg" in the system python2 directory, which python3 does not see.
# Without it, "import rospy" dies with "No module named 'rospkg'".
# Expose just that package through an isolated directory so python3 can
# find it without pulling in the rest of the python2 tree.
if ! python3 -c "import rospy" >/dev/null 2>&1; then
    for _rospkg in /usr/lib/python2.7/dist-packages/rospkg \
                   /usr/lib/python3/dist-packages/rospkg \
                   /usr/local/lib/python2.7/dist-packages/rospkg
    do
        if [ -d "$_rospkg" ]; then
            _shim="${XDG_CACHE_HOME:-$HOME/.cache}/transbot_console/rospkg_shim"
            if mkdir -p "$_shim" 2>/dev/null; then
                ln -sfn "$_rospkg" "$_shim/rospkg" 2>/dev/null && \
                    export PYTHONPATH="$_shim${PYTHONPATH:+:$PYTHONPATH}"
            fi
            break
        fi
    done
    unset _rospkg _shim
fi

# Catkin workspaces that may hold transbot_msgs. Each is optional; the
# first ones that exist are sourced with --extend so they stack on top of
# the ROS underlay. Glob candidates cover workspaces living under another
# user's home directory.
for _ws in \
    "$HOME/transbot_ws" \
    "$HOME/software/transbot_ws" \
    "$HOME/software/transbot_library" \
    "$HOME/software/world_canvas" \
    /home/*/transbot_ws \
    /home/*/software/transbot_ws \
    /home/*/software/transbot_library \
    /home/*/software/world_canvas
do
    if [ -f "$_ws/devel/setup.bash" ]; then
        source "$_ws/devel/setup.bash" --extend 2>/dev/null
    fi
done
unset _ws

# transbot_msgs is the official Yahboom message package. It normally comes
# from a built catkin workspace, but the GUI must not fail with
# "No module named 'transbot_msgs'" when no workspace is present. The
# pre-generated modules shipped in vendor/python are used as a fallback
# (they only need genpy from ROS Melodic — no catkin build required).
if ! python3 -c "import transbot_msgs" >/dev/null 2>&1; then
    if [ -d "$_VENDOR_PY/transbot_msgs" ]; then
        export PYTHONPATH="$_VENDOR_PY${PYTHONPATH:+:$PYTHONPATH}"
    fi
fi

# ---- 1. Resolve robot IP ----
# Preferred config name; fall back to the pre-consolidation names so existing
# machines keep working without editing anything.  `transbot-console.conf` is
# the hyphenated name used by this repo before the layout was unified.
CONF=~/.config/transbot_console.conf
LEGACY_CONF=~/.config/transbot-console.conf
LEGACY_CONF2=~/.config/transbot_demo.conf
ROBOT_IP="${ROBOT_IP:-}"
# Explicit env var / pinned config must NOT fall through to mDNS: the user
# asked for this exact master, even if it is currently offline.
_ROBOT_EXPLICIT=0
[ -n "$ROBOT_IP" ] && _ROBOT_EXPLICIT=1
_ROBOT_FROM_CONF=0
for _conf in "$CONF" "$LEGACY_CONF" "$LEGACY_CONF2"; do
    if [ -z "$ROBOT_IP" ] && [ -f "$_conf" ]; then
        CONF_IP=$(sed -nE 's/^\s*ROBOT_IP\s*=\s*([0-9]+\.[0-9]+\.[0-9]+\.[0-9]+).*/\1/p' "$_conf" | tail -1)
        ROBOT_IP="${CONF_IP:-}"
        [ -n "$ROBOT_IP" ] && _ROBOT_FROM_CONF=1
    fi
done
unset _conf
ROBOT_IP="${ROBOT_IP:-192.168.1.11}"

# TCP probe of the ROS master port (11311) on a candidate address.
probe() { timeout 2 bash -c "echo > /dev/tcp/$1/11311" 2>/dev/null; }

# True if the argument is assigned to a local interface (mDNS self-hit guard).
is_local_ip() {
    ip -4 -o addr | awk '{print $4}' | cut -d/ -f1 | grep -qxF "$1"
}

if ! probe "$ROBOT_IP"; then
    if [ "$_ROBOT_EXPLICIT" = 1 ] || [ "$_ROBOT_FROM_CONF" = 1 ]; then
        # Pinned address: keep it, don't second-guess with mDNS.
        echo "Warning: pinned ROS master $ROBOT_IP:11311 is unreachable."
        echo "Using it anyway; the GUI still opens for offline inspection."
    else
        # Default candidate unreachable; fall back to mDNS discovery.
        MDNS_IP=$(getent ahostsv4 transbot.local 2>/dev/null | awk 'NR==1{print $1}')
        if [ -n "$MDNS_IP" ] && ! is_local_ip "$MDNS_IP" && probe "$MDNS_IP"; then
            echo "Primary $ROBOT_IP unreachable; using Transbot.local -> $MDNS_IP"
            ROBOT_IP="$MDNS_IP"
        elif [ -n "$MDNS_IP" ] && is_local_ip "$MDNS_IP"; then
            echo "Transbot.local resolved to a local interface ($MDNS_IP); ignored."
            echo "Cause: local hostname collides with the robot name."
            echo "Check: 1) robot powered on 2) WiFi joined to hotspot Transbot."
            echo "Note: ROS is configured; the GUI still opens for offline inspection."
            _ROS_OFFLINE=1
        else
            echo "ROS master unreachable at $ROBOT_IP and via Transbot.local."
            echo "Check: 1) robot powered on 2) host on hotspot Transbot or same WLAN."
            echo "For STA mode (robot on external router), specify explicitly:"
            echo "  ROBOT_IP=<robot-ip> source ./env.sh"
            echo "Note: ROS is configured; the GUI still opens for offline inspection."
            _ROS_OFFLINE=1
        fi
    fi
fi

# ---- 2. Select the local interface on the robot's subnet ----
SUB=$(echo "$ROBOT_IP" | cut -d. -f1-3)   # e.g. 192.168.1 from 192.168.1.11
MYIP=""
while read -r a; do
    a4=${a%/*}
    case "$a4" in
        "$SUB".*) MYIP=$a4; break ;;
    esac
done < <(ip -4 -o addr | awk '{print $4}')
if [ -z "$MYIP" ]; then
    # No interface on the robot subnet; fall back to the default-route source address.
    MYIP=$(ip -4 -o route get 1.1.1.1 2>/dev/null | sed -nE 's/.*src ([0-9.]+).*/\1/p' | head -1)
fi
if [ -z "$MYIP" ]; then
    echo "Failed to determine local IP address."; return 1 2>/dev/null || exit 1
fi

# ---- 3. Export endpoint variables ----
# ROS_MASTER_URI: master to register/query. ROS_IP/ROS_HOSTNAME: address
# this host advertises; the robot dials back to it, so it must be routable
# from the robot (hence same-subnet selection above).
export ROS_MASTER_URI=http://$ROBOT_IP:11311
export ROS_IP=$MYIP
export ROS_HOSTNAME=$MYIP

echo "-------------------- ROS endpoint configured --------------------"
echo "  Robot (master) : $ROBOT_IP"
echo "  Local (ROS_IP) : $MYIP"
echo "-----------------------------------------------------------------"

unset _ROBOT_EXPLICIT _ROBOT_FROM_CONF CONF LEGACY_CONF LEGACY_CONF2 CONF_IP

# Default (non-pinned) address unreachable: keep the endpoint configured
# (so standalone tools still have a master), but return non-zero so
# ~/.bashrc can switch to local single-machine mode for debugging.
if [ "${_ROS_OFFLINE:-0}" = 1 ]; then
    unset _ROS_OFFLINE
    return 1 2>/dev/null || exit 1
fi
unset _ROS_OFFLINE
