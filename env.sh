#!/bin/bash
# ============================================================
# transbot-remote-console / env.sh
# Set up ROS environment on the control PC to talk to the robot.
#
# Usage:
#   source ./env.sh                          # auto-detect robot IP
#   ROBOT_IP=<robot-ip> source ./env.sh      # override manually
#
# Robot IP resolution order:
#   1. $ROBOT_IP env var (highest priority)
#   2. ~/.config/transbot-console.conf  (ROBOT_IP=<ip>)
#   3. Factory AP default (e.g. 192.168.1.11)
#   4. mDNS hostname (e.g. Transbot.local) for STA mode
#
# NOTE on mDNS: if the control PC itself shares the robot's hostname,
# avahi may resolve the .local name to a local NIC address. This script
# filters out any address that belongs to a local interface.
# ============================================================

# ---- 1. Resolve robot IP ----
CONF=~/.config/transbot-console.conf
LEGACY_CONF=~/.config/transbot_demo.conf
ROBOT_IP="${ROBOT_IP:-}"
for _c in "$CONF" "$LEGACY_CONF"; do
    if [ -z "$ROBOT_IP" ] && [ -f "$_c" ]; then
        CONF_IP=$(sed -nE 's/^\s*ROBOT_IP\s*=\s*([0-9]+\.[0-9]+\.[0-9]+\.[0-9]+).*/\1/p' "$_c" | tail -1)
        ROBOT_IP="${CONF_IP:-}"
    fi
done
# Factory AP-mode default; override via $ROBOT_IP or the config file above.
ROBOT_IP="${ROBOT_IP:-192.168.1.11}"
# mDNS hostname of the robot (resolves to the current IP in any WiFi mode).
ROBOT_MDNS="${ROBOT_MDNS:-Transbot.local}"

# Probe whether the ROS master port is reachable.
probe() { timeout 2 bash -c "echo > /dev/tcp/$1/11311" 2>/dev/null; }

# True if $1 is one of this machine's own IPv4 addresses (mDNS self-hit guard).
is_local_ip() {
    ip -4 -o addr | awk '{print $4}' | cut -d/ -f1 | grep -qxF "$1"
}

if ! probe "$ROBOT_IP"; then
    MDNS_IP=$(getent ahostsv4 "$ROBOT_MDNS" 2>/dev/null | awk 'NR==1{print $1}')
    if [ -n "$MDNS_IP" ] && ! is_local_ip "$MDNS_IP" && probe "$MDNS_IP"; then
        echo "[found] $ROBOT_IP unreachable, resolved robot via $ROBOT_MDNS: $MDNS_IP"
        ROBOT_IP="$MDNS_IP"
    elif [ -n "$MDNS_IP" ] && is_local_ip "$MDNS_IP"; then
        echo "[hint] $ROBOT_MDNS resolved to this machine itself ($MDNS_IP), ignored."
        echo "       This happens when the PC shares the robot's hostname. Check that"
        echo "       1) the robot is powered on, 2) this PC joined the robot's WiFi."
        return 1 2>/dev/null || exit 1
    else
        echo "[error] ROS master unreachable (tried $ROBOT_IP and $ROBOT_MDNS)."
        echo "        Check that 1) the robot is powered on, 2) this PC is on the same WiFi."
        echo "        To override manually:"
        echo "          ROBOT_IP=<robot-ip> source ./env.sh"
        return 1 2>/dev/null || exit 1
    fi
fi

# ---- 2. Pick the local IP on the same subnet as the robot ----
SUB=$(echo "$ROBOT_IP" | cut -d. -f1-3)
MYIP=""
while read -r a; do
    a4=${a%/*}
    case "$a4" in
        "$SUB".*) MYIP=$a4; break ;;
    esac
done < <(ip -4 -o addr | awk '{print $4}')
if [ -z "$MYIP" ]; then
    MYIP=$(ip -4 -o route get 1.1.1.1 2>/dev/null | sed -nE 's/.*src ([0-9.]+).*/\1/p' | head -1)
fi
if [ -z "$MYIP" ]; then
    echo "[error] could not detect local IP"; return 1 2>/dev/null || exit 1
fi

# ---- 3. Source ROS + export variables ----
source /opt/ros/melodic/setup.bash
# Optional local overlays (skip silently if absent):
for _ws in ~/transbot_ws/devel/setup.bash \
           ~/software/transbot_library/devel/setup.bash \
           ~/software/world_canvas/devel/setup.bash; do
    [ -f "$_ws" ] && source "$_ws" --extend 2>/dev/null
done

export ROS_MASTER_URI=http://$ROBOT_IP:11311
export ROS_IP=$MYIP
export ROS_HOSTNAME=$MYIP

echo "-------------------- connected --------------------"
echo "  robot (master) : $ROBOT_IP"
echo "  local  (ROS_IP): $MYIP"
echo "---------------------------------------------------"
