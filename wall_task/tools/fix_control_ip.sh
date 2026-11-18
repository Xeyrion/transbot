#!/bin/bash
# ===========================================================================
# fix_control_ip.sh -- pin the control machine's address on the robot hotspot.
#
# Why: the robot dials back to whatever address the control machine announces.
# When that address comes from DHCP it changes between sessions, and the
# already-running move_base/amcl keep advertising the old one.  The driver
# then spends its time dialling a dead address and commands stop arriving.
# A fixed address removes that whole class of failure.
#
# What it does: every NetworkManager wireless profile whose SSID is Transbot
# gets ipv4.method=manual, 192.168.1.136/24, never-default=yes, no DNS.
# never-default matters: without it the robot's internet-less hotspot becomes
# the default route and apt/pip start timing out.
#
# Usage:
#   sudo bash fix_control_ip.sh              # default 192.168.1.136/24
#   sudo bash fix_control_ip.sh 192.168.1.50/24
# ===========================================================================
set -e

ADDR="${1:-192.168.1.136/24}"
SSID="${SSID:-Transbot}"

if [ "$(id -u)" -ne 0 ]; then
    echo "run with sudo: sudo bash $0 [addr/prefix]" >&2
    exit 1
fi

FOUND=0
while IFS= read -r name; do
    [ -n "$name" ] || continue
    ssid=$(nmcli -g 802-11-wireless.ssid connection show "$name" 2>/dev/null || true)
    [ "$ssid" = "$SSID" ] || continue
    nmcli connection modify "$name" \
        ipv4.method manual \
        ipv4.addresses "$ADDR" \
        ipv4.never-default yes \
        ipv4.dns ""
    echo "  pinned: $name -> $ADDR"
    FOUND=$((FOUND + 1))
done < <(nmcli -t -f NAME,TYPE connection show \
         | awk -F: '$2 == "802-11-wireless" {print $1}')

if [ "$FOUND" -eq 0 ]; then
    echo "no wireless profile for SSID '$SSID' -- connect to it once first" >&2
    exit 1
fi

# Prefer the canonical name when several profiles point at the same SSID.
nmcli connection modify "$SSID" connection.autoconnect-priority 10 2>/dev/null || true

echo "--- $SSID ---"
nmcli -f ipv4.method,ipv4.addresses,ipv4.never-default,connection.autoconnect-priority \
    connection show "$SSID"
