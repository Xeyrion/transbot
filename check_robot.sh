#!/bin/bash
# ============================================================
# transbot-remote-console / check_robot.sh
# Health check: network + ROS topics/services/nodes.
#
# Usage:
#   bash check_robot.sh          # from repo root
#   bash tasks/00-health-check.sh
# ============================================================

DIR="$(cd "$(dirname "$0")" && pwd)"
if [ -f "$DIR/env.sh" ]; then
    source "$DIR/env.sh" || exit 1
else
    echo "[error] env.sh not found next to $0" >&2
    exit 1
fi

echo
echo "[1] Network: ping robot ($ROBOT_IP)"
ping -c 2 -W 1 "$ROBOT_IP" | tail -1

echo
echo "[2] Battery voltage (base serial link) / firmware edition"
timeout 8 rostopic echo /voltage -n 1 2>&1 | head -3
timeout 8 rostopic echo /edition -n 1 2>&1 | head -3

echo
echo "[3] LiDAR /scan publishing?"
timeout 8 rostopic echo /scan -n 1 --noarr 2>&1 | grep -E "frame_id|range_min|range_max"

echo
echo "[4] Odometry /odom"
timeout 8 rostopic echo /odom -n 1 --noarr 2>&1 | grep -E "frame_id"

echo
echo "[5] Arm joint readback (/CurrentAngle service)"
timeout 8 rosservice call /CurrentAngle "apply: ''" 2>&1 | head -8

echo
echo "[6] Node count (healthy autostart stack is ~18-20 nodes)"
timeout 8 rosnode list 2>/dev/null | wc -l
