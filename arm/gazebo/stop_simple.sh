#!/usr/bin/env bash
# Stop the simplest-arm simulation and panel, if running.
# Closing the panel does NOT stop the scene by itself; use this (or run
# run_simple.sh again, which stops the old scene before starting a new one).
set -u

pkill -f '[a]rm_simple_gui.py' 2>/dev/null || true
pkill -f '[r]oslaunch transbot_arm_simple' 2>/dev/null || true
pkill -f '[r]oslaunch transbot_arm_gazebo' 2>/dev/null || true
pkill -f '[g]zclient' 2>/dev/null || true
pkill -f '[g]zserver' 2>/dev/null || true
pkill -f '[r]osmaster --core -p 11311' 2>/dev/null || true
sleep 2
pkill -9 -x gzserver 2>/dev/null || true
pkill -9 -x gzclient 2>/dev/null || true
pkill -9 -f '[r]oslaunch transbot_arm_simple' 2>/dev/null || true
pkill -9 -f '[r]oslaunch transbot_arm_gazebo' 2>/dev/null || true
pkill -9 -f '[r]osmaster --core -p 11311' 2>/dev/null || true
sleep 1

if timeout 2 rosnode list 2>/dev/null | grep -qx '/gazebo'; then
  echo "[simple] 还有 /gazebo 残留，请手动检查" >&2
  exit 1
fi
echo "[simple] 已全部停止"
