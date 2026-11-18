#!/bin/bash
# ======================================================================
# run.sh — launch the recording console (record_gui.py).
#
#   ./run.sh                              # robot defaults to jetson@192.168.1.11
#   ./run.sh --robot jetson@10.0.0.5
#
# record_gui.py sets ROS_MASTER_URI and ROS_IP itself before importing
# rospy, so this script does not source ROS. It only checks the things that
# fail silently: no tkinter means no window at all, no ssh means the status
# panel stays empty forever.
#
# Daily entry is ../run.sh (the workbench); this launcher remains compatible.
# ======================================================================
cd "$(dirname "$0")" || exit 1

if [ ! -f record_gui.py ]; then
  echo "缺少 record_gui.py，请检查项目的 recording 目录" >&2
  exit 1
fi

if ! command -v python3 >/dev/null 2>&1; then
  echo "缺少 python3" >&2
  exit 1
fi

if ! python3 -c 'import tkinter' >/dev/null 2>&1; then
  echo "缺少 python3-tk：sudo apt install python3-tk" >&2
  exit 1
fi

if ! command -v ssh >/dev/null 2>&1; then
  echo "缺少 ssh 客户端：sudo apt install openssh-client" >&2
  exit 1
fi

if ! command -v sshpass >/dev/null 2>&1; then
  echo "提示：没装 sshpass，每条远程命令都会问一次密码（sudo apt install sshpass）" >&2
fi

exec python3 record_gui.py "$@"
