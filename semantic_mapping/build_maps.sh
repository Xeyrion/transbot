#!/usr/bin/env bash
# build_maps.sh -- 批量离线建图。在控制机上跑，不需要机器人。
#
#   ./build_maps.sh                 # 扫描 bags/，把还没有地图的会话都建一遍
#   ./build_maps.sh --dry-run       # 只列清单
#   ./build_maps.sh bags/session_x  # 只处理一个会话
#
# 单个会话的实现在 rebuild_map.py（私有 ROS master + gmapping 回放），
# 这里只负责发现、跳过和汇总。
set -eo pipefail
# set -u 会让 ROS 的 setup.bash 静默退出（里面引用了未定义变量）
set +u
source "$(dirname "${BASH_SOURCE[0]}")/../runtime/ros_env.sh"
set -u
cd "$(dirname "$0")"
exec python3 build_maps.py "$@"
