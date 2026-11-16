#!/usr/bin/env bash
# restart_camera.sh -- 干净重启 Astra 相机。
#
# 为什么需要它：Astra 驱动在订阅者全部断开后会 stopStream，之后不一定能自己
# 重新开流。表现是话题都在、rostopic hz 却没数据，录出来的包只有 camera_info。
# 唯一可靠的恢复办法是把驱动重启一遍。
#
#   ./restart_camera.sh          # 重启并等出图
#   ./restart_camera.sh --stop   # 只停
set -u

LOG=/tmp/astra.log
LAUNCH="astra_camera astrapro.launch"

set +u
source /opt/ros/melodic/setup.bash 2>/dev/null || true
source "$HOME/software/transbot_library/devel/setup.bash" --extend 2>/dev/null || true
set -u
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
export ROS_IP="${ROS_IP:-192.168.1.11}"
unset ROS_HOSTNAME

# 用 rosnode kill，不用 pkill -f：pkill 会匹配到执行这条命令的 shell 自己
# （它的命令行里就带着 launch 名字），于是把自己杀掉，SSH 直接 255。
stop_camera() {
  rosnode list 2>/dev/null | grep -qx /camera && rosnode kill /camera >/dev/null 2>&1
  # roslaunch 的父进程可能还挂着，一并收掉；这里按 roslaunch 的名字精确匹配
  for p in $(pgrep -f 'bin/roslaunch astra_camera' 2>/dev/null); do
    kill -TERM "$p" 2>/dev/null
  done
  for _ in $(seq 1 40); do
    rosnode list 2>/dev/null | grep -qx /camera || break
    sleep 0.25
  done
  sleep 1
}

stop_camera
[ "${1:-}" = "--stop" ] && { echo "相机已停止"; exit 0; }

setsid nohup roslaunch $LAUNCH enable_point_cloud:=false depth_align:=true \
  > "$LOG" 2>&1 < /dev/null &

for _ in $(seq 1 60); do
  sleep 1
  if timeout 3 rostopic echo -n1 --noarr /camera/rgb/image_raw >/dev/null 2>&1; then
    echo "相机就绪（RGB 有图）"
    exit 0
  fi
done

echo "ERROR: 60 秒内没等到图像，看日志：tail -30 $LOG" >&2
exit 1
