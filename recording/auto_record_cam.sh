#!/usr/bin/env bash
# auto_record_cam.sh -- 一键录制 RGB + 深度 + 雷达 + 位姿到同一个 bag。
#
# 用法：
#   ./auto_record_cam.sh                       # 默认录到 ~/bags，一直录，Ctrl-C 停
#   ./auto_record_cam.sh -o /media/jetson/bags # 自选保存目录
#   ./auto_record_cam.sh -d 120                # 录 120 秒
#   ./auto_record_cam.sh -p nav                # 只录雷达/里程计/TF（控制机走 WiFi 用这个）
#   ./auto_record_cam.sh -c                    # 只检查，不录（dry-run）
#   ./auto_record_cam.sh -A                    # 缺 REQUIRED 话题也录（台架验证链路用）
#   ./auto_record_cam.sh --list-disks          # 看硬盘和挂载点
#
# 环境变量（都可被命令行覆盖）：
#   OUT_DIR=...  DURATION=...  PROFILE=full|nav
#   MIN_FREE_GB=5  SPLIT_MB=2048  MAX_SPLITS=0  WAIT_SEC=30
#
# 注意：
#   * full 模式包含相机原图，必须在机器人本地录；控制机走 WiFi 会掉帧。
#   * nav 模式只有小数据，控制机也能录。
#   * 外置盘建议 exFAT；录完 sync + umount 再拔。
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

OUT_DIR="${OUT_DIR:-$HOME/bags}"
DURATION="${DURATION:-0}"
PROFILE="${PROFILE:-full}"
MIN_FREE_GB="${MIN_FREE_GB:-5}"
SPLIT_MB="${SPLIT_MB:-2048}"
MAX_SPLITS="${MAX_SPLITS:-0}"
WAIT_SEC="${WAIT_SEC:-30}"
CHECK_ONLY=0
ALLOW_MISSING=0

usage() {
  sed -n '2,20p' "$0" | sed 's/^# \{0,1\}//'
}

while [ $# -gt 0 ]; do
  case "$1" in
    -o|--out)       OUT_DIR="$2"; shift 2 ;;
    -d|--duration)  DURATION="$2"; shift 2 ;;
    -p|--profile)   PROFILE="$2"; shift 2 ;;
    -c|--check)     CHECK_ONLY=1; shift ;;
    -A|--allow-missing) ALLOW_MISSING=1; shift ;;
    -h|--help)      usage; exit 0 ;;
    --list-disks)   lsblk -f; echo; df -hT | grep -vE 'tmpfs|udev|loop'; exit 0 ;;
    *) echo "未知参数：$1" >&2; usage >&2; exit 2 ;;
  esac
done

for value in "$DURATION" "$MIN_FREE_GB" "$SPLIT_MB" "$MAX_SPLITS" "$WAIT_SEC"; do
  [[ "$value" =~ ^[0-9]+$ ]] || { echo 'ERROR: recording limits must be nonnegative integers' >&2; exit 2; }
done
[ "$SPLIT_MB" -gt 0 ] || { echo 'ERROR: SPLIT_MB must be positive' >&2; exit 2; }
if [ "$MAX_SPLITS" -gt 0 ] && [ "${ALLOW_RING_BUFFER:-0}" != 1 ]; then
  echo 'ERROR: ring-buffer deletion requires explicit ALLOW_RING_BUFFER=1' >&2
  exit 2
fi
for helper in session_finalize.py session_status.py stop_recording.py record_quality.py record_guard.py live_health.py stream_health.py clock_check.py; do
  [ -f "$SCRIPT_DIR/$helper" ] || { echo "ERROR: incomplete recorder deployment: $helper" >&2; exit 1; }
done
command -v python2 >/dev/null || { echo 'ERROR: python2 rosbag reader is required for finalization' >&2; exit 1; }

# 话题分两类：
#   REQUIRED —— 缺一个就拒绝开录（少一路这包就没用了）
#   WANT     —— 有就录，没有就跳过并打一行提示
# 2026-11-10 那批包只录了相机+雷达+里程计，结果 /map 和 /cmd_vel 都没有：
# 前者导致"叠地图"无从谈起，后者导致没法做 (观测, 动作) 的训练对。
case "$PROFILE" in
  full)
    WANT=(
      # 相机（原图 + 内参，重建点云要用 camera_info）
      /camera/rgb/image_raw /camera/depth/image_raw
      /camera/rgb/camera_info /camera/depth/camera_info
      # 底盘与传感器
      /scan /odom /odom_raw /imu/data /imu/data_raw /tf /tf_static
      # 实际下发的速度：做行为克隆必须有它
      /cmd_vel
      # 地图与定位：上次就是缺这个
      /map /map_metadata /amcl_pose
      # 电量：判断低速是电池还是算法
      /voltage
      # 排障：哪个节点在骂人、什么时候开始骂的
      /rosout
    )
    REQUIRED=(/camera/rgb/image_raw /camera/depth/image_raw /scan /odom
              /camera/rgb/camera_info /camera/depth/camera_info /tf)
    ;;
  nav)
    WANT=(
      /scan /odom /odom_raw /imu/data /tf /tf_static
      /cmd_vel /map /map_metadata /amcl_pose /voltage /rosout
    )
    REQUIRED=(/scan /odom /tf)
    ;;
  *) echo "PROFILE 只能是 full 或 nav" >&2; exit 2 ;;
esac

# ---------- ROS 环境 ----------
# ROS 的 setup.bash 会引用未定义变量（CMAKE_PREFIX_PATH、ROS_DISTRO...），
# 本脚本开了 set -u，直接 source 会让脚本在第 82 行静默退出、一句提示都没有。
# 必须临时关掉 -u（-e 也关掉，setup.bash 里的判断语句返回非 0 会误伤）。
set +eu
if ! command -v rosbag >/dev/null 2>&1; then
  # shellcheck disable=SC1091
  source /opt/ros/melodic/setup.bash 2>/dev/null || true
fi
for _ws in "$HOME/software/transbot_library/devel/setup.bash" \
           "$HOME/transbot_ws/devel/setup.bash"; do
  if [ -f "$_ws" ]; then
    # shellcheck disable=SC1090
    source "$_ws" --extend 2>/dev/null || true
  fi
done
set -eu
if ! command -v rosbag >/dev/null 2>&1; then
  echo "ERROR: 找不到 rosbag。先 source ROS：" >&2
  echo "  source /opt/ros/melodic/setup.bash" >&2
  exit 1
fi
if [ -z "${ROS_MASTER_URI:-}" ]; then
  export ROS_MASTER_URI="http://localhost:11311"
fi
# Jetson 上 `getent hosts Transbot` 只给出 IPv6 链路本地地址，rospy 初始化会
# 一直等一个连不上的 URI，表现成“卡住/ROSInitException”。直接给 IPv4。
if [ -z "${ROS_IP:-}" ]; then
  _ip=$(hostname -I 2>/dev/null | awk '{print $1}')
  [ -n "$_ip" ] && export ROS_IP="$_ip"
fi
unset ROS_HOSTNAME

if ! CLOCK_REPORT=$(python3 "$SCRIPT_DIR/clock_check.py"); then
  echo "$CLOCK_REPORT" >&2
  echo 'ERROR: 时钟检查失败；先停止数据源并完成对时，再重新启动传感器/EKF。' >&2
  exit 1
fi
echo "$CLOCK_REPORT"

# ---------- 环境提示 ----------
ARCH=$(uname -m)
if [ "$PROFILE" = "full" ] && [ "$ARCH" != "aarch64" ]; then
  echo "提示：当前是 $ARCH（不像 Jetson）。full 模式含相机原图，"
  echo "      在控制机上走 WiFi 会掉帧，建议在机器人本地跑，或改用 -p nav。"
  echo
fi

# ---------- 磁盘检查 ----------
mkdir -p "$OUT_DIR"
AVAIL_KB=$(df -Pk "$OUT_DIR" | awk 'NR==2 {print $4}')
NEED_KB=$((MIN_FREE_GB * 1024 * 1024))
AVAIL_GB=$((AVAIL_KB / 1024 / 1024))
if [ "$AVAIL_KB" -lt "$NEED_KB" ]; then
  echo "ERROR: $OUT_DIR 只剩 ${AVAIL_GB} GB，至少需要 ${MIN_FREE_GB} GB" >&2
  echo "  换目录：$0 -o /media/jetson/bags" >&2
  echo "  或清理：rm -f $OUT_DIR/*.bag（确认不要了再删）" >&2
  exit 1
fi

# ---------- 等话题 ----------
echo "检查 ROS master：$ROS_MASTER_URI"
if ! timeout 5 rostopic list >/dev/null 2>&1; then
  echo "ERROR: 连不上 ROS master。先起机器人/本机 ROS：" >&2
  echo "  机器人端：roslaunch transbot_nav transbot_bringup.launch" >&2
  exit 1
fi

echo "等待话题（最多 ${WAIT_SEC}s）..."
deadline=$(( $(date +%s) + WAIT_SEC ))
for t in "${REQUIRED[@]}"; do
  ok=0
  while ! rostopic list 2>/dev/null | grep -qx "$t"; do
    if [ "$(date +%s)" -ge "$deadline" ]; then
      if [ "$ALLOW_MISSING" -eq 1 ]; then
        echo "  WARN 等不到 $t，--allow-missing 继续（这个包缺这一路）"
        break
      fi
      echo "ERROR: 等不到 $t" >&2
      echo "  相机/雷达没起？" >&2
      echo "  roslaunch transbot_nav transbot_bringup.launch" >&2
      exit 1
    fi
    sleep 1
  done
  rostopic list 2>/dev/null | grep -qx "$t" && ok=1
  [ "$ok" -eq 1 ] && echo "  OK  $t"
done

# /map 和 /amcl_pose 要等 SLAM/定位跑起来才有，单独给 15 秒。
# 但没跑 SLAM 的时候等满 30 秒纯属浪费 —— 先看有没有相关节点。
if rosnode list 2>/dev/null | grep -qE 'gmapping|cartographer|amcl|slam'; then
  for t in /map /amcl_pose; do
    d2=$(( $(date +%s) + 15 ))
    while ! rostopic list 2>/dev/null | grep -qx "$t"; do
      [ "$(date +%s)" -ge "$d2" ] && break
      sleep 1
    done
  done
else
  echo "  （没有 SLAM/定位节点在跑，不等 /map 与 /amcl_pose）"
fi

# 整个清单都订阅，不要按"现在存不存在"过滤。
#
# 这里踩过坑：2026-11-11 那次开录时 /cmd_vel 和 /imu/data_raw 恰好没有发布者
# （还没人开始开车），被 `rostopic list` 过滤掉，整段录制没有动作数据 —— 而这
# 正是做 (观测, 动作) 训练对最关键的一列。
#
# rosbag record 完全可以订阅一个还不存在的话题：它先向 master 注册订阅，话题
# 一旦出现就自动开始收。所以过滤纯属自找麻烦，只在日志里提示一声就够了。
published=$(rostopic list -p 2>/dev/null)
TOPICS=()
DEAD=()
for t in "${WANT[@]}"; do
  TOPICS+=("$t")
  printf '%s\n' "$published" | grep -qx "$t" || DEAD+=("$t")
done
if [ ${#DEAD[@]} -gt 0 ]; then
  echo "  note 这些话题现在没有发布者，先订阅，出现后自动开始记录："
  echo "       ${DEAD[*]}"
fi
if [ ${#TOPICS[@]} -eq 0 ]; then
  echo "ERROR: 一个话题都没有" >&2
  exit 1
fi

# 话题在列表里、甚至有发布者，也不代表真有数据。
# Astra 驱动尤其坑：订阅者全部断开后它会 stopStream，下次有人订阅时不一定
# 重新开流 —— 连录两次，第二个包只有 camera_info、一张图都没有。
# 所以 REQUIRED 的每一路都真收一条消息，收不到就不开录。
NO_DATA=()
for t in "${REQUIRED[@]}"; do
  if timeout 6 rostopic echo -n1 --noarr "$t" >/dev/null 2>&1; then
    echo "  OK  $t 有数据"
  else
    NO_DATA+=("$t")
  fi
done
if [ ${#NO_DATA[@]} -gt 0 ]; then
  if [ "$ALLOW_MISSING" -eq 1 ]; then
    echo "  WARN 这些话题 6 秒内没有收到任何消息：${NO_DATA[*]}"
  else
    echo "ERROR: 这些话题 6 秒内没有收到任何消息：${NO_DATA[*]}" >&2
    echo "  话题存在但没有数据，录下来会是空文件。常见原因：" >&2
    echo "  * 相机：Astra 驱动在订阅者断开后会停流且不自恢复 ——" >&2
    echo "    pkill -f astrapro.launch 后重新 roslaunch astra_camera astrapro.launch" >&2
    echo "  * 雷达：/dev/rplidar 没建出来（USB 松了），插好后 ls -l /dev/rplidar" >&2
    echo "  确实要在缺数据的情况下录，加 -A / --allow-missing。" >&2
    exit 1
  fi
fi

# ---------- 汇总 ----------
echo
echo "保存目录 : $OUT_DIR（剩余 ${AVAIL_GB} GB）"
echo "模式     : $PROFILE"
echo "时长     : $([ "$DURATION" -gt 0 ] 2>/dev/null && echo "${DURATION}s" || echo "一直录，Ctrl-C 停")"
echo "压缩     : lz4，每 ${SPLIT_MB} MB 分卷"
[ "$MAX_SPLITS" -gt 0 ] 2>/dev/null && echo "保留卷数 : 最新 ${MAX_SPLITS} 卷（自动删旧）"
echo "话题     : ${TOPICS[*]}"
echo

if [ "$CHECK_ONLY" -eq 1 ]; then
  echo "== 检查完成，未录制（-c / --check）=="
  exit 0
fi

# ---------- 录制 ----------
if [ "${BAGTOOLS_SESSION_READY:-0}" != 1 ]; then
  OUT_DIR=$(mktemp -d "$OUT_DIR/session_$(date +%Y%m%d_%H%M%S)_XXXXXX")
fi
echo "本次会话目录：$OUT_DIR"
STAMP=$(date +%Y%m%d_%H%M%S)
BAG="$OUT_DIR/cam_${STAMP}.bag"
printf '%s\n' "$CLOCK_REPORT" > "$OUT_DIR/cam_$STAMP.clock.json"

ARGS=(--lz4 --split --size="$SPLIT_MB")
[ "$MAX_SPLITS" -gt 0 ] 2>/dev/null && ARGS+=(--max-splits="$MAX_SPLITS")

# 注意：不能用 rosbag 自己的 --duration，它和 --split 一起用会变成
# “每卷录 N 秒”而不是“总共录 N 秒”。这里自己计时，到点发 SIGINT，
# 让 rosbag 正常收尾。必须等它真正退出，否则会留下 .bag.active（不可读）。
BAG_PID=""
RUNTIME_DIR=$(mktemp -d /dev/shm/bagtools-record.XXXXXX)
HEALTH_PATH="$RUNTIME_DIR/health.json"
GUARD_PID=""
HEALTH_PID=""
RC=255
STOP_REQUESTED=0
STOP_TIMEOUT=0
STOP_REASON=natural
stop_record() {
  python3 "$SCRIPT_DIR/session_status.py" --phase finalizing --health "$HEALTH_PATH"
  STOP_REQUESTED=1
  [ -z "$BAG_PID" ] && return 0
  kill -0 "$BAG_PID" 2>/dev/null || return 0
  # rosbag 在 Melodic 里是“父进程 + 录制子进程”结构。必须把 SIGINT 发给
  # 子进程：子进程会正常收尾；发给父进程会触发它自身的 bug
  # （TypeError: 'int' object is not callable），留下未关闭的 .bag.active。
  local kids
  kids=$(pgrep -P "$BAG_PID" 2>/dev/null || true)
  if [ -n "$kids" ]; then
    for c in $kids; do kill -INT "$c" 2>/dev/null; done
  else
    kill -INT "$BAG_PID" 2>/dev/null
  fi
  # 最多等 30 秒收尾
  for _ in $(seq 1 300); do
    kill -0 "$BAG_PID" 2>/dev/null || return 0
    sleep 0.1
  done
  echo 'ERROR: 录制尚未完成收尾；保留进程和 .active 文件，不强杀。' >&2
  STOP_TIMEOUT=1
  RC=124
  return 1
}
finalize_record() {
  python3 "$SCRIPT_DIR/session_status.py" --phase finalizing --health "$HEALTH_PATH"
  if [ -n "$HEALTH_PID" ]; then
    kill -TERM "$HEALTH_PID" 2>/dev/null || true
    wait "$HEALTH_PID" 2>/dev/null || true
    HEALTH_PID=""
  fi
  for pair in "health.json:health.json" "guard.txt:guard" "guard.json:guard.json"; do
    src="${pair%%:*}"; dst="${pair#*:}"
    [ ! -f "$RUNTIME_DIR/$src" ] || cp -- "$RUNTIME_DIR/$src" "$OUT_DIR/cam_$STAMP.$dst"
  done
  shopt -s nullglob
  local complete=("$OUT_DIR"/cam_"$STAMP"*.bag)
  local active=("$OUT_DIR"/cam_"$STAMP"*.bag.active)
  local result=0
  python3 "$SCRIPT_DIR/session_finalize.py" "$OUT_DIR" "$STAMP" "$PROFILE" \
    "$RC" "$STOP_REQUESTED" "$STOP_TIMEOUT" "$STOP_REASON" || result=$?
  rm -f -- "$RUNTIME_DIR/health.json" "$RUNTIME_DIR/health.json.tmp" "$RUNTIME_DIR/guard.txt" "$RUNTIME_DIR/guard.json" "$RUNTIME_DIR/guard.json.tmp"
  rmdir -- "$RUNTIME_DIR" 2>/dev/null || true
  echo "=== 会话收尾：完成 ${#complete[@]} 卷，未关闭 ${#active[@]} 卷 ==="
  if [ "$result" -ne 0 ]; then echo 'ERROR: 会话不完整，保留全部原文件。' >&2; fi
  return "$result"
}
cancel_record() {
  trap '' INT TERM
  case "$STOP_REASON" in
    clock_jump|disk_floor|guard_failed|startup_timeout|early_exit|stream_fault|health_observer_failed) ;;
    *) STOP_REASON=requested ;;
  esac
  if [ -n "$GUARD_PID" ]; then kill -TERM "$GUARD_PID" 2>/dev/null || true; fi
  if stop_record; then
    set +e
    wait "$BAG_PID" 2>/dev/null
    RC=$?
    set -e
  fi
  local result=0
  finalize_record || result=$?
  exit "$result"
}
trap cancel_record INT TERM

rosbag record -O "$BAG" "${ARGS[@]}" "${TOPICS[@]}" &
BAG_PID=$!
python2 "$SCRIPT_DIR/live_health.py" "$PROFILE" "$HEALTH_PATH" \
  --timeout "${STREAM_TIMEOUT_SEC:-3}" > "$OUT_DIR/cam_$STAMP.health.log" 2>&1 &
HEALTH_PID=$!
python3 "$SCRIPT_DIR/session_status.py" --phase recording --health "$HEALTH_PATH"
HEALTH_ARGS=(--health "$HEALTH_PATH" --evidence "$RUNTIME_DIR/guard.json")
if [ "$ALLOW_MISSING" -eq 1 ]; then HEALTH_ARGS+=(--diagnostic); fi

python3 "$SCRIPT_DIR/record_guard.py" "$BAG_PID" "$OUT_DIR/cam_$STAMP" \
  "$DURATION" "$MIN_FREE_GB" "${HEALTH_ARGS[@]}" > "$RUNTIME_DIR/guard.txt" &
GUARD_PID=$!
set +e
wait "$GUARD_PID"
GUARD_RC=$?
set -e
GUARD_PID=""
STOP_REASON=$(cat "$RUNTIME_DIR/guard.txt")
if [ "$GUARD_RC" -ne 0 ]; then STOP_REASON=guard_failed; fi
case "$STOP_REASON" in
  process_exit) ;;
  *) stop_record || { finalize_record || true; exit 1; } ;;
esac

set +e
wait "$BAG_PID" 2>/dev/null
RC=$?
set -e
# The recorder has exited; a completed earlier split cannot hide an active tail.
finalize_record
