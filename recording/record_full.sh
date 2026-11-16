#!/usr/bin/env bash
# record_full.sh -- 机器人上一键“起全量数据源 + 录全量 bag”。
#
# 全量 = 底盘 + Astra 彩色 + 深度 + 雷达 + 里程计 + IMU + TF
#
# 用法（在机器人上跑）：
#   ./record_full.sh                     # 起栈，一直录，Ctrl-C 停
#   ./record_full.sh -d 60               # 起栈，录 60 秒
#   ./record_full.sh -d 60 -o /home/jetson/bags
#   ./record_full.sh -d 60 -n 走廊建图    # 会话目录带标签
#   ./record_full.sh -d 60 --no-launch   # 栈已经在跑，只录
#   ./record_full.sh --launch-only       # 只起栈，不录
#
# 环境变量：
#   OUT_DIR=...  MIN_FREE_GB=5  SPLIT_MB=2048
#   ROS_IP=192.168.1.11        # 跨机器访问时必须，否则控制机收不到数据
#
# 注意：
#   * 一定要在机器人本地跑，控制机走 WiFi 录不了原始图像。
#   * 录一分钟约 1.2 GB；机器人只剩 11 GB，别一次录太久。
set -euo pipefail

DURATION="${DURATION:-0}"
OUT_DIR="${OUT_DIR:-$HOME/bags}"
SESSION_NAME="${SESSION_NAME:-}"
NO_LAUNCH=0
LAUNCH_ONLY=0
ALLOW_MISSING=0

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RECORDER="$SCRIPT_DIR/auto_record_cam.sh"
if [ -z "${BAGTOOLS_CALIBRATION:-}" ] && [ -f "$SCRIPT_DIR/calibration.json" ]; then
  export BAGTOOLS_CALIBRATION="$SCRIPT_DIR/calibration.json"
fi

usage() {
  # 同 remote_record.sh：按注释块截取，不写死行号
  sed -n '2,/^[^#]/p' "$0" | sed '$d' | sed 's/^# \{0,1\}//'
}

while [ $# -gt 0 ]; do
  case "$1" in
    -d|--duration) DURATION="$2"; shift 2 ;;
    -o|--out)      OUT_DIR="$2"; shift 2 ;;
    -n|--name)     [ $# -ge 2 ] || { echo "-n 需要一个名字" >&2; exit 2; }
                   SESSION_NAME="$2"; shift 2 ;;
    --no-launch)   NO_LAUNCH=1; shift ;;
    -A|--allow-missing) ALLOW_MISSING=1; shift ;;
    --launch-only) LAUNCH_ONLY=1; shift ;;
    -h|--help)     usage; exit 0 ;;
    *) echo "未知参数：$1" >&2; usage >&2; exit 2 ;;
  esac
done

# 标签只用来拼目录名，所以宁可保守：带路径分隔符或 `..` 就直接丢掉，
# 退回纯时间戳命名。正常路径上 GUI 已经清洗过一遍，这里是最后一道闸。
case "$SESSION_NAME" in
  */*|*..*|*\\*) SESSION_NAME='' ;;
esac

if [ ! -x "$RECORDER" ]; then
  echo "ERROR: 找不到 $RECORDER" >&2
  exit 1
fi

# ---------- 单实例锁 ----------
# 2026-11-10 那次“双重录制”：起栈要 10~60 秒，这期间还没有 rosbag 进程，
# 用 pgrep 判断等于没判断，第二次点击照样起一份，两个 rosbag 各写一个包。
# 锁必须由 record_full.sh 自己拿（它从启动第一秒就存在），并且校验 PID 真的
# 是本脚本 —— 否则重启后 PID 被别的进程复用，会永远锁死。
LOCK=/tmp/transbot_record.pid
exec 9>/tmp/transbot_record.lock
if ! flock -n 9; then
  echo 'ERROR: 已有录制会话正在启动或录制' >&2
  exit 1
fi
if [ -f "$LOCK" ]; then
  OLD="$(cat "$LOCK" 2>/dev/null || true)"
  if [ -n "$OLD" ] && kill -0 "$OLD" 2>/dev/null \
     && tr '\0' ' ' < "/proc/$OLD/cmdline" 2>/dev/null | grep -q 'record_full.sh'; then
    echo "ERROR: 已经有一个录制会话在跑（PID $OLD），先停掉再开：" >&2
    ps -o pid=,etime=,cmd= -p "$OLD" 2>/dev/null | cut -c1-96 >&2
    echo "  停止：./remote_record.sh stop" >&2
    exit 1
  fi
  echo "（清理过期的锁：PID ${OLD:-?} 已经不在了）"
  rm -f "$LOCK"
fi
echo $$ > "$LOCK"
if pgrep -f '[/]rosbag/record.*-O|[r]osbag record.*-O' >/dev/null; then
  rm -f "$LOCK"
  echo 'ERROR: a recorder is still active or finalizing; stop it before starting another' >&2
  exit 1
fi
REC_PID=''
CHECK_PID=''
cleanup() { rm -f "$LOCK"; }
cancel() {
  trap '' INT TERM
  if [ -n "$CHECK_PID" ] && kill -0 "$CHECK_PID" 2>/dev/null; then
    kill -TERM -- "-$CHECK_PID" 2>/dev/null || true
    wait "$CHECK_PID" 2>/dev/null || true
  fi
  if [ -n "$REC_PID" ] && kill -0 "$REC_PID" 2>/dev/null; then
    kill -TERM "$REC_PID" 2>/dev/null || true
    wait "$REC_PID" || true
  fi
  exit 130
}
trap cleanup EXIT
trap cancel INT TERM

# ---------- ROS 环境 ----------
# ROS 的 setup.bash 里有未定义变量，set -u 会报错，先临时关掉
set +u
source /opt/ros/melodic/setup.bash
if [ -f "$HOME/software/transbot_library/devel/setup.bash" ]; then
  source "$HOME/software/transbot_library/devel/setup.bash" --extend
fi
if [ -f "$HOME/transbot_ws/devel/setup.bash" ]; then
  source "$HOME/transbot_ws/devel/setup.bash" --extend
fi
set -u
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
export ROS_IP="${ROS_IP:-192.168.1.11}"
unset ROS_HOSTNAME

# ---------- 自检 ----------
run_preflight() {
  # Sensor launchers may outlive preflight; only this wrapper owns the lock.
  setsid bash "$SCRIPT_DIR/preflight.sh" "$@" 9>&- &
  CHECK_PID=$!
  local result=0
  wait "$CHECK_PID" || result=$?
  CHECK_PID=''
  return "$result"
}
PREFLIGHT_OK=1
# 起栈、查深度对齐、补 base_link→camera_link 都在 preflight.sh 里。以前这里
# 自己判"话题在就跳过启动"，结果相机是手动起的、depth_align 没生效也照录，
# 事后才发现深度没对齐到彩色帧。
if [ "$NO_LAUNCH" -eq 0 ]; then
  # 不合格的部分 preflight 会自己重启（相机走 restart_camera.sh，底盘/雷达走
  # 我们的 launch），修完还不合格就中止 —— 宁可录不成，也别录一个事后才发现
  # 缺数据的包。
  if ! run_preflight --fix; then
    PREFLIGHT_OK=0
    if [ "$ALLOW_MISSING" -eq 0 ]; then
    echo "ERROR: 录制前自检没通过，先修好再录。" >&2
    exit 1
    fi
    echo 'WARNING: explicit --allow-missing; this recording is diagnostic, not qualified semantic data'
  fi
else
  echo "--no-launch：假定数据源已就绪，只做检查不重启"
  if ! run_preflight; then
    PREFLIGHT_OK=0
    if [ "$ALLOW_MISSING" -eq 0 ]; then
      echo 'ERROR: preflight failed; --no-launch does not bypass validation' >&2
      exit 1
    fi
    echo 'WARNING: explicit --allow-missing; failed preflight is not qualified'
  fi
fi

if [ "$LAUNCH_ONLY" -eq 1 ]; then
  echo "数据源已启动，未录制。"
  echo "  查看日志：tail -f /tmp/transbot_record.log"
  exit 0
fi

# ---------- 录制 ----------
# 会话目录名 = session_<时间戳>_<标签>。
#
# 以前是 mktemp -d 的 6 位随机尾巴（session_20261112_004052_MZgKPg），事后根本
# 认不出录的是哪一段。随机串本来是为了防同名，但同名只可能发生在同一秒里，
# 用「存在就加 _2」的 mkdir 循环同样原子，而且名字可读。
mkdir -p "$OUT_DIR"
STAMP="$(date +%Y%m%d_%H%M%S)"
BASE="$OUT_DIR/session_$STAMP"
[ -n "$SESSION_NAME" ] && BASE="${BASE}_$SESSION_NAME"
SESSION_DIR="$BASE"
_n=1
while ! mkdir "$SESSION_DIR" 2>/dev/null; do
  if [ ! -e "$SESSION_DIR" ]; then
    echo "ERROR: 建不了会话目录：$SESSION_DIR" >&2
    exit 1
  fi
  _n=$((_n + 1))
  if [ "$_n" -gt 99 ]; then
    echo "ERROR: 同一秒里的同名会话太多：$BASE" >&2
    exit 1
  fi
  SESSION_DIR="${BASE}_$_n"
done
OUT_DIR="$SESSION_DIR"
if [ -n "${BAGTOOLS_CALIBRATION:-}" ]; then
  cp -- "$BAGTOOLS_CALIBRATION" "$OUT_DIR/calibration.json"
fi
export BAGTOOLS_SESSION_READY=1
export BAGTOOLS_PREFLIGHT_OK="$PREFLIGHT_OK" BAGTOOLS_ALLOW_MISSING="$ALLOW_MISSING"
REC_EXTRA=()
[ "$ALLOW_MISSING" -eq 0 ] || REC_EXTRA=(-A)
echo "本次会话目录：$OUT_DIR"
echo
if [ "$DURATION" -gt 0 ] 2>/dev/null; then
  WAIT_SEC=60 MIN_FREE_GB="${MIN_FREE_GB:-5}" \
    "$RECORDER" "${REC_EXTRA[@]}" -p full -d "$DURATION" -o "$OUT_DIR" 9>&- &
else
  WAIT_SEC=60 MIN_FREE_GB="${MIN_FREE_GB:-5}" \
    "$RECORDER" "${REC_EXTRA[@]}" -p full -o "$OUT_DIR" 9>&- &
fi
REC_PID=$!
wait "$REC_PID"
REC_PID=''

# ---------- 顺手把地图存下来 ----------
# 2026-11-10 那批包里有 map->odom 却没有 /map，事后想叠地图一点料都没有。
# 录完趁 SLAM 还在跑，把栅格图存一份，下次就不用重新建。
if rostopic list 2>/dev/null | grep -qx /map; then
  MAPBASE="$OUT_DIR/map_$(date +%Y%m%d_%H%M%S)"
  if timeout 20 rosrun map_server map_saver -f "$MAPBASE" >/dev/null 2>&1 \
     && [ -f "$MAPBASE.pgm" ]; then
    echo "地图已保存：$MAPBASE.pgm + .yaml"
  else
    echo "WARNING: map_saver 失败，地图没存下来（bag 里的 /map 话题仍然在）"
  fi
else
  echo "（当前没有 /map 话题，跳过存图；只录数据不建图时属正常）"
fi

echo
echo "提示：数据源仍在后台运行。要停掉："
echo "  pkill -f '[r]oslaunch.*(transbot_record|record_sources).launch'"
