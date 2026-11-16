#!/usr/bin/env bash
# preflight.sh -- 录制前自检：话题、数据、TF、相机配置逐项过一遍。
#
#   bash preflight.sh          # 只检查，报告
#   bash preflight.sh --fix    # 检查 + 自动重启不合格的部分，再复检
#
# 为什么需要：话题"存在"不等于"能用"。这次的包话题全在、数据也有，但深度没
# 对齐到彩色帧（depth_align 没生效），事后只能靠重投影补救。录之前查一遍，
# 比录完再发现便宜得多。
set -u

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -z "${BAGTOOLS_CALIBRATION:-}" ] && [ -f "$HERE/calibration.json" ]; then
  export BAGTOOLS_CALIBRATION="$HERE/calibration.json"
fi
FIX=0
[ "${1:-}" = "--fix" ] && FIX=1

set +u
source /opt/ros/melodic/setup.bash 2>/dev/null || true
source "$HOME/software/transbot_library/devel/setup.bash" --extend 2>/dev/null || true
source "$HOME/transbot_ws/devel/setup.bash" --extend 2>/dev/null || true
set -u
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
export ROS_IP="${ROS_IP:-192.168.1.11}"
unset ROS_HOSTNAME

FAILED=0
report() {  # report <ok|fail|warn> <说明>
  case "$1" in
    ok)   printf '  OK    %s\n' "$2" ;;
    fail) printf '  FAIL  %s\n' "$2"; FAILED=$((FAILED + 1)) ;;
    warn) printf '  WARN  %s\n' "$2" ;;
  esac
}

has_data() {  # 6 秒内真收到一条消息，不是只看话题在不在
  timeout 6 rostopic echo -n1 --noarr "$1" >/dev/null 2>&1
}

tf_ok() {
  timeout -k 1 5 python2 "$HERE/check_tf.py" "$1" "$2" >/dev/null 2>&1
}

calibration_ok() {
  timeout -k 1 7 python2 "$HERE/check_tf.py" --calibration "$BAGTOOLS_CALIBRATION"
}

frame_of() {
  # rostopic echo 会先打值再打一行 "---"，取 tail -1 会拿到分隔符
  timeout 6 rostopic echo -n1 "$1/header/frame_id" 2>/dev/null \
    | tr -d "'\" \r" | grep -v '^---$' | grep -v '^$' | head -1
}

depth_aligned() {
  [ "$(frame_of /camera/depth/image_raw)" = "camera_color_optical_frame" ]
}

# ---------- 一次性检查 ----------
check_all() {
  FAILED=0
  echo "== 自检 =="
  if ! timeout 3 rostopic list >/dev/null 2>&1; then
    report fail "ROS master 没起来（roscore）"
    return
  fi
  for t in /scan /odom; do
    if has_data "$t"; then report ok "$t 有数据"; else report fail "$t 没数据"; fi
  done
  for t in /camera/rgb/image_raw /camera/depth/image_raw /camera/rgb/camera_info /camera/depth/camera_info; do
    if has_data "$t"; then report ok "$t 有数据"; else report fail "$t 没数据"; fi
  done
  if has_data /camera/depth/image_raw; then
    if depth_aligned; then
      report ok "深度已对齐到彩色帧"
    else
      report warn "Unregistered depth: explicit intrinsics and optical TF are required for reprojection"
    fi
  fi
  for pair in "odom base_footprint" "base_footprint base_link" "base_link laser" "base_link camera_link"; do
    set -- $pair
    if tf_ok "$1" "$2"; then report ok "TF $1→$2"; else report fail "TF $1→$2 缺"; fi
  done
  if [ -n "${BAGTOOLS_CALIBRATION:-}" ]; then
    if calibration_ok; then
      report ok "Installation TF matches calibration; one publisher per configured edge"
    else
      report fail "Installation TF differs from calibration or has conflicting publishers"
    fi
  fi
  for t in /camera/rgb/image_raw /camera/depth/image_raw; do
    frame=$(frame_of "$t")
    if [ -n "$frame" ] && tf_ok base_link "$frame"; then
      report ok "Optical TF: base_link -> $frame"
    else
      report fail "Optical TF is missing for $t ($frame)"
    fi
  done
  if tf_ok map odom; then
    report ok "TF map→odom（有地图系）"
  else
    report warn "TF map→odom 缺（没开 SLAM；物体能投到里程计系，但会漂）"
  fi
}

# ---------- 自动修 ----------
fix_all() {
  echo "== 自动修复 =="
  # 底盘/雷达没数据：起我们自己的 launch（它同时带 depth_align 和相机 TF）
  local restart=0 camera_args=''
  if ! has_data /scan || ! has_data /odom; then restart=1; fi
  if [ -n "${BAGTOOLS_CALIBRATION:-}" ]; then
    camera_args=$(python3 "$HERE/publish_calibration.py" --camera-args "$BAGTOOLS_CALIBRATION") || return 1
    calibration_ok || restart=1
  fi
  if [ "$restart" -eq 1 ]; then
    echo '  Restart recording sources with installation calibration'
    for p in $(pgrep -f 'bin/roslaunch (transbot_nav transbot_recor[d].launch|[^ ]*/record_source[s].launch)' 2>/dev/null); do
      kill -TERM "$p" 2>/dev/null
    done
    sleep 4
    if [ -n "${BAGTOOLS_CALIBRATION:-}" ]; then
      if tf_ok base_link camera_link; then
        echo 'Another camera TF publisher is still running; stop its launch before applying calibration' >&2
        return 1
      fi
      setsid nohup roslaunch "$HERE/record_sources.launch" "camera_transform:=$camera_args" \
        > /tmp/transbot_record.log 2>&1 < /dev/null &
    else
      setsid nohup roslaunch transbot_nav transbot_record.launch \
        > /tmp/transbot_record.log 2>&1 < /dev/null &
    fi
    for _ in $(seq 1 60); do
      sleep 1
      has_data /scan && has_data /odom && break
    done
  fi
  # 相机没数据、或深度没对齐：restart_camera.sh 会用 depth_align:=true 重开
  if ! has_data /camera/rgb/image_raw || ! has_data /camera/depth/image_raw; then
    echo "  重启相机：astrapro.launch（depth_align:=true）"
    bash "$HERE/restart_camera.sh" || true
  fi
  # Camera installation TF belongs to the source launch, never a second publisher.
  if ! tf_ok base_link camera_link; then
    echo 'Missing installation TF: check the recording source launch and calibration.json' >&2
    return 1
  fi
}

echo '== 录前时钟检查（只读，不在运行中的 EKF 上跳时）=='
python3 "$HERE/clock_check.py" || exit 1
check_all
if [ "$FAILED" -gt 0 ] && [ "$FIX" -eq 1 ]; then
  fix_all || exit 1
  check_all
fi

if [ "$FAILED" -gt 0 ]; then
  echo
  echo "有 $FAILED 项不合格。TF 链路问题（odom→base_footprint、base_footprint→base_link）"
  echo "不是重启能解决的，通常是底盘驱动或 URDF 没起来，先查：rosrun tf view_frames"
  exit 1
fi
echo
echo "自检通过。"
