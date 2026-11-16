#!/bin/bash
# ======================================================================
# deploy_to_robot.sh — 把录制脚本推到机器人上（在控制机上跑）。
#
# 录制是在机器人本地跑的，所以这几个文件必须放在机器人上：
#   record_full.sh      起栈 + 调录制
#   auto_record_cam.sh  真正调 rosbag 的那层
#   restart_camera.sh   相机停流后重启（Astra 的老毛病）
# 控制机上的副本只是源码，改了不推过去不生效。
#
#   ./deploy_to_robot.sh                 # 默认 jetson@192.168.1.11
#   ROBOT=jetson@10.0.0.5 ./deploy_to_robot.sh
# ======================================================================
set -euo pipefail

ROBOT="${ROBOT:-jetson@192.168.1.11}"
ROBOT_PASS="${ROBOT_PASS:-yahboom}"
DEST="${DEST:-/home/jetson}"
HERE="$(cd "$(dirname "$0")" && pwd)"
STAMP="$(date +%Y%m%d_%H%M%S)"
[[ "$DEST" =~ ^/[A-Za-z0-9_./-]+$ ]] || { echo 'ERROR: deployment DEST must be a simple absolute path' >&2; exit 2; }

SSH=(sshpass -p "$ROBOT_PASS" ssh -n -o StrictHostKeyChecking=no -o ConnectTimeout=8 "$ROBOT")
SCP=(sshpass -p "$ROBOT_PASS" scp -o StrictHostKeyChecking=no)

echo "== 机器人：$ROBOT   目标：$DEST =="
CHECK_CODE=$(python3 -c 'import base64,sys; print(base64.b64encode(open(sys.argv[1],"rb").read()).decode("ascii"))' "$HERE/remote_start.py")
"${SSH[@]}" "python3 -c 'import base64; exec(base64.b64decode(\"$CHECK_CODE\"))' --check-idle"
echo "--- 现状 ---"
"${SSH[@]}" "cd $DEST && ls -l record_full.sh auto_record_cam.sh restart_camera.sh 2>&1 | head -4"

echo "--- 备份 ---"
"${SSH[@]}" "cd $DEST && mkdir -p bagtools_archive/deployments/$STAMP && for f in record_full.sh auto_record_cam.sh restart_camera.sh preflight.sh stop_recording.py session_status.py session_finalize.py record_quality.py record_guard.py publish_calibration.py live_health.py stream_health.py record_probe.py calibration_schema.py recorder_bundle.py remote_start.py check_tf.py clock_check.py; do
  [ -f \$f ] && cp -p \$f bagtools_archive/deployments/$STAMP/ && echo \"  archived \$f\"
done; true"

echo "--- 上传 ---"
"${SCP[@]}" "$HERE/record_full.sh" "$HERE/auto_record_cam.sh" \
  "$HERE/restart_camera.sh" "$HERE/preflight.sh" "$HERE/stop_recording.py" "$HERE/session_status.py" \
  "$HERE/session_finalize.py" "$HERE/record_quality.py" "$HERE/record_guard.py" \
  "$HERE/publish_calibration.py" "$HERE/live_health.py" "$HERE/stream_health.py" "$HERE/record_probe.py" \
  "$HERE/../calibration_schema.py" "$HERE/recorder_bundle.py" "$HERE/remote_start.py" "$HERE/check_tf.py" "$HERE/clock_check.py" "$ROBOT:$DEST/"

echo "--- 校验 ---"
"${SSH[@]}" "cd $DEST && chmod +x record_full.sh auto_record_cam.sh restart_camera.sh preflight.sh \
  && bash -n record_full.sh && bash -n auto_record_cam.sh \
  && bash -n restart_camera.sh && bash -n preflight.sh \
  && python3 -m py_compile session_finalize.py record_guard.py publish_calibration.py record_probe.py calibration_schema.py recorder_bundle.py remote_start.py clock_check.py \
  && python2 -m py_compile record_quality.py live_health.py stream_health.py calibration_schema.py check_tf.py \
  && echo '  语法 OK' && ls -l record_full.sh preflight.sh restart_camera.sh"
LOCAL_HASH=$(python3 "$HERE/recorder_bundle.py" | python3 -c 'import sys,json,hashlib; print(hashlib.sha256(json.dumps(json.load(sys.stdin)["files"],sort_keys=True).encode()).hexdigest())')
REMOTE_HASH=$("${SSH[@]}" "cd $DEST && python3 recorder_bundle.py" | python3 -c 'import sys,json,hashlib; print(hashlib.sha256(json.dumps(json.load(sys.stdin)["files"],sort_keys=True).encode()).hexdigest())')
[ "$LOCAL_HASH" = "$REMOTE_HASH" ] || { echo 'ERROR: deployed recorder bundle checksum mismatch' >&2; exit 1; }

echo
echo "完成。录之前建议先跑一次干检查（只看话题在不在，不录）："
echo "  ssh $ROBOT '$DEST/auto_record_cam.sh -c -p full'"
