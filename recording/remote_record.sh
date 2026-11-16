#!/usr/bin/env bash
# remote_record.sh -- 在控制机上远程控制机器人的全量 bag 录制，并随时查看状态。
#
# 解决三个问题：
#   1. 录制在机器人本地跑（不占 WiFi），控制机只发指令
#   2. 随时能看状态：在不在录、录了多大、磁盘还剩多少、日志有没有报错
#   3. 能远程停，且是“优雅停止”（rosbag 正常收尾，不会留下坏文件）
#
# 用法（在控制机上）：
#   ./remote_record.sh start -d 300          # 起栈并录 5 分钟
#   ./remote_record.sh status                # 看状态
#   ./remote_record.sh log                   # 跟踪日志（Ctrl-C 退出，不影响录制）
#   ./remote_record.sh stop                  # 优雅停止并打印 bag 信息
#   ./remote_record.sh pull                  # 拉回 + MD5 校验
#   ./remote_record.sh pull -d               # 拉回 + 校验 + 删机器人上的
#
# 可选：
#   -o/--out DIR        机器人上的保存目录（默认 ~/bags）
#   -d/--duration SEC   录制秒数（默认 300）
#   -n/--name NAME      会话目录里的标签（留空则只有时间戳）
#   ROBOT=user@host     默认 jetson@192.168.1.11
set -euo pipefail

ROBOT="${ROBOT:-jetson@192.168.1.11}"
ROBOT_PASS="${ROBOT_PASS:-yahboom}"
DURATION="${DURATION:-300}"
OUT_DIR="${OUT_DIR:-/home/jetson/bags}"
SESSION_NAME="${SESSION_NAME:-}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 打印文件开头那段注释当帮助。别写死行号 —— 以前是 sed -n '2,22p'，往头部加
# 一行说明就会把 `set -euo pipefail` 一起打进帮助里，而且没人会发现。
usage() { sed -n '2,/^[^#]/p' "$0" | sed '$d' | sed 's/^# \{0,1\}//'; }

need_sshpass() {
  if command -v sshpass >/dev/null 2>&1; then
    SSH=(sshpass -p "$ROBOT_PASS" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=8 "$ROBOT")
  else
    SSH=(ssh -o StrictHostKeyChecking=no -o ConnectTimeout=8 "$ROBOT")
  fi
}
need_sshpass

cmd="${1:-}"; [ $# -gt 0 ] && shift || true

# -d/-o 只对 start 有意义。pull 后面的 -d 是“传完删除”，要原样传给 pull_bags.sh，
# 不能在这里被吃掉（以前 pull -d 会因此报 unbound variable）。
if [ "$cmd" = "start" ]; then
  while [ $# -gt 0 ]; do
    case "$1" in
      -d|--duration) [ $# -ge 2 ] || { echo "-d 需要一个秒数" >&2; exit 2; }
                     DURATION="$2"; shift 2 ;;
      -o|--out)      [ $# -ge 2 ] || { echo "-o 需要一个目录" >&2; exit 2; }
                     OUT_DIR="$2"; shift 2 ;;
      -n|--name)     [ $# -ge 2 ] || { echo "-n 需要一个名字" >&2; exit 2; }
                     SESSION_NAME="$2"; shift 2 ;;
      *) break ;;
    esac
  done
fi

# 名字最终会拼进远端命令行（heredoc 没加引号，本地先展开），所以必须只留
# 字母数字、汉字、点、下划线、连字符 —— 一个 `$` 或 `;` 就能让远端 shell 干别的。
# 清洗规则只在 session_catalog.session_label() 里写一份，这里借来用；
# 没有 python3 就退回纯时间戳命名，不因为一个可选功能把 start 弄挂。
if [ -n "$SESSION_NAME" ]; then
  SESSION_NAME="$(python3 -c '
import sys
sys.path.insert(0, sys.argv[1])
from session_catalog import session_label
print(session_label(sys.argv[2]))
' "$(dirname "$SCRIPT_DIR")" "$SESSION_NAME" 2>/dev/null || true)"
fi
NAME_ARG=''
[ -n "$SESSION_NAME" ] && NAME_ARG="-n $SESSION_NAME"

case "$cmd" in
  start)
    CODE=$(python3 -c 'import base64,sys; print(base64.b64encode(open(sys.argv[1],"rb").read()).decode("ascii"))' "$SCRIPT_DIR/remote_start.py")
    python3 "$SCRIPT_DIR/recorder_bundle.py" --duration "$DURATION" --out "$OUT_DIR" --name "$SESSION_NAME" --root "${ROBOT_HOME:-/home/jetson}" \
      | "${SSH[@]}" "python3 -c 'import base64; exec(base64.b64decode(\"$CODE\"))'"
    ;;

  status)
    "${SSH[@]}" 'python3 /home/jetson/session_status.py'

    ;;

  stop)
    echo "== 优雅停止 =="
    "${SSH[@]}" 'python3 /home/jetson/stop_recording.py'

    ;;

  log)
    echo "== 实时日志（Ctrl-C 只看不退录制）=="
    "${SSH[@]}" "tail -f /tmp/record_full_console.log"
    ;;

  pull)
    # 不 exec pull_bags.sh：从 Windows 同步过来的脚本通常没有执行位（SFTP 只搬
    # 内容不搬 mode，落盘是 umask 002 的 0664），直接执行会得到 126 Permission
    # denied。这里用 python3 显式解释执行，并补上 shim 原本负责的 ROBOT_PASS ——
    # pull_session.py 靠它决定是否套 sshpass，缺了会退化成交互式密码提示而失败。
    exec env ROBOT_PASS="$ROBOT_PASS" python3 "$SCRIPT_DIR/pull_session.py" "$@"
    ;;

  -h|--help|"")
    usage
    ;;

  *)
    echo "未知命令：$cmd" >&2
    usage >&2
    exit 2
    ;;
esac
