#!/usr/bin/env bash
# Resumable SSH receiver: .part + SHA-256 + manifest + batch deletion.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
export ROBOT_PASS="${ROBOT_PASS:-yahboom}"
exec python3 "$HERE/pull_session.py" "$@"
