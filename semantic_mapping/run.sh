#!/usr/bin/env bash
# Unified workbench; optional --record / --review / data-directory shortcuts.
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
if ! command -v python3 >/dev/null 2>&1; then
  echo "缺少 python3" >&2; exit 1
fi
if ! python3 -c 'import tkinter' >/dev/null 2>&1; then
  echo "缺少 python3-tk：sudo apt install python3-tk" >&2; exit 1
fi
exec python3 "$HERE/workbench.py" "$@"
