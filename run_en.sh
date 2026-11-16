#!/usr/bin/env bash
# English demo entry; uses exactly the same code and data directories.
set -e
export BAGTOOLS_LANG=en
HERE="$(cd "$(dirname "$0")" && pwd)"
exec bash "$HERE/run.sh" "$@"
