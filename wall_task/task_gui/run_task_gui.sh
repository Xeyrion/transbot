#!/bin/bash
# Kept so older notes and muscle memory keep working.
# The real entry point is start_task.sh -- see the header there.
exec "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/start_task.sh" "$@"
