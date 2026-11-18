#!/bin/bash
# English interface -- same launcher, same preflight, just --lang en.
# Handy for demoing to an English-speaking audience.
exec "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/start_task.sh" --lang en "$@"
