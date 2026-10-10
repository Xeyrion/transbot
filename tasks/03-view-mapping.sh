#!/bin/bash
# 03-view-mapping: rviz view for mapping/navigation (Fixed Frame: map, Map+laser preset).
# Prereq: gmapping or map_server running on the robot side.
# Usage: bash tasks/03-view-mapping.sh
DIR="$(cd "$(dirname "$0")" && pwd)"
source "$DIR/../env.sh" || exit 1
exec rviz -d "$DIR/../rviz/mapping.rviz"
