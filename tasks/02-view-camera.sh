#!/bin/bash
# 02-view-camera: rviz view for RGB + depth cloud + laser + TF (Fixed Frame: base_link).
# Prereq: robot camera stack is up.
DIR="$(cd "$(dirname "$0")" && pwd)"
source "$DIR/../env.sh" || exit 1
exec rviz -d "$DIR/../rviz/camera.rviz"
