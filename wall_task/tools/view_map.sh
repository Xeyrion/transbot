#!/bin/bash
# ===========================================================================
# view_map.sh -- open a saved map (PGM + YAML) in rviz.
#
# Why: a .pgm is just a grayscale image.  The useful view is /map, which is
# what map_server publishes after reading the YAML.  This script starts a
# private roscore (port 11312) + map_server, publishes a tiny static frame so
# rviz has a "map" frame, then opens tools/view_map.rviz.  Nothing on the
# robot is touched.
#
# Usage:
#   ./view_map.sh                    # maps/fig8.yaml
#   ./view_map.sh lab_1106_new       # maps/lab_1106_new.yaml
#   ./view_map.sh maps/foo.yaml      # explicit file
#   ./view_map.sh --list             # show saved maps
#   ./view_map.sh --robot fig8       # use the robot's master (live /scan)
#   ./view_map.sh --config my.rviz   # use another rviz layout
#   ./view_map.sh --no-rviz fig8     # just check that the map loads
#
# The rviz layout (colors, layers, camera) is tools/view_map.rviz -- edit that
# file, not this script, if you want a different look.
# ===========================================================================

# No `set -u`: ROS's own setup.bash reads variables before setting them.
set +u

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
MAPS="$ROOT/maps"
RVIZ_CFG="$HERE/view_map.rviz"
PORT="${VIEW_MAP_PORT:-11312}"
MAP_ARG=""
USE_ROBOT=0
NO_RVIZ=0
LIST=0
STARTED_MASTER=0
MASTER_PID=""
MAP_PID=""
TF_PID=""

step() { printf '\n=== %s ===\n' "$*"; }
ok()   { printf '  ok    %s\n' "$*"; }
warn() { printf '  warn  %s\n' "$*"; }
die()  { printf '  FAIL  %s\n' "$*"; exit 1; }

usage() {
    sed -n '2,22p' "$0" | sed 's/^# \{0,1\}//'
}

while [ $# -gt 0 ]; do
    case "$1" in
        --list)    LIST=1; shift ;;
        --robot)   USE_ROBOT=1; shift ;;
        --no-rviz) NO_RVIZ=1; shift ;;
        --config)  RVIZ_CFG="$2"; shift 2 ;;
        --port)    PORT="$2"; shift 2 ;;
        -h|--help) usage; exit 0 ;;
        -*)        die "unknown option: $1 (try --help)" ;;
        *)         [ -z "$MAP_ARG" ] && MAP_ARG="$1" || die "only one map at a time"
                   shift ;;
    esac
done

# ------------------------------------------------------------- map lookup --
if [ "$LIST" = 1 ]; then
    printf 'maps in %s:\n' "$MAPS"
    shopt -s nullglob
    for y in "$MAPS"/*.yaml; do
        name="$(basename "$y" .yaml)"
        info="$(python3 - "$MAPS/$name.json" <<'PY' 2>/dev/null
import json, sys
try:
    m = json.load(open(sys.argv[1]))
except Exception:
    sys.exit(0)
bits = []
if 'size_m' in m:
    bits.append('%.1f x %.1f m' % tuple(m['size_m']))
if 'free_pct' in m:
    bits.append('free %.1f%%' % m['free_pct'])
if 'coverage_pct' in m:
    bits.append('cover %.1f%%' % m['coverage_pct'])
if 'saved' in m:
    bits.append(m['saved'])
print(', '.join(bits))
PY
)"
        printf '  %-16s %s\n' "$name" "${info:-$y}"
    done
    exit 0
fi

if [ -z "$MAP_ARG" ]; then
    MAP_ARG="fig8"
fi
if [ -f "$MAP_ARG" ]; then
    MAP_YAML="$(cd "$(dirname "$MAP_ARG")" && pwd)/$(basename "$MAP_ARG")"
elif [ -f "$MAP_ARG.yaml" ]; then
    MAP_YAML="$(cd "$(dirname "$MAP_ARG.yaml")" && pwd)/$(basename "$MAP_ARG.yaml")"
elif [ -f "$MAPS/$MAP_ARG.yaml" ]; then
    MAP_YAML="$MAPS/$MAP_ARG.yaml"
elif [ -f "$MAPS/$MAP_ARG" ]; then
    MAP_YAML="$MAPS/$MAP_ARG"
else
    die "map not found: $MAP_ARG  (try --list)"
fi
case "$MAP_YAML" in
    *.yaml|*.yml) ;;
    *.pgm)
        MAP_YAML="${MAP_YAML%.pgm}.yaml"
        [ -f "$MAP_YAML" ] || die "no YAML next to that PGM"
        ;;
    *) die "not a .yaml/.pgm map: $MAP_YAML" ;;
esac
MAP_DIR="$(dirname "$MAP_YAML")"
grep -q '^image:' "$MAP_YAML" || die "$MAP_YAML has no image: line"

# ------------------------------------------------------------- ROS setup --
step "ROS environment"
if [ -f "$ROOT/../console/env.sh" ]; then
    source "$ROOT/../console/env.sh" >/dev/null 2>&1
    ok "sourced console/env.sh"
elif [ -f /opt/ros/melodic/setup.bash ]; then
    source /opt/ros/melodic/setup.bash
    ok "sourced /opt/ros/melodic/setup.bash"
else
    die "ROS not found (no console/env.sh and no /opt/ros/melodic)"
fi
command -v roscore >/dev/null || die "roscore not on PATH"
command -v rviz    >/dev/null || die "rviz not on PATH"

probe_master() {
    local uri="$1" host port
    host="$(printf '%s' "$uri" | sed -E 's#.*://([^:/]+).*#\1#')"
    port="$(printf '%s' "$uri" | sed -nE 's#.*:([0-9]+).*#\1#p')"
    [ -n "$host" ] && timeout 1 bash -c "echo > /dev/tcp/$host/${port:-11311}" 2>/dev/null
}

cleanup() {
    trap - EXIT INT TERM
    # Ask nicely, then force it.  A plain `wait` has hung here when a node
    # ignores SIGTERM, so never block on children.
    [ -n "$TF_PID" ]  && kill "$TF_PID"  2>/dev/null
    [ -n "$MAP_PID" ] && kill "$MAP_PID" 2>/dev/null
    if [ "$STARTED_MASTER" = 1 ] && [ -n "$MASTER_PID" ]; then
        kill -- -"$MASTER_PID" 2>/dev/null
    fi
    sleep 1
    [ -n "$TF_PID" ]  && kill -9 "$TF_PID"  2>/dev/null
    [ -n "$MAP_PID" ] && kill -9 "$MAP_PID" 2>/dev/null
    if [ "$STARTED_MASTER" = 1 ] && [ -n "$MASTER_PID" ]; then
        kill -9 -- -"$MASTER_PID" 2>/dev/null
    fi
}
trap cleanup EXIT INT TERM

# --------------------------------------------------------------- 1. master --
step "1/3 ROS master"
if [ "$USE_ROBOT" = 1 ]; then
    [ -n "${ROS_MASTER_URI:-}" ] || die "ROS_MASTER_URI is not set"
    probe_master "$ROS_MASTER_URI" \
        || die "robot master $ROS_MASTER_URI is not reachable (drop --robot for offline view)"
    ok "using robot master $ROS_MASTER_URI"
else
    export ROS_MASTER_URI="http://127.0.0.1:$PORT"
    export ROS_IP="127.0.0.1"
    unset ROS_HOSTNAME
    if timeout 1 bash -c "echo > /dev/tcp/127.0.0.1/$PORT" 2>/dev/null; then
        ok "master already on :$PORT (will not stop it)"
    else
        : > /tmp/view_map_roscore.log
        setsid roscore -p "$PORT" > /tmp/view_map_roscore.log 2>&1 &
        MASTER_PID=$!
        disown "$MASTER_PID" 2>/dev/null
        STARTED_MASTER=1
        for _ in $(seq 1 50); do
            timeout 1 rosnode list >/dev/null 2>&1 && break
            sleep 0.2
        done
        timeout 1 rosnode list >/dev/null 2>&1 \
            || die "local roscore did not start, see /tmp/view_map_roscore.log"
        ok "local roscore on :$PORT (pid $MASTER_PID)"
    fi
fi

# ----------------------------------------------------------- 2. map_server --
step "2/3 map_server"
printf '  map     %s\n' "$MAP_YAML"
grep -E '^(image|resolution|origin|negate):' "$MAP_YAML" | sed 's/^/  /'

if rostopic list 2>/dev/null | grep -qx '/map'; then
    warn "a /map publisher already exists; rviz may show the other map too"
fi

: > /tmp/view_map_server.log
( cd "$MAP_DIR" && exec rosrun map_server map_server "$MAP_YAML" \
      __name:=view_map_server ) > /tmp/view_map_server.log 2>&1 &
MAP_PID=$!
disown "$MAP_PID" 2>/dev/null
for _ in $(seq 1 50); do
    if ! kill -0 "$MAP_PID" 2>/dev/null; then
        printf '%s\n' "--- /tmp/view_map_server.log ---"
        tail -n 20 /tmp/view_map_server.log
        die "map_server exited -- is the PGM next to the YAML?"
    fi
    rostopic type /map 2>/dev/null | grep -q '^nav_msgs/OccupancyGrid$' && break
    sleep 0.2
done
rostopic type /map 2>/dev/null | grep -q '^nav_msgs/OccupancyGrid$' \
    || die "no /map after 10 s, see /tmp/view_map_server.log"
ok "/map is up"

# rviz needs the fixed frame to exist in the TF tree.  A leaf edge is enough
# and does not conflict with the robot's own map->odom->base_link chain.
: > /tmp/view_map_tf.log
rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 map view_map_anchor \
    __name:=view_map_tf > /tmp/view_map_tf.log 2>&1 &
TF_PID=$!
disown "$TF_PID" 2>/dev/null
ok "static map frame published"

if [ "$NO_RVIZ" = 1 ]; then
    step "3/3 done (--no-rviz)"
    ok "map loads; stopping"
    exit 0
fi

# ------------------------------------------------------------------ 3. rviz --
step "3/3 rviz"
[ -n "${DISPLAY:-}" ] || die "no DISPLAY -- cannot open rviz from this shell"
[ -f "$RVIZ_CFG" ] || die "rviz layout not found: $RVIZ_CFG"
printf '  layout  %s\n\n' "$RVIZ_CFG"
rviz -d "$RVIZ_CFG"

step "cleanup"
ok "rviz closed; stopping map_server"
