#!/usr/bin/env bash
# Time `bagwiz movify` on a real bag across the layouts that stress different
# parts of the pipeline (JPEG decode, x264, point-cloud fetch/raster, the
# map panel). Each case runs REPEAT times back to back (warm page cache) and
# reports wall / user / peak RSS per run, so two binaries can be compared by
# interleaving invocations of this script (host drift dominates otherwise).
#
#   scripts/bench-movify.sh [-b <bagwiz binary>] [-n <repeat>] [-o <out dir>]
#                           [-c <case>...] <bag>
#
# The topic names below are the merged-corpus defaults (8x 4K JPEG cameras,
# 4 Seyond lidars, OxTS NavSatFix); override them through the environment
# (CAM, CAM2-4, LIDAR, LIDARS, GNSS, FRAME) for another bag; EXTRA appends
# flags to every case.
set -euo pipefail

BIN=${BIN:-bagwiz}
REPEAT=2
OUT=${OUT:-/tmp/bench-movify}
CASES=()
while getopts "b:n:o:c:" opt; do
    case $opt in
    b) BIN=$OPTARG ;;
    n) REPEAT=$OPTARG ;;
    o) OUT=$OPTARG ;;
    c) CASES+=("$OPTARG") ;;
    *)
        echo "usage: $0 [-b bagwiz] [-n repeat] [-o out_dir] [-c case]... <bag>" >&2
        exit 2
        ;;
    esac
done
shift $((OPTIND - 1))
[ $# -eq 1 ] || {
    echo "usage: $0 [-b bagwiz] [-n repeat] [-o out_dir] [-c case]... <bag>" >&2
    exit 2
}
BAG=$1
mkdir -p "$OUT"
# Extra movify flags appended to every case (e.g. EXTRA='--encoder x264').
EXTRA=${EXTRA:-}

CAM=${CAM:-/sensing/camera/camera0/image_raw/compressed}
CAM2=${CAM2:-/sensing/camera/camera1/image_raw/compressed}
CAM3=${CAM3:-/sensing/camera/camera2/image_raw/compressed}
CAM4=${CAM4:-/sensing/camera/camera3/image_raw/compressed}
LIDAR=${LIDAR:-/sensing/lidar/front/seyond_points}
LIDARS=${LIDARS:-'/sensing/lidar/*/seyond_points'}
GNSS=${GNSS:-/sensing/ins/oxts/nav_sat_fix}
FRAME=${FRAME:-base_link}

# name|args (the output path is appended)
ALL_CASES=(
    "cam4k|--cam $CAM --no-rectify"
    "cam4k_rect_overlay|--cam $CAM --cam-pcd $LIDAR"
    "cam4k_grid2x2|--cam $CAM $CAM2 $CAM3 $CAM4 --no-rectify"
    "pcd_3d|--pcd $LIDAR"
    "pcd4_bev|--pcd $LIDARS --frame $FRAME --view bev --range 80"
    "cam_pcd_gnss|--cam $CAM --no-rectify --pcd $LIDAR --gnss $GNSS --width 1920"
)

run_case() {
    local name=$1 args=$2
    for ((r = 1; r <= REPEAT; r++)); do
        local out=$OUT/$name.mp4
        # shellcheck disable=SC2086  # $args is a deliberate word list
        /usr/bin/time -f "$name run=$r wall=%e s user=%U s sys=%S s maxrss=%M KB" \
            "$BIN" movify -i "$BAG" $args $EXTRA -o "$out" -w >"$OUT/$name.log" 2>&1 || {
            echo "$name run=$r FAILED (see $OUT/$name.log)"
            return 1
        }
        tail -1 "$OUT/$name.log" | grep -q "wall=" && tail -1 "$OUT/$name.log"
        grep -o "wrote [0-9]* frame(s).*" "$OUT/$name.log" | tail -1 | sed "s/^/  /"
    done
}

for entry in "${ALL_CASES[@]}"; do
    name=${entry%%|*}
    args=${entry#*|}
    if [ ${#CASES[@]} -gt 0 ]; then
        keep=0
        for c in "${CASES[@]}"; do [ "$c" = "$name" ] && keep=1; done
        [ $keep -eq 1 ] || continue
    fi
    run_case "$name" "$args"
done
