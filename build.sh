#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# Default parallel job count: half of the CPU count from nproc(1) (logical
# processors), minimum 1. nproc is in coreutils and avoids parsing lscpu.
default_parallel_workers() {
    local cores half
    cores=$(nproc 2>/dev/null || echo 2)
    half=$((cores / 2))
    if [[ ${half} -lt 1 ]]; then
        half=1
    fi
    echo "${half}"
}

usage() {
    cat <<EOF
Usage: $(basename "$0") [options]

Options:
  -c, --clean              Remove install/, build/, log/ before building (clean build).
      --build-type <T>     CMake configuration: release (default), info, or debug.
                           Maps to Release, RelWithDebInfo, or Debug respectively.
  -j, --parallel <N>       Number of parallel colcon workers (positive integer).
                           Default: half of the CPU count from nproc(1) (minimum 1).
  -h, --help               Show this help message and exit.

With no options, performs an incremental colcon build with build type release.
EOF
}

clean=0
build_type="release"
parallel_workers=""

while [[ $# -gt 0 ]]; do
    case "$1" in
    --clean | -c)
        clean=1
        shift
        ;;
    --build-type)
        shift
        if [[ $# -eq 0 ]]; then
            echo "[build.sh] --build-type requires a value (release, info, or debug)." >&2
            exit 1
        fi
        build_type="${1}"
        shift
        ;;
    --build-type=*)
        build_type="${1#*=}"
        shift
        ;;
    -j | --parallel)
        shift
        if [[ $# -eq 0 ]]; then
            echo "[build.sh] -j / --parallel requires a positive integer (e.g. -j 8)." >&2
            exit 1
        fi
        parallel_workers="${1}"
        shift
        ;;
    --parallel=*)
        parallel_workers="${1#*=}"
        shift
        ;;
    -j*)
        parallel_workers="${1#-j}"
        if [[ -z ${parallel_workers} ]]; then
            echo "[build.sh] -j requires a positive integer (e.g. -j 8)." >&2
            exit 1
        fi
        shift
        ;;
    --help | -h)
        usage
        exit 0
        ;;
    *)
        echo "[build.sh] Unknown argument: ${1}" >&2
        usage >&2
        exit 1
        ;;
    esac
done

if [[ -z ${ROS_DISTRO:-} ]]; then
    echo "[build.sh] ROS_DISTRO is not set. Source your ROS environment first." >&2
    # shellcheck disable=SC2016  # show literal ${ROS_DISTRO} in the suggested command
    echo '[build.sh] Example: source /opt/ros/${ROS_DISTRO}/setup.bash' >&2
    exit 1
fi

case "${build_type}" in
release)
    cmake_build_type="Release"
    ;;
info)
    cmake_build_type="RelWithDebInfo"
    ;;
debug)
    cmake_build_type="Debug"
    ;;
*)
    echo "[build.sh] Invalid --build-type '${build_type}'. Use release, info, or debug." >&2
    exit 1
    ;;
esac

if [[ -z ${parallel_workers} ]]; then
    parallel_workers="$(default_parallel_workers)"
fi

if ! [[ ${parallel_workers} =~ ^[1-9][0-9]*$ ]]; then
    echo "[build.sh] Parallel worker count must be a positive integer, got: '${parallel_workers}'" >&2
    exit 1
fi

if [[ ${clean} -eq 1 ]]; then
    echo "[build.sh] Clean build: removing install/, build/, log/"
    rm -rf "${SCRIPT_DIR}/install" "${SCRIPT_DIR}/build" "${SCRIPT_DIR}/log"
fi

echo "[build.sh] CMAKE_BUILD_TYPE=${cmake_build_type}"
echo "[build.sh] parallel workers=${parallel_workers}"

# bagwiz keeps its package.xml at the workspace root rather than under
# src/. colcon stops recursing once it identifies a package, so
# --base-paths is limited to this directory.
colcon build \
    --symlink-install \
    --parallel-workers "${parallel_workers}" \
    --base-paths "${SCRIPT_DIR}" \
    --packages-up-to bagwiz \
    --cmake-args "-DCMAKE_BUILD_TYPE=${cmake_build_type}" -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
