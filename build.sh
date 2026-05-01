#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

usage() {
    cat <<EOF
Usage: $(basename "$0") [options]

Options:
  -c, --clean   Remove install/, build/, log/ before building (clean build).
  -h, --help    Show this help message and exit.

With no options, performs an incremental colcon build.
EOF
}

clean=0
for arg in "$@"; do
    case "${arg}" in
    --clean | -c) clean=1 ;;
    --help | -h)
        usage
        exit 0
        ;;
    *)
        echo "[build.sh] Unknown argument: ${arg}" >&2
        usage >&2
        exit 1
        ;;
    esac
done

if [[ ${clean} -eq 1 ]]; then
    echo "[build.sh] Clean build: removing install/, build/, log/"
    rm -rf "${SCRIPT_DIR}/install" "${SCRIPT_DIR}/build" "${SCRIPT_DIR}/log"
    # The current shell may have sourced a previous install/setup.bash, leaving
    # AMENT/CMAKE/COLCON_PREFIX_PATH pointing at directories we just deleted.
    # colcon would then emit a flood of "path doesn't exist" warnings. Drop
    # those entries so the clean build starts from a clean environment.
    unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
fi

# Discover bundled message packages under dependencies/ so they are built
# alongside bagwiz. The directory is populated by setup.bash via vcs
# import; if it is missing or empty, only bagwiz and its standard deps
# are built.
deps_pkgs=()
if [[ -d "${SCRIPT_DIR}/dependencies" ]]; then
    while IFS= read -r pkg; do
        [[ -n ${pkg} ]] && deps_pkgs+=("${pkg}")
    done < <(colcon list --names-only --base-paths "${SCRIPT_DIR}/dependencies" 2>/dev/null || true)
fi

# bagwiz keeps its package.xml at the workspace root rather than under
# src/. colcon stops recursing once it identifies a package, so a default
# `--base-paths .` only finds bagwiz and ignores anything under
# dependencies/. Pass the dependency tree as a separate base path so its
# packages get discovered and become eligible for --packages-up-to.
colcon build \
    --symlink-install \
    --base-paths "${SCRIPT_DIR}" "${SCRIPT_DIR}/dependencies" \
    --packages-up-to bagwiz ${deps_pkgs[@]+"${deps_pkgs[@]}"} \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
