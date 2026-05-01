#!/usr/bin/env bash
# setup.sh - Bootstrap the bagwiz workspace.
#
# Imports bundled message-package sources into ./dependencies via vcstool
# and installs ROS package dependencies via rosdep. Run after cloning the
# repository, and again whenever bagwiz.repos changes.
#
# bagwiz.repos enumerates every transitive repository directly (each
# pinned to a commit SHA), so a single `vcs import` is enough — no
# chained imports of nested .repos files.
#
# Despite the filename, this script is meant to be executed, not sourced.
#
# Requires:
#   - ROS_DISTRO sourced (e.g. `source /opt/ros/humble/setup.bash`)
#   - vcstool (`sudo apt install python3-vcstool`)
#   - rosdep (initialised with `sudo rosdep init && rosdep update`)

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
DEPS_DIR="${SCRIPT_DIR}/dependencies"
TOP_REPOS="${SCRIPT_DIR}/bagwiz.repos"

if [[ -z ${ROS_DISTRO:-} ]]; then
    echo "[setup.sh] ROS_DISTRO is not set. Source your ROS environment first." >&2
    exit 1
fi

for cmd in vcs rosdep; do
    if ! command -v "${cmd}" >/dev/null 2>&1; then
        echo "[setup.sh] '${cmd}' is not installed or not on PATH." >&2
        exit 1
    fi
done

mkdir -p "${DEPS_DIR}"

echo "[setup.sh] Importing dependencies from $(basename "${TOP_REPOS}")"
vcs import "${DEPS_DIR}" <"${TOP_REPOS}"

# Some upstream repos (nebula, oxts_ros2_driver) bundle many packages, but
# bagwiz only needs the message packages from each. Marking the unwanted
# packages with COLCON_IGNORE keeps colcon from descending into them and
# also avoids pulling in their transitive deps (e.g. sync_tooling_msgs in
# the case of nebula). Conversely, COLCON_IGNORE is removed for kept
# packages so re-running setup.sh with an expanded keep-list takes
# effect (the markers are untracked and survive vcs import).
restrict_to_packages() {
    local repo_dir="$1"
    shift
    local keep_pkgs=("$@")
    if [[ ! -d ${repo_dir} ]]; then
        return
    fi
    echo "[setup.sh] Restricting $(basename "${repo_dir}") to ${keep_pkgs[*]} via COLCON_IGNORE"
    while IFS= read -r -d '' pkg_xml; do
        local pkg_dir pkg_name keep
        pkg_dir="$(dirname "${pkg_xml}")"
        pkg_name="$(sed -nE 's|.*<name>([^<]+)</name>.*|\1|p' "${pkg_xml}" | head -n1)"
        keep=0
        for k in "${keep_pkgs[@]}"; do
            if [[ ${pkg_name} == "${k}" ]]; then
                keep=1
                break
            fi
        done
        if [[ ${keep} -eq 1 ]]; then
            rm -f "${pkg_dir}/COLCON_IGNORE"
        else
            touch "${pkg_dir}/COLCON_IGNORE"
        fi
    done < <(find "${repo_dir}" -name package.xml -not -path '*/.git/*' -print0)
}

restrict_to_packages "${DEPS_DIR}/nebula" \
    nebula_msgs pandar_msgs robosense_msgs continental_msgs continental_srvs
restrict_to_packages "${DEPS_DIR}/oxts_ros2_driver" oxts_msgs

echo "[setup.sh] Installing ROS package dependencies via rosdep (distro=${ROS_DISTRO})"
# Pass the dependency tree as a separate path because rosdep, like colcon,
# stops recursing once it identifies a package — so a single
# `--from-paths "${SCRIPT_DIR}"` only sees bagwiz at the workspace root and
# misses every package under dependencies/.
rosdep install \
    --from-paths "${SCRIPT_DIR}" "${DEPS_DIR}" \
    --ignore-src \
    --rosdistro "${ROS_DISTRO}" \
    -r -y

echo "[setup.sh] Done. Build with ./build.sh"
