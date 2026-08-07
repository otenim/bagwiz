#!/usr/bin/env bash
# bagwiz-build.sh - build bagwiz via colcon, picking the {core,full} profile from
# --core and the CPU/CUDA profile from the active pixi environment. Invoked by the
# pixi build tasks (build / build-full; see pixi.toml); not meant to be run on
# its own, since it relies on the pixi environment's activated ROS 2 + conda
# toolchain.
#
# "full" builds add the `map` command group (in-process GLIM SLAM); "core" builds
# omit it. `bagwiz map slam` links the GLIM stack (GTSAM + gtsam_points + glim),
# which the full builds compile once into a vendored prefix via
# scripts/build-glim-deps.sh before compiling bagwiz against it. The CPU/CUDA choice
# is derived from the environment name: a *-cuda env (humble-cuda/jazzy-cuda) builds
# CUDA, any other env builds CPU. The profiles:
#
#   * build (--core) in a CPU env        -> core bagwiz, NO `map`/SLAM, no GLIM
#                                           stack. The fast build. Any distro.
#   * build (--core) in a *-cuda env     -> same core binary (core links no CUDA),
#                                           built into the *-cuda env's base; the
#                                           symmetric core entry of the cpu/cuda
#                                           matrix. humble-cuda/jazzy-cuda.
#   * build-full in a CPU env            -> CPU SLAM (-DBAGWIZ_WITH_SLAM=ON), after
#                                           building the GLIM CPU deps. humble/jazzy.
#   * build-full in a *-cuda env         -> CUDA SLAM (-DBAGWIZ_WITH_SLAM_CUDA=ON),
#                                           after building the GLIM CPU + CUDA deps.
#                                           humble-cuda/jazzy-cuda.
#
# Each pixi environment builds into its OWN base (build/<env>, install/<env>) keyed
# on $PIXI_ENVIRONMENT_NAME, so switching `-e <env>` never reuses another env's
# colcon/CMake cache (which would silently link the wrong ROS or CUDA libraries).
# build and build-full SHARE a base (e.g. install/humble), so every profile
# passes BAGWIZ_WITH_SLAM/_CUDA/_MAP_VIEWER explicitly: that forces the CMake cache
# to the intended profile and stops a prior full build from leaving `map` compiled
# into a later core build (or vice versa). The trailing symlink keeps
# build/compile_commands.json pointing at the last-built env for editor tooling.
#
# Build speed comes from two transparent layers below, both no-ops for the produced
# binaries: ccache reuses compiled objects across rebuilds, distros and git
# worktrees, and Ninja drives the per-package builds. A package configured with a
# different generator in the past (e.g. an old Unix Makefiles cache) has its build
# dir wiped and reconfigured automatically instead of aborting the colcon run.
#
# Intentionally NOT pixi-input-cached: the bundled GLIM step is already idempotent
# via its own stamp (build-glim-deps.sh), and an `inputs` cache keyed on src/ alone
# would report a false "cache hit" and skip the whole task even when the GLIM prefix
# is missing (e.g. after `pixi run clean`), leaving the SLAM build to fail at
# find_package(glim). colcon's own incremental build keeps a no-change rebuild fast.
set -euo pipefail

# First positional (anything not starting with `-`) is the CMake build type;
# when no positional is given, BAGWIZ_BUILD_TYPE applies, then the Release
# default. Flags: --core selects the core (no `map`/SLAM) profile; the CPU/CUDA
# profile comes from the active env, not a flag. --parallel-workers N caps
# colcon's parallel package builds; when neither the flag nor
# BAGWIZ_BUILD_PARALLELISM is set, the default computed below applies.
build_type="${BAGWIZ_BUILD_TYPE:-Release}"
core=0
parallel_workers="${BAGWIZ_BUILD_PARALLELISM:-}"
while [ $# -gt 0 ]; do
    case "$1" in
    --core) core=1 ;;
    --parallel-workers)
        if [ $# -lt 2 ]; then
            echo "bagwiz-build: --parallel-workers needs a value" >&2
            exit 2
        fi
        parallel_workers="$2"
        shift
        ;;
    -*)
        echo "bagwiz-build: unknown flag '$1' (expected --core or --parallel-workers N)" >&2
        exit 2
        ;;
    *) build_type="$1" ;;
    esac
    shift
done
case "${parallel_workers}" in
"") ;; # unset: the half-physical-cores default below applies
*[!0-9]* | 0)
    echo "bagwiz-build: --parallel-workers must be a positive integer (got '${parallel_workers}')" >&2
    exit 2
    ;;
esac

if [ -z "${parallel_workers}" ]; then
    # Default: half the PHYSICAL cores, min 1. colcon's own default (all logical
    # cores) puts one Ninja package build on every SMT sibling, and the
    # template-heavy ROS/Eigen translation units are memory-hungry, so full SMT
    # oversubscription mostly buys peak RAM, not wall-clock speed. Unique
    # (core, socket) pairs from lscpu keep SMT siblings out of the count; fall
    # back to nproc (logical) when lscpu is missing or unparseable. `|| true`
    # keeps an empty lscpu output (grep exits 1, and pipefail propagates it)
    # from aborting the script: wc still prints 0, caught by the case below.
    physical="$(lscpu -b -p=CORE,SOCKET 2>/dev/null | grep -v '^#' | sort -u | wc -l || true)"
    core_kind="physical"
    case "${physical}" in
    '' | *[!0-9]* | 0)
        physical="$(nproc 2>/dev/null || echo 2)"
        core_kind="logical (lscpu unavailable)"
        ;;
    esac
    parallel_workers=$((physical / 2))
    if [ "${parallel_workers}" -lt 1 ]; then
        parallel_workers=1
    fi
    echo "bagwiz-build: --parallel-workers defaults to ${parallel_workers} (half of ${physical} ${core_kind} cores)"
fi

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO="${PIXI_PROJECT_ROOT:-$(cd -- "${SCRIPT_DIR}/.." && pwd)}"
ENV_NAME="${PIXI_ENVIRONMENT_NAME:-humble}"

# CPU/CUDA profile is derived from the env name, not a flag: a *-cuda env builds
# CUDA, any other env builds CPU. Fail fast here (before the slow GLIM build) if a
# *-cuda env is missing its toolkit, since nvcc comes from the `gpu` feature that
# only the *-cuda environments carry.
cuda=0
if [ "${ENV_NAME%-cuda}" != "${ENV_NAME}" ]; then
    if [ ! -x "${CONDA_PREFIX:-/no-conda}/bin/nvcc" ]; then
        echo "bagwiz-build: environment '${ENV_NAME}' looks like a CUDA environment but nvcc was not found." >&2
        echo "  Make sure you are running in a *-cuda pixi environment, e.g.:" >&2
        echo "    pixi run -e humble-cuda build-full      # or jazzy-cuda" >&2
        exit 1
    fi
    cuda=1
fi

# Resolve whether to compile the `map`/SLAM command group:
#   slam=1 -> full build (map present);  slam=0 -> core build (no map).
# --core forces a core build anywhere; otherwise a full build compiles SLAM.
slam=1
if [ "${core}" -eq 1 ]; then
    slam=0
fi

cd "${REPO}"

# Compile through ccache when it is on PATH (the pixi env provides it). CMake
# initializes each language's launcher from these env vars, so no CMakeLists
# changes are needed and the exports propagate to the build-glim-deps.sh calls
# below. CCACHE_NOHASHDIR drops the build directory from ccache's hash so
# rebuilds hit across distros and git worktrees (Release builds carry no -g, so
# the working directory never lands in debug info). All four exports leave a
# value already set by the caller untouched.
if command -v ccache >/dev/null 2>&1; then
    export CMAKE_C_COMPILER_LAUNCHER="${CMAKE_C_COMPILER_LAUNCHER:-ccache}"
    export CMAKE_CXX_COMPILER_LAUNCHER="${CMAKE_CXX_COMPILER_LAUNCHER:-ccache}"
    export CMAKE_CUDA_COMPILER_LAUNCHER="${CMAKE_CUDA_COMPILER_LAUNCHER:-ccache}"
    export CCACHE_NOHASHDIR="${CCACHE_NOHASHDIR:-1}"
fi

# Prefer Ninja (from the pixi env) over Unix Makefiles: faster no-op and
# incremental builds. colcon invokes cmake without -G, so the standard
# CMAKE_GENERATOR env var selects the generator; without ninja the build falls
# back to Make as before.
if [ -z "${CMAKE_GENERATOR:-}" ] && command -v ninja >/dev/null 2>&1; then
    export CMAKE_GENERATOR=Ninja
fi

# A package configured with a different generator than the selected one (e.g. a
# Unix Makefiles cache predating the Ninja switch) makes CMake abort with
# "generator does not match", failing the whole colcon run. Wipe just those
# package build dirs - they are pure build artifacts that this run recreates.
# The depth-1 glob matches only the ament packages; the GLIM deps caches live
# deeper (glim-src*/<dep>/build/) and the glim-deps* prefixes under
# install/<env>/ are never touched.
if [ -n "${CMAKE_GENERATOR:-}" ]; then
    for cache in "build/${ENV_NAME}"/*/CMakeCache.txt; do
        [ -e "${cache}" ] || continue
        if ! grep -qxF "CMAKE_GENERATOR:INTERNAL=${CMAKE_GENERATOR}" "${cache}"; then
            echo "bagwiz-build: removing $(dirname "${cache}") (configured with a different generator; reconfiguring)"
            rm -rf "$(dirname "${cache}")"
        fi
    done
fi

cmake_args=(-DCMAKE_BUILD_TYPE="${build_type}" -DCMAKE_EXPORT_COMPILE_COMMANDS=ON)

if [ "${cuda}" -eq 1 ] && [ "${core}" -eq 1 ]; then
    # build in a *-cuda env: a core build that targets the *-cuda env's base
    # (install/<env>), giving the cpu/cuda matrix a symmetric core entry. It links
    # NO CUDA (core has no `map`/SLAM), so it is byte-identical to build in a
    # CPU env -- the only difference is the install base. The *-cuda env and its
    # nvcc were already validated above.
    #
    # Core profile (no map/SLAM/CUDA), explicitly forced so a prior full build in
    # this same base cannot leave `map` compiled in.
    cmake_args+=(
        -DBAGWIZ_WITH_SLAM=OFF
        -DBAGWIZ_WITH_SLAM_CUDA=OFF
        -DBAGWIZ_WITH_MAP_VIEWER=OFF
    )
elif [ "${cuda}" -eq 1 ]; then
    # build-full in a *-cuda env: CUDA SLAM (humble-cuda/jazzy-cuda). The *-cuda env
    # and its nvcc were already validated above; here we just need the conda C++
    # compiler on $CXX (the CUDA host compiler).
    : "${CONDA_PREFIX:?build-full needs an activated pixi env (CONDA_PREFIX unset)}"
    : "${CXX:?build-full needs the conda C++ compiler on \$CXX}"
    # GLIM CPU deps first (the CUDA stack reuses this prefix's GTSAM), then the
    # CUDA deps. Both are sub-second no-ops once their stamped prefixes exist.
    bash "${SCRIPT_DIR}/build-glim-deps.sh"
    bash "${SCRIPT_DIR}/build-glim-deps.sh" --cuda
    # Pass the CUDA toolchain through environment variables instead of -D flags.
    # colcon forwards --cmake-args to every package, so -DCMAKE_CUDA_COMPILER and
    # friends would trigger "Manually-specified variables were not used" warnings
    # in packages that do not enable the CUDA language. CMake recognises CUDACXX,
    # CUDAHOSTCXX and CUDAToolkit_ROOT, and only the package that calls
    # enable_language(CUDA) / find_package(CUDAToolkit) consumes them.
    export CUDACXX="${CONDA_PREFIX}/bin/nvcc"
    export CUDAHOSTCXX="${CXX}"
    export CUDAToolkit_ROOT="${CONDA_PREFIX}"
    cmake_args+=(
        -DBAGWIZ_WITH_SLAM_CUDA=ON
        -DBAGWIZ_WITH_MAP_VIEWER=ON
        "-DCMAKE_PREFIX_PATH=${REPO}/install/${ENV_NAME}/glim-deps-cuda;${REPO}/install/${ENV_NAME}/glim-deps"
        -DCMAKE_CUDA_ARCHITECTURES=86
    )
elif [ "${slam}" -eq 1 ]; then
    # build-full (CPU SLAM).
    bash "${SCRIPT_DIR}/build-glim-deps.sh"
    cmake_args+=(
        -DBAGWIZ_WITH_SLAM=ON
        -DBAGWIZ_WITH_SLAM_CUDA=OFF
        -DBAGWIZ_WITH_MAP_VIEWER=ON
        "-DCMAKE_PREFIX_PATH=${REPO}/install/${ENV_NAME}/glim-deps"
    )
else
    # build (CPU): core profile, no `map`/SLAM. Force the toggles OFF so a
    # prior full build in this same base cannot leave `map` compiled into this
    # core build.
    cmake_args+=(
        -DBAGWIZ_WITH_SLAM=OFF
        -DBAGWIZ_WITH_SLAM_CUDA=OFF
        -DBAGWIZ_WITH_MAP_VIEWER=OFF
    )
fi

colcon build --symlink-install --packages-up-to bagwiz \
    --build-base "build/${ENV_NAME}" --install-base "install/${ENV_NAME}" \
    --parallel-workers "${parallel_workers}" --cmake-args "${cmake_args[@]}"
ln -sfn "${ENV_NAME}/compile_commands.json" build/compile_commands.json
