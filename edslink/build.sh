#!/usr/bin/env bash
# Build the EdsLib runtime database + CPython binding for this mission's EDS tree.
#
# The result is what lets the Basilisk side speak the mission's own schema instead
# of a second hand-written one (ROADMAP D6):
#
#   dist/orrery_eds_db.so   EDS compiled to a runtime database (EDSLIB_ORRERY_*)
#   dist/EdsLib.so             CPython binding: import EdsLib; EdsLib.Database("...")
#
# The database must match the interpreter that imports it, so the Python headers are
# taken from the project venv (override with EDSLINK_PYTHON=/path/to/python).
# The database is dlopen()ed by name, so callers need this directory on the module
# path and as the working directory or in LD_LIBRARY_PATH (see README.md).
#
# usage: build.sh [check|prep|build|install|all]
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$HERE/.."
BUILD_DIR="${EDSLINK_BUILD_DIR:-$HERE/build}"
DIST="${EDSLINK_DIST:-$HERE/dist}"
PY="${EDSLINK_PYTHON:-$REPO/.venv/bin/python}"
JOBS="${JOBS:-$(nproc)}"
# Set EDSLINK_NO_PYTHON=1 for a flight-build-only run (the EdsLib runtime archive +
# the compiled database); it skips everything that needs a CPython to build against.
NO_PYTHON="${EDSLINK_NO_PYTHON:-0}"

check() {
    local missing=0
    for tool in cmake make cc pkg-config; do
        command -v "$tool" >/dev/null || { echo "missing tool: $tool" >&2; missing=1; }
    done
    # A CMake cache belongs to the directory it was configured in, and this one
    # has TWO customers: the host (this script, with the venv's Python) and the
    # container's mission build via cfs_mission/build.sh, whose tree lives at
    # /work. When the container had to configure this directory itself, the host
    # build later died on cmake's own verdict - "The current CMakeCache.txt
    # directory ... is different than ..." - which names the symptom and hides
    # the one-command fix. Refuse earlier and say it plainly.
    if [[ -f "$BUILD_DIR/CMakeCache.txt" ]]; then
        local home
        home=$(sed -n 's/^# For build in directory: //p' "$BUILD_DIR/CMakeCache.txt")
        if [[ -n "$home" && "$home" != "$BUILD_DIR" ]]; then
            echo "edslink/build.sh: $BUILD_DIR holds a cache configured for '$home'." >&2
            echo "  That build ran in a different world - the mission build container," >&2
            echo "  which sees this tree as /work. Its artifacts are not reusable here." >&2
            echo "  fix: rm -rf $BUILD_DIR   (then: bash edslink/build.sh)" >&2
            exit 1
        fi
    fi
    pkg-config --exists lua5.4 || pkg-config --exists lua-5.4 \
        || { echo "missing lua >= 5.3 headers (apt install liblua5.4-dev)" >&2; missing=1; }
    if [[ "$NO_PYTHON" != "1" ]]; then
        [[ -x "$PY" ]] || { echo "no interpreter at $PY (uv sync, or set EDSLINK_PYTHON)" >&2; missing=1; }
    fi
    [[ -f "$HERE/eds_dirs.cmake" ]] \
        || { echo "run: uv run python -m codegen.eds_sources --write" >&2; missing=1; }
    return "$missing"
}

py_include() {
    "$PY" -c 'import sysconfig; print(sysconfig.get_paths()["include"])'
}

prep() {
    if [[ "$NO_PYTHON" == "1" ]]; then
        cmake -S "$HERE" -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Release
        return
    fi
    # Extensions resolve interpreter symbols at load time, so no libpython is needed.
    cmake -S "$HERE" -B "$BUILD_DIR" \
        -DPYTHON_FOUND=TRUE \
        -DPYTHON_INCLUDE_DIRS="$(py_include)" \
        -DPYTHON_LIBRARIES="" \
        -DCMAKE_BUILD_TYPE=Release
}

build() {
    # The database archive + runtime archive are what the flight software links; the
    # python module is what the simulator imports. Both come from one EDS tree.
    cmake --build "$BUILD_DIR" --target edstool-execute-orrery -j "$JOBS" \
        && cmake --build "$BUILD_DIR" --target edslib_runtime_static -j "$JOBS" \
        && { [[ "$NO_PYTHON" == "1" ]] \
            || cmake --build "$BUILD_DIR" --target edslib_python_module -j "$JOBS"; }
}

install() {
    if [[ "$NO_PYTHON" == "1" ]]; then
        echo "flight artifacts: $BUILD_DIR/obj/orrery_eds_db.a"
        echo "                  $BUILD_DIR/eds/edslib/fsw/libedslib_runtime_static.a"
        echo "                  $BUILD_DIR/inc/*_eds_defines.h"
        return
    fi
    mkdir -p "$DIST"
    cp "$BUILD_DIR/obj/orrery_eds_db.so" "$DIST/"
    cp "$BUILD_DIR/eds/edslib/python/EdsLib.so" "$DIST/"
    # verify the artifacts THIS run produced, at the path THIS run was told
    # to write: with EDSLINK_DIST pointing elsewhere (a container building its
    # own binding), the bare default would import the tree's host-CPython .so
    # and report a symbol error about binaries nobody just built
    "$PY" "$HERE/verify.py" "$DIST"
    echo "dist: $DIST"
}

case "${1:-all}" in
    check)   check ;;
    prep)    check && prep ;;
    build)   build ;;
    install) install ;;
    all)     check && prep && build && install ;;
    *)       echo "usage: $0 [check|prep|build|install|all]" >&2; exit 2 ;;
esac
