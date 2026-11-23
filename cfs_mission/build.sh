#!/usr/bin/env bash
# orrery mission build wrapper.
#
# Builds a cFS mission whose defs and apps live ENTIRELY outside the cFS core:
#   -DMISSION_DEFS=<mission>/orrery_defs   (defs dir outside MISSION_SOURCE_DIR)
#   CFS_APP_PATH=<mission>                    (extra module search path; finds apps/*)
# so the core stays pristine, which is the rule the pin exists to keep: cFS is
# fetched from cfs_mission/cfs.lock by scripts/fetch_cfs.sh, never vendored, and
# nothing of ours is ever written inside it. Run inside orrery-cfs:dev
# where the repo is mounted at /work - and run the CONTAINER as the host user
# (`docker run --user $(id -u):$(id -g)`): launchers boot the mission with that
# same --user, so a root-owned build tree aborts PSP boot on 'open: Permission
# denied'. (Host precondition: cFS SB pipes want fs.mqueue.msg_max >= 32; an
# unprivileged OSAL silently truncates to the sysctl, which only a root can
# raise via /etc/sysctl.d - pipes at depth 10 still fly lockstep just fine.)
#
# usage: build.sh [prep|build|install|all|run]
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CFS_DIR="${CFS_DIR:-$HERE/cFS}"
MISSION_DIR="$HERE/mission"
BUILD_DIR="${BUILD_DIR:-$MISSION_DIR/build}"
JOBS="${JOBS:-$(nproc)}"
EDSLINK_BUILD_DIR="${EDSLINK_BUILD_DIR:-$HERE/../edslink/build}"

# The link schema (ROADMAP D6): bsk_link decodes with the EdsLib database that
# edslink/ compiles from the same XML the ground defs come from. It needs host
# cmake/gcc/Lua, all present in the build image; the result is a plain static
# archive, so it is built once and reused by the mission build.
# The core is fetched, not vendored (cfs_mission/cfs.lock). Without this check a
# mission build with no core fails inside cmake as "source directory … does not
# exist", which names the wrong thing and sends the reader to the build system
# when the answer is one command. Same sentinel the lock lists for a finished
# fetch, so "cmake can start" and "the fetch went the whole way" are one fact.
require_core() {
    if [[ ! -f "$CFS_DIR/cfe/modules/core_api/fsw/inc/cfe.h" ]]; then
        echo "build.sh: no cFS core at $CFS_DIR" >&2
        echo "          fetch the pinned one:  scripts/fetch_cfs.sh" >&2
        exit 1
    fi
}

edslink_prep() {
    if [[ ! -f "$EDSLINK_BUILD_DIR/obj/orrery_eds_db.a" ]]; then
        # NOTE: edslink/build.sh dispatches on $1 alone - "prep build" as one
        # call silently runs only prep (cost a confusing build once). Chain
        # the calls explicitly.
        EDSLINK_BUILD_DIR="$EDSLINK_BUILD_DIR" EDSLINK_NO_PYTHON=1 \
            bash "$HERE/../edslink/build.sh" prep \
            && EDSLINK_BUILD_DIR="$EDSLINK_BUILD_DIR" EDSLINK_NO_PYTHON=1 \
                bash "$HERE/../edslink/build.sh" build
    fi
    [[ -f "$EDSLINK_BUILD_DIR/eds/edslib/fsw/libedslib_runtime_static.a" ]] \
        || EDSLINK_BUILD_DIR="$EDSLINK_BUILD_DIR" EDSLINK_NO_PYTHON=1 \
            bash "$HERE/../edslink/build.sh" build
}

prep() {
    edslink_prep
    # search-path entries are PARENTS of module directories
    export CFS_APP_PATH="$MISSION_DIR/apps"
    # NOTE: CFE_EDS_ENABLED=false deliberately. The in-tree EDS build (IC-era
    # EdsLib) encodes packet headers AND payloads with host little-endian
    # bit-packing (verified on the wire: EVS payloads arrive byte-swapped),
    # which is not CCSDS ground format. EDS XML remains the definition source
    # of truth: codegen/eds2cosmos derives the COSMOS defs + gs_tcp MsgId
    # table from it, and the non-EDS runtime emits identical MsgIds
    # (base|topic == EDS topic mapping; golden tests pin this) plus the
    # classic big-endian headers ground systems expect. The Basilisk *link*
    # is the exception: it is not a CCSDS ground packet, so bsk_link decodes
    # it straight through the EdsLib runtime database above.
    cmake -S "$CFS_DIR/cfe" -B "$BUILD_DIR" \
        -DMISSIONCONFIG=orrery \
        -DMISSION_DEFS="$MISSION_DIR/orrery_defs" \
        -DORRERY_EDSLINK_BUILD="$EDSLINK_BUILD_DIR" \
        -DSIMULATION=native \
        -DENABLE_UNIT_TESTS=FALSE \
        -DCFE_EDS_ENABLED="${CFE_EDS_ENABLED:-false}" \
        -DCMAKE_BUILD_TYPE=debug \
        -DCMAKE_INSTALL_PREFIX=/exe
}

build()   { cmake --build "$BUILD_DIR" --target mission-all -j "$JOBS"; }
install() { DESTDIR="$BUILD_DIR" cmake --build "$BUILD_DIR" --target mission-install; }
run()     { cd "$BUILD_DIR/exe/cpu1" && ./container-start; }

# Provenance: record the exact tree this binary came from, so every launcher can
# refuse to fly a yard something else built (scripts/firmware.sh). Stamping is
# part of a successful build rather than a separate chore - a stamp nobody
# remembers to take is a stamp that quietly goes stale, which is the failure
# this guard exists to close.
stamp() { bash "$HERE/../scripts/firmware.sh" stamp; }

verb="${1:-all}"
# Anything that compiles needs the core; `run` does not (it launches what is
# already built, and refusing to bring a yard DOWN over a missing core is the
# same rule scripts/fleet.sh follows for provenance).
case "$verb" in prep | build | install | all) require_core ;; esac

case "$verb" in
    prep)    prep ;;
    build)   build && stamp ;;
    install) install && stamp ;;
    run)     run ;;
    all)     prep && build && install && stamp ;;
    *)       echo "usage: $0 [prep|build|install|all|run]" >&2; exit 2 ;;
esac
