#!/usr/bin/env bash
# The mission build, in the only form that works: inside the cFS image, as the
# host user, with the repo mounted at /work.
#
#   scripts/mission_build.sh [prep|build|install|all|run]   (default: all)
#
# Three things happen here, in this order, and each one is here because it was
# once missing:
#
#   1. the IMAGE exists. On a machine that has never built this project the tag
#      does not exist, and the old failure was `Unable to find image
#      'orrery-cfs:dev' locally` followed by a Docker Hub pull error — for an
#      image that is defined by a file in this repository. Building it from that
#      file, once, is cheaper than being a registry nobody keeps current.
#   2. the CORE exists. `scripts/fetch_cfs.sh` runs INSIDE the container, where
#      git can write the tree and where a fresh clone needs nothing but network:
#      `git clone` plus this one command produces flight software. It is
#      idempotent, so on a normal machine it costs half a second and one line of
#      log naming the core that will be compiled against.
#   3. the build itself, and its provenance stamp (`build.sh` ends by stamping a
#      content hash of its inputs; every launcher then demands that stamp).
#
# WHY A FILE AND NOT A LINE IN verify.sh. A check is executed as
# `timeout … bash -c "$cmd"`, so it runs in a child shell. Two attempts to make
# the build check a shell function failed there - rc=127 ("command not found",
# the function was invisible), then rc=125 (`docker: invalid spec: :/work`,
# because the function came with $REPO, not $REPO's value). A script is the same
# command a human types, and a script cannot be invisible to anything.
#
# Container ONLY. cfs_mission/mission/build/CMakeCache.txt remembers /work, so
# running cfs_mission/build.sh on the host fails cmake's source-directory check
# (harmlessly, but with no binary at the end).
#
# --user is not cosmetic: launchers boot the mission as the host user, so a
# root-owned build tree aborts PSP boot with 'open: Permission denied'.
# --hostname and USER are pinned for the same kind of reason they are NOT
# pinned in the launchers: cFE bakes its build-environment table into
# core-cpu1 - BUILDUSER and BUILDHOST are compiled-in strings, and the GNU
# build-id hashes them. Two containers therefore used to yield two binaries
# differing in exactly 30 bytes (measured: 20 build-id + 10 hostname) with
# identical physics, so "did my build change anything?" could not be answered
# by a digest. Pin both HERE, where they matter, and a build differs from any
# other build of the same tree ONLY in the build-minute cFE itself stamps
# (202611230642 vs ...0647 measured: 21 bytes) - two builds within the same
# minute are byte-identical, full stop. A digest diff much larger than a
# minute's worth is then a real change, which is the question being asked.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUNTIME="${RUNTIME:-docker}"
CFS_IMAGE="${CFS_IMAGE:-orrery-cfs:dev}"
VERB="${1:-all}"

case "$VERB" in
    prep | build | install | all | run) ;;
    *)
        echo "usage: $0 [prep|build|install|all|run]" >&2
        exit 2
        ;;
esac

if ! "$RUNTIME" info >/dev/null 2>&1; then
    echo "mission_build: $RUNTIME is not answering - the mission builds only inside $CFS_IMAGE" >&2
    exit 1
fi

if ! "$RUNTIME" image inspect "$CFS_IMAGE" >/dev/null 2>&1; then
    echo "mission_build: no $CFS_IMAGE here yet - building it from docker/cfs.Dockerfile"
    echo "               (once, a couple of minutes: scripts/image.sh)"
    CFS_IMAGE="$CFS_IMAGE" bash "$REPO/scripts/image.sh" cfs
fi

# The recorder sizes the flying scenario declared must be on disk before cmake
# sees the app that includes them (codegen/emit_store). A scenario's own header
# is left exactly as the yard door landed it; an absent one lands as the union
# default - no vehicle claims a recorder - because a quiet yard means the union,
# and a build that failed for a missing generated file teaches nothing about
# the flight software it never reached.
[ -f "$REPO/cfs_mission/mission/orrery_defs/inc/orrery_store.h" ] ||
    (cd "$REPO" && uv run python -m codegen.emit_store >/dev/null)

exec "$RUNTIME" run --rm --user "$(id -u):$(id -g)" -e HOME=/tmp \
    --hostname orrery-build -e USER=orrery-build \
    -v "$REPO":/work "$CFS_IMAGE" \
    bash -c 'set -euo pipefail
        cd /work
        scripts/fetch_cfs.sh
        # The container gets its OWN edslink cache (build-fsw). The one at
        # edslink/build belongs to the host - it carries the CPython binding
        # the tests import, and a CMake cache refuses to be reinterpreted from
        # a different path, so a container that configured it poisoned every
        # host build afterwards (and vice versa: measured, both directions).
        # Static archives rebuilt here cost seconds and are byte-identical;
        # edslink/build.sh now also refuses across worlds if anyone routes it
        # back into the shared directory by hand.
        export EDSLINK_BUILD_DIR=/work/edslink/build-fsw
        cd cfs_mission
        exec ./build.sh "$1"' _ "$VERB"
