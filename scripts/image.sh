#!/usr/bin/env bash
# The images this repository builds and runs on — built here, not hunted for.
#
#   scripts/image.sh cfs             build orrery-cfs:dev if it is not there
#   scripts/image.sh basilisk        build orrery-bsk:dev (the Basilisk engine)
#   scripts/image.sh cfs --rebuild   build it again from the current Dockerfile
#   scripts/image.sh --list          what the repository expects, and what is here
#
# WHY. `scripts/mission_build.sh` and every launcher name `orrery-cfs:dev`, and
# on a machine that has never built it the first thing a newcomer meets is
# `Unable to find image ... locally` followed by a pull failure against Docker
# Hub — because that image exists only as a file in this repository. One command
# that makes it, called automatically by the build, turns "find the wiki page
# that tells you how to build the build image" into `git clone` plus one script.
#
# The image is intentionally not published either: it is a dozen apt packages and
# a working directory, so a rebuild from source is the same thing as a pull
# without anyone having to keep a registry current.
#
# Environment: CFS_IMAGE (the tag, same name the launchers use),
# RUNTIME (docker|podman), NO_CACHE=1.
set -uo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUNTIME="${RUNTIME:-docker}"
CFS_IMAGE="${CFS_IMAGE:-orrery-cfs:dev}"
BSK_IMAGE="${BSK_IMAGE:-orrery-bsk:dev}"
DOCKERFILE_DIR="$REPO/docker"

die() {
    echo "image.sh: $*" >&2
    exit 1
}

present() {
    "$RUNTIME" image inspect "$1" >/dev/null 2>&1
}

describe() {
    local tag=$1 size created
    size=$("$RUNTIME" image inspect --format '{{.Size}}' "$tag" 2>/dev/null || echo "")
    created=$("$RUNTIME" image inspect --format '{{.Created}}' "$tag" 2>/dev/null || echo "")
    # awk, not bc: this reports on a machine where nothing is installed yet
    printf '%s  %s MB  built %s' "$tag" \
        "$(awk -v s="${size:-0}" 'BEGIN { printf "%.0f", s / 1048576 }')" "${created:0:19}"
}

# The build context is docker/, not the repository: the Dockerfile COPYs nothing
# (the tree is mounted at run time), and pointing docker at the repo root would
# package a working tree that includes a fetched cFS core — hundreds of
# megabytes sent for no reason, every build.
build() {
    local tag=$1 dockerfile=$2 label=$3 started=$SECONDS
    echo "building $label ($tag) from ${dockerfile#"$REPO"/}"
    "$RUNTIME" build ${NO_CACHE:+--no-cache} -t "$tag" -f "$dockerfile" "$DOCKERFILE_DIR" ||
        die "image build failed: $dockerfile"
    echo "$label image ready in $((SECONDS - started))s"
}

list() {
    local line label tag dockerfile
    for line in "cfs|$CFS_IMAGE|cfs.Dockerfile" "basilisk|$BSK_IMAGE|basilisk.Dockerfile"; do
        IFS='|' read -r label tag dockerfile <<<"$line"
        if present "$tag"; then
            printf '  %-8s %s\n' "$label" "$(describe "$tag")"
        else
            printf '  %-8s %s  MISSING - build it: scripts/image.sh %s\n' "$label" "$tag" "$label"
        fi
    done
}

REBUILD=0
TARGET=""
for arg in "$@"; do
    case "$arg" in
    --rebuild) REBUILD=1 ;;
    --list)
        echo "images this repository expects:"
        list
        exit 0
        ;;
    -h | --help)
        cat <<'EOF'
image.sh — build the images this repository builds and runs on.

  scripts/image.sh cfs             build orrery-cfs:dev if it is not there
  scripts/image.sh basilisk        build orrery-bsk:dev (the Basilisk engine)
  scripts/image.sh cfs --rebuild   build it again from the current Dockerfile
  scripts/image.sh --list          what the repository expects, and what is here

Environment: CFS_IMAGE, BSK_IMAGE (tags), RUNTIME (docker|podman), NO_CACHE=1.
EOF
        exit 0
        ;;
    cfs | basilisk) TARGET=$arg ;;
    *) die "unknown argument '$arg' (cfs, basilisk, --rebuild, --list, --help)" ;;
    esac
done

[ -n "$TARGET" ] || TARGET=cfs
"$RUNTIME" info >/dev/null 2>&1 ||
    die "$RUNTIME is not answering - the images build inside $RUNTIME, start the daemon first"

case "$TARGET" in
    cfs) image=$CFS_IMAGE; dockerfile=cfs.Dockerfile; label="cfs mission" ;;
    basilisk) image=$BSK_IMAGE; dockerfile=basilisk.Dockerfile; label="basilisk engine" ;;
    *) die "no such image '$TARGET' (cfs, basilisk)" ;;
esac
if [ "$REBUILD" -eq 0 ] && present "$image"; then
    echo "$TARGET image already here: $(describe "$image")"
    echo "                      (--rebuild to build it again)"
    exit 0
fi
build "$image" "$DOCKERFILE_DIR/$dockerfile" "$label"
