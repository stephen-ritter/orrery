#!/usr/bin/env bash
# Firmware provenance: refuse to fly a yard this tree did not build.
#
#   scripts/firmware.sh stamp   write the stamp (cfs_mission/build.sh does this
#                               at the end of every successful build)
#   scripts/firmware.sh check   refuse if the stamp disagrees (every launcher
#                               of container-start does this)
#   scripts/firmware.sh hash    print the provenance hash
#   scripts/firmware.sh inputs  print what the hash is taken over
#
# WHY. scripts/fleet.sh and the gate drivers start
# cfs_mission/mission/build/exe/cpu1/container-start - a binary that sits on
# disk whether or not it matches the sources that produced it. The registry
# digest baked into the firmware (OB) catches the one mismatch that is a
# REGISTRY change; a trajectory digest may or may not move for anything else.
# Neither says the plain sentence "you edited C and rebuilt nothing", and the
# cost of not saying it is an afternoon spent debugging flight software that is
# not the flight software in front of you. This says it before the container
# starts, not three minutes into a boot.
#
# The stamp is a CONTENT hash of the inputs, not an mtime comparison, and that
# distinction is the whole design: a `git checkout` rewrites mtimes without
# changing a byte, so an mtime guard cries wolf after every branch switch, gets
# ignored, and dies the way ignored guards die. Inputs are the tracked files
# under cfs_mission/ that the compiler reads - including the generated part
# headers, so a registry edit that was rendered counts as an input, and one that
# was NOT rendered is caught twice over, by the offline mirrors and by OB at boot
# - plus the cFS pin in cfs_mission/cfs.lock: the core is fetched, not vendored,
# so the pin is the fact that has to be recorded, and moving it is a deliberate
# act that must re-fleet the yard. Prose is not an input, and the lock file
# enters as that pin line rather than as bytes: see provenance_inputs().
#
# Escape hatch: ORRERY_ALLOW_STALE=1 flies anyway and SAYS SO on every launch,
# for the case where the guard is right and you still want the flight (an
# operator reproducing a bug against the binary they already have).
set -uo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
EXE_DIR="${EXE_DIR:-$REPO/cfs_mission/mission/build/exe/cpu1}"
STAMP_FILE="${STAMP_FILE:-$REPO/cfs_mission/mission/build/orrery_firmware.sha}"
LOCK="${CFS_LOCK:-$REPO/cfs_mission/cfs.lock}"
GIT=(git -C "$REPO" -c safe.directory='*')

die() {
    echo "firmware.sh: $*" >&2
    exit 1
}

# The pinned cFS core, read straight out of cfs_mission/cfs.lock. Not from git,
# and not by calling scripts/fetch_cfs.sh: the stamp is taken inside the build
# container and checked outside it, so the reader has to be a grep that works
# anywhere. The two readers of the pin agree by test
# (tests/test_firmware_guard.py), not by hope.
cfs_pin() {
    local pin
    pin=$(grep -m1 -E '^[[:space:]]*commit[[:space:]]*=' "$LOCK" 2>/dev/null |
        sed -E 's/^[^=]*=[[:space:]]*"([^"]*)".*/\1/')
    [ "${#pin}" -eq 40 ] ||
        die "cannot read a 40-character 'commit' from $LOCK - the cFS pin is unreadable"
    printf '%s\n' "$pin"
}

# What cFS is actually checked out on disk, when there is one. Empty if not.
cfs_checkout() {
    local dir
    dir=$(grep -m1 -E '^[[:space:]]*checkout[[:space:]]*=' "$LOCK" 2>/dev/null |
        sed -E 's/^[^=]*=[[:space:]]*"([^"]*)".*/\1/')
    [ -n "$dir" ] && [ -d "$REPO/$dir" ] || return 0
    git -C "$REPO/$dir" -c safe.directory='*' rev-parse HEAD 2>/dev/null || echo "unreadable"
}


# What the stamp is taken over: tracked files under cfs_mission/ that the
# compiler actually reads. Two exclusions, both deliberate and both tested:
#
#   *.md            prose is not an input. Documentation lives next to the code
#                   and gets edited far more often than it; grounding the whole
#                   yard every time a sentence improves would train people to set
#                   ORRERY_ALLOW_STALE, which is worse than no guard at all.
#   cfs.lock        the pin is not a file to hash, it is a FACT to name: it
#                   enters below as `cfs-pin <commit>`, so editing its prose
#                   cannot re-stamp the mission while moving its commit must.
#
# Narrowing the input set is the dangerous kind of edit to this file, because a
# guard that sees less keeps passing - so tests/test_firmware_guard.py pins both
# the exclusions and the inclusion of every header and source that changes the
# binary.
provenance_inputs() {
    # NUL-separated, because paths are data and a filename with a space is a real
    # thing that has broken build scripts before.
    (
        cd "$REPO" 2>/dev/null || die "cannot enter $REPO to list its inputs"
        "${GIT[@]}" ls-files -z -- cfs_mission 2>/dev/null |
            grep -zv -E '\.md$|^cfs_mission/cfs\.lock$' | LC_ALL=C sort -z
    )
}

# sha256 over "hash  path" lines in a fixed order, plus the cFS pin line.
provenance() {
    # cd into the repo, because `git ls-files` answers in paths relative to the
    # REPOSITORY ROOT while callers stand wherever they stand: the mission build
    # calls this from cfs_mission/, where "cfs_mission/mission/orrery_defs/..."
    # resolves to nothing and the guard - correctly - refused to hash a tree it
    # could not read. Hashing must mean the same thing from any directory, or it
    # means nothing: the stamp would differ per caller, which is a guard that
    # fails on Tuesdays for reasons nobody can name.
    (
        cd "$REPO" 2>/dev/null || die "cannot enter $REPO to hash it"
        local -a inputs=()
        mapfile -d '' -t inputs < <(provenance_inputs)
        [ "${#inputs[@]}" -gt 0 ] || die "no build inputs under cfs_mission/ - refusing to hash nothing"
        # Hash the inputs BEFORE piping them, so a failure aborts the stamp rather
        # than flowing into it: on the other side of a pipe, the digest is computed
        # from whatever did arrive and the pipeline's exit status belongs to `awk`.
        # That is how a guard reports a missing file and prints a hash anyway.
        local sums
        # -r: never run sha256sum with no arguments - it would read stdin, which
        # inside a command substitution is not a failure but a hang.
        sums=$(printf '%s\0' "${inputs[@]}" | xargs -0 -r sha256sum) ||
            die "a tracked cfs_mission file is missing from the tree - nothing trustworthy to hash"
        {
            printf '%s\n' "$sums"
            echo "cfs-pin $(cfs_pin)"
        } | sha256sum | awk '{print $1}'
    )
}

# The core on disk is allowed to disagree with the pin only until someone builds
# against it. Checked at launch because a swapped-out core is invisible in every
# other signal this yard produces: the binary still boots, and the mission stops
# reproducing somewhere else's machine.
check_core() {
    local got want
    want=$(cfs_pin) || return 1
    got=$(cfs_checkout)
    if [ -z "$got" ]; then
        echo "firmware: the cFS core is not fetched ($(lock_checkout))"
        echo "          fetch it:  scripts/fetch_cfs.sh"
        return 1
    fi
    if [ "$got" = "unreadable" ]; then
        echo "firmware: $(lock_checkout) exists but is not a readable git checkout"
        echo "          check it:  scripts/fetch_cfs.sh --verify"
        return 1
    fi
    if [ "$got" != "$want" ]; then
        echo "firmware: THE cFS CORE IS NOT THE PINNED ONE"
        echo "          on disk  ${got:0:12}"
        echo "          pinned   ${want:0:12} ($(lock_tag))"
        echo "          the core is pristine by rule: scripts/fetch_cfs.sh --verify names"
        echo "          the difference, and moves the pin only when that is the decision."
        return 1
    fi
    return 0
}

lock_checkout() {
    grep -m1 -E '^[[:space:]]*checkout[[:space:]]*=' "$LOCK" 2>/dev/null |
        sed -E 's/^[^=]*=[[:space:]]*"([^"]*)".*/\1/'
}

lock_tag() {
    grep -m1 -E '^[[:space:]]*tag[[:space:]]*=' "$LOCK" 2>/dev/null |
        sed -E 's/^[^=]*=[[:space:]]*"([^"]*)".*/\1/'
}


cmd=${1:-check}
case "$cmd" in
    hash)
        provenance
        ;;
    inputs)
        # What the stamp is taken over, one path per line. A verb rather than a
        # comment: the safety of a narrowed input set depends on being inspectable
        # (tests/test_firmware_guard.py asks this script, not a human).
        provenance_inputs | tr '\0' '\n'
        ;;
    stamp)
        # Refuse to bless a build of the wrong core: the stamp is a promise about
        # what was compiled, and the pin is part of what was compiled.
        check_core || exit 1
        hash=$(provenance) || exit 1
        mkdir -p "$(dirname "$STAMP_FILE")"
        printf '%s\n' "$hash" >"$STAMP_FILE" || die "cannot write $STAMP_FILE"
        echo "firmware: stamped provenance ${hash:0:12} (cFS $(cfs_pin | cut -c1-7))"
        ;;
    check)
        if [ "${ORRERY_ALLOW_STALE:-0}" = 1 ]; then
            echo "firmware: STALE FLYING - ORRERY_ALLOW_STALE=1, provenance not checked"
            exit 0
        fi
        want=$(provenance) || exit 1
        if [ ! -x "$EXE_DIR/core-cpu1" ]; then
            echo "firmware: nothing built at $EXE_DIR"
            echo "          build it:  docker run --rm --user \"\$(id -u):\$(id -g)\" -e HOME=/tmp \\"
            echo "                       -v \"$REPO\":/work orrery-cfs:dev \\"
            echo "                       bash -c \"cd /work/cfs_mission && ./build.sh all\""
            exit 1
        fi
        if [ ! -f "$STAMP_FILE" ]; then
            echo "firmware: no provenance stamp at $STAMP_FILE"
            echo "          the binary predates this guard (or the build tree was partly"
            echo "          wiped). Rebuild once - build.sh stamps from now on - or fly it"
            echo "          knowingly with ORRERY_ALLOW_STALE=1."
            exit 1
        fi
        got=$(head -n1 "$STAMP_FILE")
        if [ "$got" != "$want" ]; then
            echo "firmware: THE MISSION ON DISK WAS NOT BUILT FROM THIS TREE"
            echo "          stamped $got"
            echo "          this tree $want"
            echo "          rebuild it (build.sh stamps), or ORRERY_ALLOW_STALE=1 to fly"
            echo "          the old binary knowingly."
            exit 1
        fi
        check_core || exit 1
        echo "firmware: provenance OK ${want:0:12} (cFS $(cfs_pin | cut -c1-7))"
        ;;
    *)
        die "unknown verb '$cmd' (stamp, check, hash, inputs)"
        ;;
esac
