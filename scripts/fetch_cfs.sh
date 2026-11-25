#!/usr/bin/env bash
# Fetch — or verify — the cFS core this mission builds against.
#
#   scripts/fetch_cfs.sh            fill in cfs_mission/cFS/ if it is not there
#   scripts/fetch_cfs.sh --verify   check what is there against the pin, touch nothing
#   scripts/fetch_cfs.sh --force    discard the checkout and fetch it again
#   scripts/fetch_cfs.sh --print    the pin, as key=value lines (for scripts)
#
# WHY THIS EXISTS. cFS is not part of this repository (cfs_mission/cfs.lock says
# why), but the mission cannot build without it, and a flight binary's identity
# includes the core underneath it. So the pin is committed and the source is
# fetched — which means "is the core here, and is it the core we think it is?"
# becomes a question with an answer instead of a ritual. Run this first, before
# any build; it is idempotent and needs nothing but git and network the first time.
#
# WHY --VERIFY MATTERS MORE THAN THE CLONE. This project's rule is that the cFS
# core is pristine: everything orrery adds lives in cfs_mission/mission/ and is
# grafted on at configure time. That rule is only true while nobody edits the
# core, and an edit to the core is easy to make by accident and nearly impossible
# to notice later — the build still works, the binary still boots, and the
# difference shows up as a mission that does not reproduce on someone else's
# machine. --verify is the tool that means it, and the checks run it.
set -uo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOCK="$REPO/cfs_mission/cfs.lock"
JOBS="${JOBS:-8}"

# One git invocation for the whole script. `safe.directory` because this runs
# both on the host and inside the build container, where the tree arrives as a
# bind mount: same paths, different owner, and git refuses to read a repository
# it thinks someone else owns. Refusing is right in general and wrong here, and
# "dubious ownership" is the least helpful way to fail a fetch check.
GIT=("${GIT_BIN:-git}" -c safe.directory='*')

die() {
    echo "fetch_cfs: $*" >&2
    exit 1
}

# --------------------------------------------------------------------------
# the pin
# --------------------------------------------------------------------------

# The lock is TOML with five interesting keys; read with grep rather than with
# Python, because this script runs before anything is installed. scripts/
# firmware.sh reads `commit` the same way: two readers of a five-line format is
# cheaper than a launch-time dependency between the fetcher and the guard, and
# tests/test_firmware_guard.py fails if the two ever disagree.
lock_value() {
    local key=$1 line
    line=$(grep -m1 -E "^[[:space:]]*${key}[[:space:]]*=" "$LOCK" 2>/dev/null) || return 1
    printf '%s\n' "$line" | sed -E 's/^[^=]*=[[:space:]]*"([^"]*)".*/\1/'
}

lock_paths() {
    sed -n '/^required_paths[[:space:]]*=/,/^\]/p' "$LOCK" | grep -oE '"[^"]+"' | tr -d '"'
}

[ -f "$LOCK" ] || die "no pin file at $LOCK (this script is the reason it must exist)"

PIN_REPO=$(lock_value repository)
PIN_TAG=$(lock_value tag)
PIN_COMMIT=$(lock_value commit)
CHECKOUT=$(lock_value checkout)

[ -n "$PIN_REPO" ] || die "$LOCK: no 'repository'"
case "$PIN_COMMIT" in
*[!0-9a-f]*) die "$LOCK: 'commit' is not a lowercase hex SHA ('$PIN_COMMIT')" ;;
esac
[ "${#PIN_COMMIT}" -eq 40 ] || die "$LOCK: 'commit' is ${#PIN_COMMIT} characters, not 40"
[ -n "$CHECKOUT" ] || die "$LOCK: no 'checkout' directory"

CORE="$REPO/$CHECKOUT"
short() {
    printf '%s' "${1:0:7}"
}

# --------------------------------------------------------------------------
# verbs
# --------------------------------------------------------------------------

print_pin() {
    printf 'repository=%s\ntag=%s\ncommit=%s\ncheckout=%s\n' \
        "$PIN_REPO" "$PIN_TAG" "$PIN_COMMIT" "$CHECKOUT"
}

# Every problem with the core, reported together: a half-listed diagnosis sends
# someone off to fix one thing, come back, and find the second.
verify() {
    local -a problems=()

    if [ ! -d "$CORE" ]; then
        echo "cFS is not fetched: $CORE"
        echo "          get it:  scripts/fetch_cfs.sh"
        return 1
    fi
    if ! "${GIT[@]}" -C "$CORE" rev-parse --git-dir >/dev/null 2>&1; then
        problems+=("$CORE exists but is not a git repository — it did not come from this script")
    else
        local got
        got=$("${GIT[@]}" -C "$CORE" rev-parse HEAD 2>/dev/null || echo "")
        if [ -z "$got" ]; then
            problems+=("$CORE has no readable HEAD (an interrupted fetch?)")
        elif [ "$got" != "$PIN_COMMIT" ]; then
            problems+=("core is $(short "$got"), the pin is $(short "$PIN_COMMIT") ($(describe_state "$got"))")
        fi
        local dirty
        dirty=$("${GIT[@]}" -C "$CORE" status --porcelain 2>/dev/null | head -20)
        if [ -n "$dirty" ]; then
            problems+=("core is MODIFIED — $(printf '%s\n' "$dirty" | wc -l) change(s), first few:
$(printf '%s\n' "$dirty" | sed 's/^/             /')")
        fi
        local drift
        drift=$("${GIT[@]}" -C "$CORE" submodule status --recursive 2>/dev/null |
            grep -c '^[+-U]' || true)
        [ "${drift:-0}" -eq 0 ] ||
            problems+=("$drift nested repositor(y/ies) not at the commit this pin resolves to")
    fi

    local path
    while IFS= read -r path; do
        [ -n "$path" ] || continue
        [ -e "$CORE/$path" ] || problems+=("missing build dependency: $CHECKOUT/$path")
    done < <(lock_paths)

    if [ "${#problems[@]}" -gt 0 ]; then
        echo "cFS core DOES NOT MATCH THE PIN ($PIN_TAG @ $(short "$PIN_COMMIT"))"
        printf '  - %s\n' "${problems[@]}"
        echo
        echo "The core is pristine by rule: everything orrery adds lives in"
        echo "cfs_mission/mission/ and is grafted on at configure time, so a change"
        echo "here is either an accident or a mission-wide decision about cFS."
        echo "  undo an accident   git -C $CHECKOUT checkout -- . && git -C $CHECKOUT submodule update --init --recursive"
        echo "  start over         scripts/fetch_cfs.sh --force   (discards local core edits)"
        echo "  move the pin       edit cfs_mission/cfs.lock deliberately: it re-stamps firmware provenance"
        return 1
    fi

    local nested
    nested=$("${GIT[@]}" -C "$CORE" submodule status --recursive 2>/dev/null | wc -l | tr -d ' ')
    echo "cFS core OK: $PIN_TAG @ $(short "$PIN_COMMIT") ($nested nested repositories, pristine)"
}

# "v7.0.1" is worth more to a reader than a bare SHA — when the checkout disagrees
# with the pin, the question is usually "what did I check out instead?"
describe_state() {
    local at
    at=$("${GIT[@]}" -C "$CORE" describe --tags "$1" 2>/dev/null || echo "")
    if [ -n "$at" ]; then
        printf '%s' "$at"
    else
        printf 'no tag at that commit'
    fi
}

fetch() {
    if [ -d "$CORE" ]; then
        if [ "${FORCE:-0}" = 1 ]; then
            echo "removing $CORE (--force)"
            rm -rf "$CORE" || die "could not remove $CORE"
        elif verify_out=$(verify 2>&1); then
            echo "$verify_out"
            echo "nothing to do: the core is already the pinned one (scripts/fetch_cfs.sh --force to redo it)"
            return 0
        else
            echo "refusing to touch $CORE: it is already there and it is not what the pin says" >&2
            printf '%s\n' "$verify_out" >&2
            echo >&2
            echo "This script will not delete a core it did not fetch. If the edits" >&2
            echo "there are yours, keep them somewhere and move the pin deliberately;" >&2
            echo "if they are not, re-fetch with --force." >&2
            return 1
        fi
    fi

    mkdir -p "$(dirname "$CORE")" || die "cannot create $(dirname "$CORE")"
    echo "cloning $PIN_REPO"
    echo "         ~500 MB including every nested repository this pin resolves to:"
    echo "         minutes, not seconds."
    # Capture the failure, because the loud one lies about its own cause:
    # "server certificate verification failed. CAfile: none" inside the build
    # container reads like a network or proxy fault, when what it means is that
    # the image has no CA bundle. It happened exactly that way, to me, on the
    # fresh-clone proof — the pruned image had lost ca-certificates.
    local clone_out
    if ! clone_out=$("${GIT[@]}" clone --quiet "$PIN_REPO" "$CORE" 2>&1); then
        [ -n "$clone_out" ] && echo "$clone_out" >&2
        case "$clone_out" in
        *[Cc]ertificate* | *SSL*)
            die "clone failed: TLS could not be verified.
         In the build container this means the image carries no CA bundle —
         rebuild it with ca-certificates: scripts/image.sh cfs --rebuild"
            ;;
        esac
        die "clone failed: $PIN_REPO"
    fi
    "${GIT[@]}" -C "$CORE" checkout --quiet --detach "$PIN_COMMIT" ||
        die "$(short "$PIN_COMMIT") is not in the clone — the pin names a commit this repository does not have"
    echo "fetching the nested repositories at the commits this pin resolves to"
    "${GIT[@]}" -C "$CORE" submodule update --init --recursive --jobs "$JOBS" ||
        die "nested fetch failed — check network, then re-run: scripts/fetch_cfs.sh --force"

    verify
}

# --------------------------------------------------------------------------
# arguments
# --------------------------------------------------------------------------

VERB=fetch
FORCE=0
for arg in "$@"; do
    case "$arg" in
    --verify) VERB=verify ;;
    --force)
        VERB=fetch
        FORCE=1
        ;;
    --print) VERB=print ;;
    -h | --help)
        cat <<'EOF'
fetch_cfs.sh — the cFS core this mission builds against, from its pin.

  scripts/fetch_cfs.sh            fill in cfs_mission/cFS/ if it is not there
  scripts/fetch_cfs.sh --verify   check what is there against the pin, touch nothing
  scripts/fetch_cfs.sh --force    discard the checkout and fetch it again
  scripts/fetch_cfs.sh --print    the pin, as key=value lines (for scripts)

The pin lives in cfs_mission/cfs.lock. The checkout stays out of git.
EOF
        exit 0
        ;;
    *) die "unknown argument '$arg' (--verify, --force, --print, --help)" ;;
    esac
done

case "$VERB" in
print) print_pin ;;
verify) verify ;;
fetch) fetch ;;
esac
