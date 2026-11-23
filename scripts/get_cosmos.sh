#!/usr/bin/env bash
# The ground station, fetched to its pin: COSMOS CE (OpenC3), pinned by cosmos.lock.
#
#   scripts/get_cosmos.sh            clone the pinned checkout if none; else verify
#   scripts/get_cosmos.sh --verify   a fraction of a second, writes nothing
#   scripts/get_cosmos.sh --force    move OUR checkout back onto the pin (refuses a
#                                    tree with local edits beyond .env)
#   scripts/get_cosmos.sh --print    what the pin says, and where it should live
#
# WHY a pin. COSMOS CE deploys from a checkout: openc3.sh SOURCES that checkout's
# .env and starts the images it names. So there are two version facts and they
# must agree - the checkout (compose templates, launcher scripts) and the images
# (OPENC3_TAG) - and `OPENC3_TAG=latest`, the upstream default, quietly
# guarantees they do not: the deployment this pin exists for ran a v6.4-era
# checkout against 7.5-era images, because the checkout was cloned in May and
# the images were pulled in October. Nothing else in this yard runs on an
# unverified version; the ground station was the one place that trusted a tag
# line. cfs.lock is the sibling lockfile - same discipline, different fetcher,
# because the cFS core is a build input and this is a deployment.
#
# The checkout lives OUTSIDE this repository (COSMOS_HOME, default
# ~/Projects/cosmos): it is a live installation with volumes, buckets and loaded
# gems, and vendoring it into our tree would pretend it is ours. The one tracked
# file this script EDITS is .env - upstream's documented deployment knob, which
# is where OPENC3_TAG lives - so "on the pin" here means: HEAD equals the pinned
# commit, .env carries the pinned image tag, and nothing else is modified.
# Anything further is the operator's work, and --force refuses to bulldoze it.
#
# Environment: COSMOS_HOME, GIT_BIN, COSMOS_LOCK (tests point at a fake lock).
set -uo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOCK="${COSMOS_LOCK:-$REPO/cosmos.lock}"
COSMOS_HOME="${COSMOS_HOME:-$HOME/Projects/cosmos}"
# one git invocation for the whole script; safe.directory because the checkout
# may arrive via bind mount or belong to another day's user
GIT=("${GIT_BIN:-git}" -c safe.directory='*')

die() {
    echo "get_cosmos: $*" >&2
    exit 1
}

[ -f "$LOCK" ] || die "no lock file at $LOCK - the pin is not optional"

lock_get() { sed -n "s/^$1[[:space:]]*=[[:space:]]*\"\([^\"]*\)\".*/\1/p" "$LOCK" | head -1; }
PIN_REPO=$(lock_get repository)
PIN_TAG=$(lock_get tag)
PIN_COMMIT=$(lock_get commit)
PIN_IMAGE=$(lock_get image_tag)
[ -n "$PIN_REPO" ] && [ -n "$PIN_COMMIT" ] && [ -n "$PIN_IMAGE" ] ||
    die "lock file $LOCK is missing repository, commit or image_tag"

# Array values in the lock (required_paths, renames_from): the quoted entries
# between the key line and the closing bracket. grep/sed, not Python: this runs
# before anything is installed, on the deployment box, at any hour.
lock_array() {
    sed -n "/^$1/,/^\]/p" "$LOCK" | sed -n 's/.*"\(.*\)".*/\1/p'
}
required_paths() { lock_array required_paths; }

# GitHub paths are case-insensitive and .git is optional; compare URLs on the
# project's own terms, not byte-wise.
_norm_url() { tr 'A-Z' 'a-z' <<<"$1" | sed 's/\.git$//'; }

# The checkout's origin must be the pinned repository OR a name it has answered
# to (renames_from). A project that renamed once will rename again; the lock
# records the lineage so --force can tell "old clone of the right project" from
# "someone else's deployment" - which is the whole question a refusal answers.
origin_on_lineage() {
    local want; want=$(_norm_url "${1:-}")
    [ -n "$want" ] || return 1
    local name
    for name in "$PIN_REPO" $(lock_array renames_from); do
        [ "$(_norm_url "$name")" = "$want" ] && return 0
    done
    return 1
}

short() { cut -c1-7 <<<"$1"; }

image_tag_in_env() { sed -n 's/^OPENC3_TAG=//p' "$COSMOS_HOME/.env" 2>/dev/null | head -1; }

# Everything this script can see wrong, in one pass: a half-listed diagnosis
# sends someone off to fix one thing, come back, and find the second.
problems() {
    [ -e "$COSMOS_HOME" ] || { echo "no checkout at $COSMOS_HOME (clone it: scripts/get_cosmos.sh)"; return; }
    "${GIT[@]}" -C "$COSMOS_HOME" rev-parse --git-dir >/dev/null 2>&1 ||
        { echo "$COSMOS_HOME exists but is not a git repository - move it aside or set COSMOS_HOME"; return; }
    local got dirty
    got=$("${GIT[@]}" -C "$COSMOS_HOME" rev-parse HEAD 2>/dev/null || echo "")
    [ "$got" = "$PIN_COMMIT" ] ||
        echo "checked out $(short "${got:-nothing}"), the pin is $(short "$PIN_COMMIT") ($PIN_TAG)"
    local p
    for p in $(required_paths); do
        [ -e "$COSMOS_HOME/$p" ] || echo "missing $p - the fetch did not go the whole way"
    done
    local tag
    tag=$(image_tag_in_env)
    [ "$tag" = "$PIN_IMAGE" ] ||
        echo ".env carries OPENC3_TAG=${tag:-<unset>}, the pin wants $PIN_IMAGE - the stack would run images this checkout does not describe"
    dirty=$("${GIT[@]}" -C "$COSMOS_HOME" status --porcelain 2>/dev/null | grep -v '^ M \.env$' | head -5)
    [ -z "$dirty" ] ||
        echo "local edits beyond .env - this checkout is someone's work, --force will not move it:"$'\n'"$(sed 's/^/        /' <<<"$dirty")"
}

verify() {
    local found
    found=$(problems)
    if [ -n "$found" ]; then
        echo "get_cosmos: $COSMOS_HOME is NOT the pinned ground station:" >&2
        sed 's/^/  - /' <<<"$found" >&2
        echo "  ways out: scripts/get_cosmos.sh --force   (move our checkout onto the pin)" >&2
        echo "              COSMOS_HOME=...               (point at another checkout)" >&2
        echo "              do it by hand                 (you know something we don't)" >&2
        return 1
    fi
    echo "COSMOS checkout OK: $PIN_TAG @ $(short "$PIN_COMMIT"), images tagged $PIN_IMAGE ($COSMOS_HOME)"
}

# The .env rewrite: upstream default is OPENC3_TAG=latest; the pin replaces it,
# idempotently, appending only if the key vanished from a future layout.
pin_env() {
    local envf="$COSMOS_HOME/.env"
    [ -f "$envf" ] || die "$envf missing after checkout - required_paths should have caught this"
    if grep -q '^OPENC3_TAG=' "$envf"; then
        sed -i "s/^OPENC3_TAG=.*/OPENC3_TAG=$PIN_IMAGE/" "$envf"
    else
        printf '\nOPENC3_TAG=%s\n' "$PIN_IMAGE" >>"$envf"
    fi
}

fetch() {
    if [ ! -e "$COSMOS_HOME" ]; then
        mkdir -p "$(dirname "$COSMOS_HOME")" || die "cannot create $(dirname "$COSMOS_HOME")"
        echo "cloning $PIN_REPO ($PIN_TAG) into $COSMOS_HOME"
        echo "         deployment checkout - images are pulled later, by openc3.sh."
        local out
        if ! out=$("${GIT[@]}" clone --quiet --branch "$PIN_TAG" "$PIN_REPO" "$COSMOS_HOME" 2>&1); then
            [ -n "$out" ] && echo "$out" >&2
            case "$out" in
            *[Cc]ertificate* | *SSL*)
                die "clone failed: TLS could not be verified - no CA bundle where this ran (on the host: install ca-certificates; in a container: rebuild the image that carries them)"
                ;;
            esac
            die "clone failed: $PIN_REPO"
        fi
        pin_env
    elif [ "${FORCE:-0}" = "1" ]; then
        # --force may only move OUR lineage's checkout: a repository the pin
        # knows (by current name or recorded rename), and the only local edit
        # is the .env line this script itself writes.
        local origin
        origin=$("${GIT[@]}" -C "$COSMOS_HOME" remote get-url origin 2>/dev/null || echo "")
        origin_on_lineage "$origin" ||
            die "--force refuses: $COSMOS_HOME is not this pin's checkout
         origin=${origin:-<none>}
         pin lineage: $( (echo "$PIN_REPO"; lock_array renames_from) | tr '\n' ' ')"
        local dirty
        dirty=$("${GIT[@]}" -C "$COSMOS_HOME" status --porcelain 2>/dev/null | grep -v '^ M \.env$' | head -3)
        [ -z "$dirty" ] || die "--force refuses: $COSMOS_HOME has local edits beyond .env"
        # Fetch opportunistically: a deployment box may be offline, or the
        # origin may be a recorded rename that no longer resolves. The commit
        # must exist LOCALLY either way - that is the fact checkout needs.
        "${GIT[@]}" -C "$COSMOS_HOME" fetch --quiet origin "refs/tags/$PIN_TAG:refs/tags/$PIN_TAG" 2>/dev/null ||
            "${GIT[@]}" -C "$COSMOS_HOME" fetch --quiet origin 2>/dev/null || true
        "${GIT[@]}" -C "$COSMOS_HOME" cat-file -e "${PIN_COMMIT}^{commit}" 2>/dev/null ||
            die "$(short "$PIN_COMMIT") is not in $COSMOS_HOME and fetching failed - fetch it, or this is the wrong checkout"
        "${GIT[@]}" -C "$COSMOS_HOME" checkout --quiet --detach "$PIN_COMMIT" ||
            die "cannot check out $(short "$PIN_COMMIT") in $COSMOS_HOME"
        pin_env
    fi
    verify
}

case "${1:-fetch}" in
    --print)
        printf '%s\n' "repository = $PIN_REPO" "tag        = $PIN_TAG" \
            "commit     = $PIN_COMMIT" "image_tag  = $PIN_IMAGE" "checkout   = $COSMOS_HOME"
        ;;
    --verify)
        verify
        ;;
    --force)
        FORCE=1 fetch
        ;;
    fetch | "")
        fetch
        ;;
    *)
        die "unknown verb '$1' (fetch, --verify, --force, --print)"
        ;;
esac
