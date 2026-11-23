#!/usr/bin/env bash
# The acceptance ladder, in one command, in the only order that works.
#
#   scripts/gates.sh                      offline + build + live (a full sweep)
#   scripts/gates.sh --tier offline       the mirrors, ~10 s
#   scripts/gates.sh --tier build         EDS database + mission image
#   scripts/gates.sh --tier live          the fleet ladder (needs the yard)
#   scripts/gates.sh --list               the plan and what each gate proves
#   scripts/gates.sh --only fleet-fly      one gate, by name fragment
#
# TIERS ARE CUMULATIVE, on purpose. `--tier live` also runs offline and build,
# because scripts/fleet.sh launches the ALREADY-BUILT cfs_mission/mission/build/
# exe/cpu1/container-start: run a live gate against a stale build directory and
# the fleet flies old firmware while the tree says otherwise. That is precisely
# the lie OB's registry digest was minted to catch, and a runner that let you
# ask for "just the live gates" would be handing it to you.
#
# WHY THIS FILE EXISTS. Every gate predates it, and the order they had to run
# in lived in a head (this one's), which is the same as saying it lived in a
# conversation log. Five rules that have each already cost an afternoon are now
# structure rather than memory:
#
#   1. the yard is swept before any digest gate - the twin gate refuses to
#      anchor against a world that is already running ("start-frame anchor
#      missed: first frame at trigger was 2635"), and the sweep costs seconds;
#   2. a dirty tree does not fly digest gates - several gates record `git
#      status` as proof that the flight matched the commit, so editing files
#      under a running gate voids that gate's own receipt (--allow-dirty to
#      override, and know what you are voiding);
#   3. COSMOS is probed before the ground gates, not assumed: without a loaded
#      gem the wrap fails as a missing-packet timeout three minutes in, when a
#      two-second JSON-RPC call could have named the fix;
#   4. nothing is skipped quietly. A gate that could not run is INCOMPLETE and
#      exits non-green with the command that fixes it - a SKIP that still
#      reports green is how a gate quietly becomes folklore; and a gate that
#      COULD NOT run because a dependency failed is BLOCKED, --keep-going
#      notwithstanding: collecting verdicts among peers never outranks physics;
#   4b. the yard itself now refuses stale firmware: every launcher of
#      container-start calls scripts/firmware.sh, which stamps the mission build
#      with a content hash of its inputs and refuses to boot a binary that does
#      not match this tree (ORRERY_ALLOW_STALE=1 to fly it knowingly). So even a
#      direct `scripts/fly_fleet.sh`, bypassing this runner, cannot fly
#      flight software the tree did not produce;
#   4c. a gate is a command a child shell can resolve - a script or an argv, not
#       a shell function or a variable that lives only here (see below);
#   5. the yard is restored on the way out, so the standing perpetual fleet is
#      found as its operator left it (--no-restore to keep it down).
#
# Every gate's own output is kept in $LOG_DIR (default
# /tmp/opencode/gates/<UTC stamp>/), one file per gate: this script adds no
# verdict of its own beyond an exit code, because the gates are the authority.
set -uo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO"

TIER=live
KEEP_GOING=0
LIST_ONLY=0
ALLOW_DIRTY=0
NO_RESTORE=0
ONLY=""
GATE_TIMEOUT="${GATE_TIMEOUT:-3600}"
LOG_DIR="${GATE_LOG_DIR:-/tmp/opencode/gates/$(date -u +%Y%m%d-%H%M%SZ)}"
STATE_DIR="${STATE_DIR:-/tmp/opencode}"
PERPETUAL="${PERPETUAL_SCENARIO:-sim/scenarios/twin_ops.toml}"

# tier | gate | what it proves | command | tag
GATES=(
    "offline|pytest|mirrors, registry, coverage, policy, fit handshake|uv run pytest -q|"
    "offline|ruff|no lint debt|uv run ruff check .|"
    "offline|format|generated and hand-written surfaces are formatted|uv run ruff format --check .|"
    "offline|types|no untyped drift in the harness|uv run ty check .|"
    "build|cfs-pin|the core on disk IS the pinned core: right commit, pristine, complete|bash scripts/fetch_cfs.sh --verify|"
    "build|edslink|EDS database + binding: each container's REAL size, checked against struct.pack|bash edslink/build.sh all|"
    "build|mission-build|cFS builds, every registry row resolves against the linked schema|bash scripts/mission_build.sh|"
    "live|cosmos-pin|the ground station on disk IS the pinned station: checkout and image tag agree|bash scripts/get_cosmos.sh --verify|ground"
    "live|pair-digest|witness + anchored pair digests: the sim timeline, not the host clock|bash scripts/check_pair_digest.sh|"
    "live|ops-replay|the scheduled flight repeats itself AND disagrees with the reactive one|bash scripts/check_ops_replay.sh|"
    "live|fleet-fly|one world, N containers, both digests, no stale anchor|bash scripts/fly_fleet.sh|"
    "live|modes|the supervisor trips, refuses, and recovers on a live bus|bash scripts/check_modes.sh|"
    "live|fit-handshake|the four boot verdicts: matched, silent, denied, abort|bash scripts/check_fit_handshake.sh|"
    "live|truth-exhibits|exhibits at device scale, truth uncommandable, refusal logged|bash scripts/check_truth_exhibits.sh|ground"
    "live|beacon-wrap|beacon table vs generated defs, both ways, 0 UNKNOWN|bash scripts/check_beacon_tables.sh|ground"
)

declare -A TIER_RANK=([offline]=1 [build]=2 [live]=3)

usage() {
    # the header comment IS the help: one place, no drift
    awk 'NR>1 && /^#/ {sub(/^# ?/, ""); print; next} NR>1 {exit}' "${BASH_SOURCE[0]}"
}

# A gate is a command a CHILD shell must resolve (`timeout … bash -c "$cmd"`):
# a script path or an argv, never a shell function defined here, and never a
# variable that lives only in this process. Both failures of the build gate were
# that mistake - rc=127 when the function was invisible to the child, then
# rc=125 when $REPO arrived as a name instead of a value (`docker: invalid spec:
# :/work`). scripts/mission_build.sh is the fix, and it is also the command a
# human would type.

while [ $# -gt 0 ]; do
    case "$1" in
        --tier) TIER="${2:?--tier wants offline, build, live, or all}"; shift 2 ;;
        --keep-going) KEEP_GOING=1; shift ;;
        --list) LIST_ONLY=1; shift ;;
        --allow-dirty) ALLOW_DIRTY=1; shift ;;
        --no-restore) NO_RESTORE=1; shift ;;
        --only) ONLY="${2:?--only wants a name fragment}"; shift 2 ;;
        -h|--help) usage; exit 0 ;;
        *) echo "gates.sh: unknown option $1 (try --help)" >&2; exit 2 ;;
    esac
done

[ "$TIER" = all ] && TIER=live
if [ -z "${TIER_RANK[$TIER]:-}" ]; then
    echo "gates.sh: no such tier '$TIER' (offline, build, live, all)" >&2
    exit 2
fi
WANTED_TIER="${TIER_RANK[$TIER]}"

# --- plan -------------------------------------------------------------------
selected() {
    local entry tier name proves cmd tag rank
    for entry in "${GATES[@]}"; do
        IFS='|' read -r tier name proves cmd tag <<<"$entry"
        rank="${TIER_RANK[$tier]}"
        [ "$rank" -le "$WANTED_TIER" ] || continue
        if [ -n "$ONLY" ] && [[ "$name" != *"$ONLY"* ]]; then
            continue
        fi
        printf '%s\n' "$tier|$name|$proves|$cmd|$tag"
    done
}

if [ "$LIST_ONLY" -eq 1 ]; then
    echo "tiers up to: $TIER    logs: $LOG_DIR"
    while IFS='|' read -r tier name proves cmd tag; do
        printf '  [%-7s] %-14s %s\n' "$tier" "$name" "$proves"
        if [ -n "$tag" ]; then
            printf '  %17s (COSMOS is probed with one JSON-RPC call before this runs)\n' ""
        fi
        printf '  %17s $ %s\n' "" "$cmd"
    done < <(selected)
    exit 0
fi

# --- preflight --------------------------------------------------------------
mkdir -p "$LOG_DIR"
RESULTS=()
FAILURES=0
SKIPPED=0
BLOCKED=0
RAN=0
BUILD_BROKEN=""
SWEPT=0
YARD_AT_START=0
STOPPED_AT=""

yard_count() { docker ps --filter name=orrery- --format '{{.Names}}' 2>/dev/null | grep -c . || true; }

# The probe asks for the thing the ground gates actually need, not for a web
# server: COSMOS answers JSON-RPC all day with no orrery gem loaded, and the
# wrap would then die on an unknown target three minutes in.
cosmos_answers() {
    GROUND_WHY="$(uv run python -c '
import sys

try:
    from sim.p2_cosmos_check import login, rpc

    targets = set(rpc(login(), "get_target_names"))
except Exception as exc:  # noqa: BLE001 - "not ready" has many shapes, one message
    sys.exit(f"COSMOS did not answer: {type(exc).__name__}: {str(exc).splitlines()[0]}")

missing = sorted({"SAT_CHIEF", "TRUTH_CHIEF"} - targets)
if missing:
    sys.exit(f"loaded targets lack {missing} - the orrery gem is not loaded")
' 2>&1)"
    [ -z "$GROUND_WHY" ]
}

restore_yard() {
    local code=$?
    if [ "$SWEPT" -eq 1 ] && [ "$NO_RESTORE" -eq 0 ] && [ "$YARD_AT_START" -gt 0 ]; then
        echo "--- restoring the standing fleet ($PERPETUAL, perpetual) ---"
        DURATION=inf bash scripts/fleet.sh up "$PERPETUAL" >"$STATE_DIR/gates_restore.log" 2>&1 || {
            echo "gates.sh: RESTORE FAILED - see $STATE_DIR/gates_restore.log" >&2
            echo "          the yard is EMPTY until this succeeds; re-run:" >&2
            echo "          DURATION=inf bash scripts/fleet.sh up $PERPETUAL" >&2
        }
    fi
    if [ -n "$STOPPED_AT" ]; then
        echo "gates.sh: stopped at '$STOPPED_AT'; the rest of the ladder did not run"
    fi
    echo "logs: $LOG_DIR"
    exit "$code"
}
trap restore_yard EXIT

echo "=== orrery gates: tiers up to $TIER, logs $LOG_DIR ==="

if [ "$WANTED_TIER" -ge 3 ]; then
    YARD_AT_START="$(yard_count)"
    if [ "$YARD_AT_START" -gt 0 ] && [ "$KEEP_GOING" -eq 0 ]; then
        echo "note: $YARD_AT_START container(s) already in the yard; they will be swept and restored"
    fi
    if [ -n "$(git status --porcelain)" ] && [ "$ALLOW_DIRTY" -eq 0 ]; then
        echo "gates.sh: the tree is DIRTY, and live gates record 'git status' as their receipt."
        echo "          Commit or stash first, or pass --allow-dirty to fly anyway"
        echo "          (that voids the receipt those gates are collecting)."
        git status --porcelain | sed 's/^/            /'
        STOPPED_AT="dirty tree"
        exit 1
    fi
    if ! docker info >/dev/null 2>&1; then
        echo "gates.sh: docker is not answering, and every live gate is a container" >&2
        STOPPED_AT="no docker"
        exit 1
    fi
fi

TIMEOUT_BIN=""
command -v timeout >/dev/null 2>&1 && TIMEOUT_BIN="timeout ${GATE_TIMEOUT}s"

# Two lines per gate: a sweep runs for the better part of an hour, and a
# single line written only when a gate finishes looks like a hang from the
# third gate onward.
run_gate() {
    local tier=$1 name=$2 proves=$3 cmd=$4 log="$LOG_DIR/$1-$2.log"
    local start rc dur status
    printf '%-7s %-13s running    %s\n' "$tier" "$name" "$proves"
    start=$SECONDS
    # shellcheck disable=SC2086  # word splitting of TIMEOUT_BIN is the point
    $TIMEOUT_BIN bash -c "$cmd" >"$log" 2>&1
    rc=$?
    dur=$((SECONDS - start))
    if [ "$rc" -eq 0 ]; then
        status=PASS
    elif [ "$rc" -eq 124 ]; then
        status="TIMEOUT (${GATE_TIMEOUT}s)"
    else
        status="FAIL (rc=$rc)"
    fi
    printf '        %-13s %-14s %ss\n' "" "$status" "$dur"
    [ "$status" = PASS ] || printf '                 log: %s\n' "$log"
    RESULTS+=("$status|$name")
    [ "$status" = PASS ] && return 0
    return 1
}

GROUND_OK=""
while IFS='|' read -r tier name proves cmd tag; do
    [ -n "$tier" ] || continue

    # A dependency is not a peer. --keep-going collects verdicts among gates
    # that are independent of each other; it must never override the physical
    # fact that live gates fly the firmware the build tier produces. This was
    # the worse half of the rc=127 discovery: the ladder walked into the fleet
    # after its own build gate had failed. Note the order - blocked live gates
    # are reported BEFORE the sweep, so a broken build does not even take the
    # operator's yard down.
    if [ "$tier" = live ] && [ -n "$BUILD_BROKEN" ]; then
        printf '%-7s %-13s %-14s  the build tier did not pass (%s)\n' \
            "$tier" "$name" "BLOCKED" "$BUILD_BROKEN"
        RESULTS+=("BLOCKED|$name")
        BLOCKED=$((BLOCKED + 1))
        continue
    fi

    RAN=$((RAN + 1))

    if [ "$tier" = live ] && [ "$SWEPT" -eq 0 ]; then
        echo "--- sweeping the yard (digest gates must anchor against nothing) ---"
        bash scripts/fleet.sh down >"$LOG_DIR/sweep.log" 2>&1 || true
        SWEPT=1
    fi

    if [ "$tag" = ground ]; then
        if [ -z "$GROUND_OK" ]; then
            if cosmos_answers; then
                GROUND_OK=1
            else
                GROUND_OK=0
            fi
        fi
        if [ "$GROUND_OK" = 0 ]; then
            printf '%-7s %-13s %-14s  %s\n' "$tier" "$name" "INCOMPLETE" \
                "${GROUND_WHY:-COSMOS JSON-RPC did not answer}" | cut -c1-150
            echo "                 fix: bash scripts/cosmos_stack.sh   (loads the current gem)"
            RESULTS+=("INCOMPLETE|$name")
            SKIPPED=$((SKIPPED + 1))
            [ "$KEEP_GOING" -eq 1 ] && continue
            STOPPED_AT="$name (COSMOS not loaded)"
            exit 1
        fi
    fi

    if ! run_gate "$tier" "$name" "$proves" "$cmd"; then
        if [ "$tier" = build ]; then
            BUILD_BROKEN="$name"
        fi
        if [ "$KEEP_GOING" -eq 1 ]; then
            FAILURES=$((FAILURES + 1))
            continue
        fi
        FAILURES=$((FAILURES + 1))
        STOPPED_AT="$name"
        break
    fi
done < <(selected)

# --- verdict ----------------------------------------------------------------
echo
echo "=== verdict ==="
for result in "${RESULTS[@]}"; do
    printf '  %-14s %s\n' "${result%%|*}" "${result#*|}"
done
if [ "$BLOCKED" -gt 0 ]; then
    echo "BLOCKED: $BLOCKED gate(s) never ran - the build tier failed ($BUILD_BROKEN)"
    echo "         Fix the build; the yard was left as it was."
    exit 1
fi
if [ "$SKIPPED" -gt 0 ]; then
    echo "INCOMPLETE: $SKIPPED gate(s) could not run - that is not a green sweep"
    exit 1
fi
if [ "$FAILURES" -gt 0 ]; then
    echo "FAILED: $FAILURES of $RAN gate(s) did not pass"
    exit 1
fi
echo "ALL $RAN GATES PASS - tiers up to $TIER"
