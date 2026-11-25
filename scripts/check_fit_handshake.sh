#!/usr/bin/env bash
# Fit boot-handshake flight: the three views of a vehicle's fit, live.
#
#   scripts/check_fit_handshake.sh                 all five probes (~9 min)
#   PROBES=stale scripts/check_fit_handshake.sh    one probe, while iterating
#
# The claim is that absence is a fact the spacecraft knows, not a
# gap the ground papers over. The claim is only worth what a flight makes it, so
# this script flies one vehicle through four manifests and reads the verdict the
# FLIGHT SOFTWARE reaches, from its own console, in each case:
#
#   honest    the ops plant as written              -> FIT MATCHED
#   silent    same plant, claims a part it lacks    -> FIT SILENT, mission flies
#   denied    same plant, hides a part it streams   -> FIT DENIED
#   stale     world claims another registry digest  -> FIT ABORT, mission refuses
#
# The first row is the normal case; the last three are planted with the world's
# injection knobs (scripts/fleet.sh's FIT_DECLARE / FIT_HIDE / FIT_DIGEST, each
# echoed into the fleet log so a planted lie can never be mistaken for a broken
# spacecraft). The difference between rows 2-3 and row 4 is the whole design: a
# claim about THIS VEHICLE degrades and reports; a claim about the SCHEMA stops
# the mission, because bytes decoded under the wrong layout are not knowledge.
#
# Nothing here regenerates, rebuilds, or bumps a gem: the handshake frame has no
# TopicId, so it never reaches the ground and the ground defs cannot notice it.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export DURATION="${DURATION:-120}"
WORK="${WORK:-/tmp/opencode/ob_fit}"
STATE_DIR="${STATE_DIR:-/tmp/opencode}"
OPERATIONAL="sim/scenarios/twin_ops.toml"
SRC="sim/scenarios/chief.toml"
SOAK="${SOAK:-25}"
cd "$REPO"

mkdir -p "$WORK"
GIT_AT_TAKEOFF="$(git status --porcelain)"

echo "--- clearing the yard ---"
bash scripts/fleet.sh down >/dev/null 2>&1 || true
# A stray host-side craft (a diagnostic run that outlived its terminal) eats
# POSIX message queues, and this box caps them at 10: every mission boot then
# dies on CFE_SB_CreatePipe and the first probe fails with the yard's own
# ghost in the story. Name the ghost before it stars in one.
if pgrep -x core-cpu1 >/dev/null; then
    echo "FAIL: a host core-cpu1 is running (pid $(pgrep -xd core-cpu1 | tr '\n' ' '));" \
         "it squats the msg queues this yard needs - kill it first" >&2
    exit 1
fi
# and the link farm must belong to nobody: a world that outlived its container
# accepts a fresh craft's handshake with a dying peer, and that craft then
# receives nothing while its socket reads connected (the zero-frame stall).
if ss -tln | grep -qE ':5580([[:space:]]|$)'; then
    echo "FAIL: tcp/5580 already has a listener before the first boot:" >&2
    ss -tlnp | grep -E ':5580([[:space:]]|$)' >&2 || true
    echo "      a squatting world squanders every probe - kill it first" >&2
    exit 1
fi

# The probe vehicle: chief.toml, the superset plant the part-landing flight used
# (GNSS + wheels + ST + CSS + mag), so all three registered rows are in play.
PROBE="$WORK/probe_fit.toml"
cp "$SRC" "$PROBE"
NAME="$(uv run python -c "from sim.fleet import scenario_spacecraft; print(scenario_spacecraft('$PROBE')[0])")"
CONTAINER="orrery-$NAME"
echo "probe: $PROBE (vehicle $NAME)"

# ---- assertions ------------------------------------------------------------
console_of() { docker logs "$CONTAINER" >"$1" 2>&1 || true; }

expect_in_console() { # <regex> <label>
    if ! grep -qE "$1" "$WORK/console.log"; then
        echo "FAIL: no '$2' in the mission console (wanted /$1/)" >&2
        grep -E "FIT|BSK_LINK 1[4-9]" "$WORK/console.log" | tail -10 >&2 || true
        exit 1
    fi
    grep -E "$1" "$WORK/console.log" | tail -1 | sed 's/^/    /'
}

refuse_in_console() { # <regex> <label>
    if grep -qE "$1" "$WORK/console.log"; then
        echo "FAIL: '$2' appeared where this probe must not produce it (/$1/)" >&2
        grep -E "$1" "$WORK/console.log" | tail -3 >&2
        exit 1
    fi
}

barrier_alive() {
    if ! grep -q "basilisk sensor frame" "$WORK/console.log"; then
        echo "FAIL: the mission never took a sensor tick - that is no mission, not a fit report" >&2
        exit 1
    fi
}

# The barrier's own retransmit races the handshake: the check re-shouts
# SENSOR every 0.5 s while a tick's FIT arrives once per second, so a mission
# may decode a tick or two before the first manifest lands and the abort
# engages. That is the design's order of operations, not a hole - with no
# manifest a link flies the legacy contract, because a spacecraft that has
# never heard from its world cannot name a fit. So the stale probe asserts the
# honest thing: once aborted, NO further tick is decoded.
ticks_after_abort() {
    awk '/FIT ABORT/ { aborted = 1; next }
         aborted && /basilisk sensor frame/ { found++ }
         END { print found + 0 }' "$WORK/console.log"
}

# ---- the harness -----------------------------------------------------------
# run_probe <tag> [KEY=VAL ...]: fleet up with those injections, soak, receipts.
# Optional WITNESS_CMD runs during the soak (output to ${tag}_witness.log):
# a witness has to speak while the yard is still up, and its receipts join
# the other evidence files.
run_probe() {
    local tag="$1" pair
    shift
    local log="$WORK/${tag}.log"
    : >"$log"
    echo "=== [$tag] fleet up (injections: ${*:-none}) ==="
    for pair in "$@"; do
        export "${pair%%=*}"="${pair#*=}"
    done
    if ! bash scripts/fleet.sh up "$PROBE" 2>&1 | tee -a "$log"; then
        echo "[$tag] FAIL: fleet did not come up" >&2
        exit 1
    fi
    local witness_pid=""
    if [ -n "${WITNESS_CMD:-}" ]; then
        timeout 40 bash -c "$WITNESS_CMD" >"$WORK/${tag}_witness.log" 2>&1 &
        witness_pid=$!
    fi
    sleep "$SOAK"
    console_of "$WORK/console.log"
    if ! grep -qE "basilisk sensor frame|FIT ABORT" "$WORK/console.log"; then
        # The race this yard taught today (measured twice, logs as the proof):
        # a world whose ack deadline is shorter than a craft boot executes its
        # own stall before frame 1 is ever answered - the leg then proves
        # nothing about the flight, only about boot choreography. This is NOT
        # a verdict to assert on: it is a leg that never flew. Two sentences
        # prove the world spoke to its craft: a decoded tick, or a FIT ABORT
        # (the stale leg refuses the barrier BY DESIGN, so it never decodes a
        # tick). Boot the pair once more; a second miss is a real failure and
        # says so.
        echo "[$tag] no sensor tick in ${SOAK}s - the world missed its craft; re-booting the pair once" >&2
        [ -n "$witness_pid" ] && kill "$witness_pid" 2>/dev/null || true
        bash scripts/fleet.sh down "$PROBE" >/dev/null 2>&1 || true
        if ! bash scripts/fleet.sh up "$PROBE" >>"$log" 2>&1; then
            echo "[$tag] FAIL: fleet did not come up on the retry" >&2
            exit 1
        fi
        sleep "$SOAK"
        console_of "$WORK/console.log"
        if ! grep -qE "basilisk sensor frame|FIT ABORT" "$WORK/console.log"; then
            echo "[$tag] FAIL: the mission heard no frame twice - this is not the race" >&2
            exit 1
        fi
        if [ -n "$witness_pid" ]; then
            # the witness dialed the ghost, not this world: start it fresh on
            # the boot that actually flies, so its tally is a witness again
            kill "$witness_pid" 2>/dev/null || true
            timeout 40 bash -c "$WITNESS_CMD" >"$WORK/${tag}_witness.log" 2>&1 &
            witness_pid=$!
        fi
    fi
    if [ -n "$witness_pid" ]; then
        wait "$witness_pid" 2>/dev/null || { echo "[$tag] FAIL: witness died" >&2; cat "$WORK/${tag}_witness.log" >&2; exit 1; }
    fi
    # the receipts are captured AFTER the leg is known to fly, from the boot
    # that actually speaks (the pre-check capture above is the same file,
    # refreshed; the world log joins it here)
    # The world's log is the second half of every receipt: a verdict that never
    # reached the sim is a verdict nobody acted on. The compose world logs to
    # its container (compose.yml names it orrery-world); a leftover file from
    # the old launcher must never stand in for it (an existence guard on such
    # a file once passed on yesterday's evidence).
    if ! docker logs orrery-world >"$WORK/${tag}_sim.log" 2>&1; then
        echo "FAIL: world container unreadable - nothing to read the stall from" >&2
        exit 1
    fi
    for pair in "$@"; do
        unset "${pair%%=*}"
    done
    bash scripts/fleet.sh down "$PROBE" >/dev/null 2>&1 || true
    echo "[$tag] evidence: $log  $WORK/${tag}_sim.log  $WORK/console.log"
    echo
}

# ---- probe 1: manifest, build, and bus agree ------------------------------
probe_honest() {
    run_probe honest
    expect_in_console "FIT MATCHED: declared=[0-9a-f]{8} streamed=[0-9a-f]{8} registry=[0-9a-f]{8}" "FIT MATCHED"
    expect_in_console "FIT MATCHED: declared=00000007 streamed=00000007" "a MATCHED over a FULL bus"
    refuse_in_console "FIT (SILENT|DENIED|ABORT)" "a fit alarm"
    barrier_alive
    echo "[honest] OK: one fit, three voices, and no alarm"
}

# ---- probe 2: the manifest claims a part the vehicle does not have --------
# chief.toml HAS a star tracker, so its hardware is removed first (the same
# block the part-landing flight adds). The claim is then the lie, and the verdict
# must name the row while the mission keeps flying on the fit it really has.
probe_silent() {
    local scen="$WORK/probe_silent.toml"
    uv run python - "$WORK/probe_fit.toml" "$scen" <<'PY'
import re
import sys
from pathlib import Path

src, dst = Path(sys.argv[1]), Path(sys.argv[2])
head, *blocks = src.read_text(encoding="utf-8").split("[[spacecraft.sensors]]")
keep, dropped = [], 0
for block in blocks:
    if re.search(r'type\s*=\s*"starTracker"', block):
        dropped += 1
    else:
        keep.append(block)
if dropped != 1:
    raise SystemExit(f"{src}: expected exactly one starTracker block, found {dropped}")
dst.write_text(head + "".join("[[spacecraft.sensors]]" + b for b in keep), encoding="utf-8")
print(f"wrote {dst}: starTracker removed, {len(keep)} sensor blocks left")
PY
    PROBE="$scen"
    run_probe silent FIT_DECLARE=attitude
    expect_in_console "FIT SILENT: manifest claims ST attitude \(00000004\) but the bus never streamed it" "FIT SILENT"
    expect_in_console "basilisk sensor frame" "the barrier"
    refuse_in_console "FIT ABORT" "an abort"
    echo "[silent] OK: a claimed part is named, and the mission flies on the fit it has"
}

# ---- probe 3: the bus streams what the manifest denies ---------------------
probe_denied() {
    PROBE="$WORK/probe_denied.toml"
    cp "$SRC" "$PROBE"
    run_probe denied FIT_HIDE=gnss
    expect_in_console "FIT DENIED: bus streams GNSS fix \(00000001\) that the manifest denies" "FIT DENIED"
    barrier_alive
    echo "[denied] OK: an undeclared stream is an alarm, not telemetry the ground keeps quietly"
}

# ---- probe 4: firmware and catalog disagree -------------------------------
probe_stale() {
    PROBE="$WORK/probe_stale.toml"
    cp "$SRC" "$PROBE"
    run_probe stale FIT_DIGEST=deadbeef
    expect_in_console "FIT ABORT: firmware built from registry [0-9a-f]{8}, world runs deadbeef" "FIT ABORT"
    # The other half of the receipt: with no ack the check must stall, loudly.
    if ! grep -qiE "lockstep stall" "$WORK/stale_sim.log"; then
        echo "FAIL: an aborting mission left the world waiting without naming the stall" >&2
        tail -5 "$WORK/stale_sim.log" >&2 || true
        exit 1
    fi
    grep -iE "lockstep stall" "$WORK/stale_sim.log" | tail -1 | sed 's/^/    /'
    local refused
    refused="$(ticks_after_abort)"
    if [ "$refused" != "0" ]; then
        echo "FAIL: $refused sensor ticks decoded AFTER the abort - ticks were accepted, not refused" >&2
        exit 1
    fi
    echo "    0 sensor ticks decoded after the abort"
    echo "[stale] OK: stale firmware refuses to fly, and says which digest it was built from"
}

WANTED="${PROBES:-honest silent denied stale ids}"
wanted() { case " $WANTED " in *" $1 "*) return 0 ;; *) return 1 ;; esac; }

# ---- probe 5: the handshake counts instances ------------------------------
# The eye's frame says WHICH eye it came from, and the manifest claims ids.
# Leg one is the honest eye: declared id 1, streamed id 1 - MATCHED must stay
# byte-identical, because attribution that changes a clean verdict is noise.
# Leg two claims a second eye no device carries: the verdict must name the
# id, not just the part, and the mission flies on the fit it really has.
# Each leg carries a raw-port witness: a ground tally of the ids the world
# actually streams, dialed straight off the sensor PUB, so a verdict that
# names an id has an outside witness that no verdict wrote.
probe_ids() {
    PROBE="$WORK/probe_ids.toml"
    cp sim/scenarios/camera_ops.toml "$PROBE"
    export WITNESS_CMD="uv run python -m sim.id_witness --row camstate --seconds 8"
    local tally
    run_probe ids-honest
    tally="$(cat "$WORK/ids-honest_witness.log")"
    expect_in_console "FIT MATCHED: declared=[0-9a-f]{8} streamed=[0-9a-f]{8} registry=[0-9a-f]{8}" "FIT MATCHED"
    refuse_in_console "FIT (SILENT|DENIED|ABORT)" "a fit alarm beside an honest eye"
    barrier_alive
    [[ "$tally" == *"ids=[1]"* ]] || { echo "[ids-honest] FAIL: raw port streamed no id 1 ($tally)" >&2; exit 1; }
    echo "[ids-honest] OK: an eye's id is counted ($tally), and a clean fit still reads clean"

    run_probe ids-silent FIT_DECLARE_ID="camstate=2"
    tally="$(cat "$WORK/ids-silent_witness.log")"
    expect_in_console "FIT SILENT: manifest claims CamState id 2 but no frame carried it" "an id-level SILENT"
    barrier_alive
    refuse_in_console "FIT ABORT" "an abort"
    [[ "$tally" == *"ids=[1]"* ]] || { echo "[ids-silent] FAIL: the bus must still carry only id 1 ($tally)" >&2; exit 1; }
    echo "[ids-silent] OK: id 2 is claimed but nowhere on the wire ($tally) - named by id, mission flies on"
    unset WITNESS_CMD
}

if wanted honest; then probe_honest; fi
if wanted silent; then probe_silent; fi
if wanted denied; then probe_denied; fi
if wanted stale; then probe_stale; fi
if wanted ids; then probe_ids; fi

# ---- receipts --------------------------------------------------------------
echo "=== what this flight edited ==="
if [ "$(git status --porcelain)" != "$GIT_AT_TAKEOFF" ]; then
    echo "FAIL: the handshake flight changed tracked files - it must not:" >&2
    diff <(printf '%s\n' "$GIT_AT_TAKEOFF") <(git status --porcelain) >&2 || true
    exit 1
fi
echo "git status: unchanged from takeoff (0 regen, 0 rebuild, 0 gem bump)"
echo "evidence: $WORK/{honest,silent,denied,stale}.log + _sim.log + console.log"

if [ "${RESTORE_FLEET:-1}" = "1" ]; then
    echo "--- restoring the operational fleet ($OPERATIONAL, perpetual) ---"
    DURATION=inf bash scripts/fleet.sh up "$OPERATIONAL" | tail -1
fi
echo "FIT HANDSHAKE FLIGHT OK"
