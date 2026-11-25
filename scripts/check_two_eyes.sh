#!/usr/bin/env bash
# Two-eyed camera acceptance: one bird, two eyes, one law per eye.
#
#   record   camera_record.toml - the flown single-eye anchor, re-flown so
#            the digest comparison below compares TODAY's physics to
#            TODAY's physics, not to an archived number.
#   twins    two_eyes.toml - the same bird with a second [[camera]] block.
#            Both eyes stream CAMSTATE (ids 1, 2), each keeps its own
#            cadence and its own store outlet, and the ground commands ONE
#            named eye at a time (cam_probe --mode twins drives the script).
#
# The three witnesses this acceptance stands on:
#
#   1. raw-port id tally (sim.id_witness, dialed DURING the soak): the wire
#      itself carries camstate ids [1, 2]. No verdict wrote this receipt.
#   2. per-eye divergence (cam_probe twins mode): CAM_HK's commanded pair
#      must equal the NAMED eye's CamState pair, eye 1 must never blink
#      while eye 2 is being closed, and both ids must prove they streamed.
#   3. digest invariance: the twins flight's trajectory sha256 must equal
#      the record leg's byte for byte. A second eye that moved the bird is
#      a second eye wired into the control law, and this is how that bug
#      announces itself.
#
# /cf/img joins the byte argument too: files for BOTH eye ids must land,
# each sha256-identical to a PNG the sim's own writer produced.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
bash "$REPO/scripts/firmware.sh" check
RUNTIME="${RUNTIME:-docker}"
BOOT_TIMEOUT="${BOOT_TIMEOUT:-120}"
SIM_DURATION="${SIM_DURATION:-90}"
PROBE_DURATION="${PROBE_DURATION:-56}"
RECORD_SCENARIO="$REPO/sim/scenarios/camera_record.toml"
TWINS_SCENARIO="$REPO/sim/scenarios/two_eyes.toml"
WORK="${WORK:-/tmp/opencode/two_eyes}"
mkdir -p "$WORK"
HOST_IMG="$REPO/cfs_mission/mission/build/exe/cpu1/cf/img"

INSTANCE=0
BEACON=$((5050 + 2 * INSTANCE))
UPLINK=$((BEACON + 1))
SENSOR=$((5580 + 20 * INSTANCE))
LINK_CMD=$((SENSOR + 1))

# The two boot-poisoning squatters the yard learned to refuse (POSIX msg
# queues are a fixed 10 on this box, and a lingering listener steals the
# craft's handshake into a dead backlog): check before ANYTHING boots.
if pgrep -x core-cpu1 >/dev/null; then
    echo "FAIL: a host core-cpu1 is running (pid $(pgrep -xd core-cpu1 | tr '\n' ' '));" \
        "it starves every mission boot of message queues. Kill it first." >&2
    exit 1
fi
if ss -tln | grep -qE ":${SENSOR}([[:space:]]|\$)"; then
    echo "FAIL: tcp/${SENSOR} already has a listener before the first boot:" >&2
    ss -tlnp | grep -E ":${SENSOR}([[:space:]]|\$)" >&2 || true
    exit 1
fi

cd "$REPO"
CONTAINER=""
SIM_PID=""
RENDERER="orrery-cam-viz"
cleanup() {
    [ -n "$SIM_PID" ] && kill "$SIM_PID" >/dev/null 2>&1 || true
    "$RUNTIME" rm -f "$RENDERER" >/dev/null 2>&1 || true
    [ -n "$CONTAINER" ] && "$RUNTIME" stop "$CONTAINER" >/dev/null 2>&1 || true
}
trap cleanup EXIT

stale="$("$RUNTIME" ps -q --filter ancestor=orrery-cfs:dev || true)"
[ -n "$stale" ] && "$RUNTIME" stop $stale >/dev/null || true

# fly_cycle <probe-mode> <scenario> -> CYCLE_RC, CYCLE_DIGEST, CYCLE_LOG,
# CYCLE_STORE_DIRS (lines "id:dir", every outlet the sim announced),
# CYCLE_WITNESS (raw-port tally log, when the leg asked for one)
fly_cycle() {
    local mode="$1" scenario="$2"
    CYCLE_RC=0
    CYCLE_STORE_DIRS=""
    CYCLE_WITNESS=""
    rm -rf "$HOST_IMG"

    CONTAINER="$("$RUNTIME" run -d --network host --user "$(id -u):$(id -g)" -e HOME=/tmp \
        -e ORRERY_TLM_PORT="$BEACON" -e ORRERY_CMD_PORT="$UPLINK" \
        -e ORRERY_ZMQ_SENSOR_ENDPOINT="tcp://127.0.0.1:$SENSOR" \
        -e ORRERY_ZMQ_CMD_ENDPOINT="tcp://127.0.0.1:$LINK_CMD" \
        -v "$REPO:/work" orrery-cfs:dev \
        bash -c "cd /work/cfs_mission/mission/build/exe/cpu1 && ./container-start")"

    echo "=== [$mode] waiting for gs_tcp + fcs + cam (max ${BOOT_TIMEOUT}s) ==="
    local waited=0
    while [ "$waited" -lt "$BOOT_TIMEOUT" ]; do
        local logs
        logs="$("$RUNTIME" logs "$CONTAINER" 2>&1 || true)"
        if grep -q "gs_tcp downlink :$BEACON uplink :$UPLINK" <<<"$logs" \
            && grep -q "FCS App Initialized" <<<"$logs" \
            && grep -q "CAM App Initialized" <<<"$logs"; then
            break
        fi
        if grep -qE "(GS_TCP|FCS|BSK_LINK|CAM).*(fail|DEAD)|Could not load file|ES Load Module Error" <<<"$logs"; then
            echo "[$mode] FAILED (app init error)"; grep -E "GS_TCP|FCS|BSK_LINK|CAM |Could not load" <<<"$logs" | tail -12
            return 1
        fi
        sleep 1
        waited=$((waited + 1))
    done
    logs="$("$RUNTIME" logs "$CONTAINER" 2>&1 || true)"
    if ! grep -q "CAM App Initialized" <<<"$logs"; then
        echo "[$mode] FAILED (boot markers not seen within ${BOOT_TIMEOUT}s)"
        return 1
    fi

    echo "=== [$mode] staging the patient renderer ==="
    RUNTIME="$RUNTIME" bash "$REPO/scripts/renderer.sh" stage "$scenario" "$RENDERER"

    echo "=== [$mode] camera world (${scenario##*/}, ${SIM_DURATION} sim-s) ==="
    PYTHONUNBUFFERED=1 uv run python -m sim.forge_scenario --scenario "$scenario" \
        --duration "$SIM_DURATION" --zmq --pace >"$WORK/sim_$mode.log" 2>&1 &
    SIM_PID=$!
    sleep 3
    if ! kill -0 "$SIM_PID" 2>/dev/null; then
        echo "[$mode] world died at startup: a host link port is already taken?" >&2
        tail -3 "$WORK/sim_$mode.log" >&2 || true
        return 1
    fi

    if [ "$mode" = twins ]; then
        # The raw-port witness flies BEFORE the probe: it tallies what the
        # world streams when nobody has commanded anything, so no verdict,
        # no probe send, and no HK poll wrote its receipt.
        echo "=== [twins] raw-port id witness (8 s) ==="
        uv run python -m sim.id_witness --row camstate --seconds 8 --instance "$INSTANCE" \
            >"$WORK/witness_twins.log" 2>&1 || true
        CYCLE_WITNESS="$WORK/witness_twins.log"
        tail -2 "$CYCLE_WITNESS" | sed 's/^/    /'
    fi

    echo "=== [$mode] ground probe (${PROBE_DURATION}s) ==="
    uv run python -m sim.cam_probe --mode "$mode" --duration "$PROBE_DURATION" \
        >"$WORK/probe_$mode.log" 2>&1 || CYCLE_RC=$?
    tail -4 "$WORK/probe_$mode.log" | sed 's/^/    /'

    wait "$SIM_PID"; SIM_PID=""
    CYCLE_DIGEST="$(grep -oE 'trajectory sha256 [0-9a-f]{64}' "$WORK/sim_$mode.log" | tail -1 | awk '{print $3}')"
    [ -n "$CYCLE_DIGEST" ] || { echo "[$mode] FAILED: no digest printed"; return 1; }

    CYCLE_LOG="$WORK/console_$mode.log"
    "$RUNTIME" stop "$CONTAINER" >/dev/null 2>&1 || true
    "$RUNTIME" logs "$CONTAINER" >"$CYCLE_LOG" 2>&1 || true
    CONTAINER=""
    "$RUNTIME" rm -f "$RENDERER" >/dev/null 2>&1 || true

    CYCLE_STORE_DIRS="$(grep -oE 'camera[0-9]+ store outlet ON \(every [0-9]+th capture -> [^)]+\)' \
        "$WORK/sim_$mode.log" | sed -E 's/camera([0-9]+) store outlet ON \(every [0-9]+th capture -> ([^)]+)\)/\1:\2/')"
    echo "=== [$mode] digest ${CYCLE_DIGEST:0:16}... probe rc=$CYCLE_RC ==="
}

# --- record leg: today's single-eye anchor ----------------------------------
fly_cycle record "$RECORD_SCENARIO" || { echo "TWO-EYES CHECK FAILED (record leg)"; exit 1; }
RECORD_DIGEST="$CYCLE_DIGEST"
[ "$CYCLE_RC" -eq 0 ] || { echo "TWO-EYES CHECK FAILED (record probe)"; exit 1; }

# --- twins leg: the second eye mounts beside the same physics ----------------
fly_cycle twins "$TWINS_SCENARIO" || { echo "TWO-EYES CHECK FAILED (twins leg)"; exit 1; }
TWINS_DIGEST="$CYCLE_DIGEST"
TWINS_LOG="$CYCLE_LOG"
[ "$CYCLE_RC" -eq 0 ] || { echo "TWO-EYES CHECK FAILED (twins probe)"; exit 1; }

echo "=== witness 1: the wire itself carried both eyes ==="
[ -n "$CYCLE_WITNESS" ] || { echo "TWO-EYES FAILED: the raw-port witness never ran"; exit 1; }
grep -q 'ids=\[1, 2\]' "$CYCLE_WITNESS" || {
    echo "TWO-EYES FAILED: raw-port tally does not show both eyes:"
    cat "$CYCLE_WITNESS"; exit 1;
}
echo "  raw-port tally: $(grep -oE 'row=camstate frames=[0-9]+ ids=\[[0-9, ]+\]' "$CYCLE_WITNESS" | tail -1)"

echo "=== witness 2: the fit stayed honest with two eyes ==="
grep -q "FIT MATCHED" "$TWINS_LOG" || {
    echo "TWO-EYES FAILED: the two-eyed world never earned FIT MATCHED:"
    grep -E "FIT (SILENT|DENIED|ABORT)" "$TWINS_LOG" | tail -3; exit 1;
}
grep -qE "FIT (SILENT|DENIED|ABORT)" "$TWINS_LOG" && {
    echo "TWO-EYES FAILED: the fit lied somewhere while claiming to match:"
    grep -E "FIT (SILENT|DENIED)" "$TWINS_LOG" | tail -3; exit 1;
} || true
echo "  FIT MATCHED with both ids streamed and no alarm sentence"

echo "=== witness 3: a second eye moved no physics ==="
if [ "$RECORD_DIGEST" != "$TWINS_DIGEST" ]; then
    echo "TWO-EYES FAILED: the second eye touched the trajectory"
    echo "  record: ${RECORD_DIGEST:0:16}..."
    echo "  twins : ${TWINS_DIGEST:0:16}..."
    exit 1
fi
echo "  digest pair byte-identical: ${RECORD_DIGEST:0:16}..."

echo "=== both eyes' files crossed the airgap, byte for byte ==="
[ -d "$HOST_IMG" ] || { echo "no /cf/img on the host: nothing crossed the airgap"; exit 1; }
uv run python - "$HOST_IMG" "$CYCLE_STORE_DIRS" <<'PY'
import hashlib
import sys
from pathlib import Path

host = Path(sys.argv[1])
store_dirs = []
for line in sys.argv[2].splitlines():
    eye, _, directory = line.partition(":")
    store_dirs.append((eye, Path(directory)))

reasons: list[str] = []
sources = {
    hashlib.sha256(p.read_bytes()).hexdigest(): (eye, p.name)
    for eye, directory in store_dirs
    for p in directory.glob("*.png")
}
for eye, directory in store_dirs:
    landed = sorted(host.glob(f"frame_{eye}_*.png"))
    if len(landed) < 3:
        reasons.append(f"eye {eye}: only {len(landed)} files reached /cf/img (want >= 3)")
    for path in landed:
        if not path.read_bytes().startswith(b"\x89PNG"):
            reasons.append(f"eye {eye}: {path.name} is not a PNG")
        if hashlib.sha256(path.read_bytes()).hexdigest() not in sources:
            reasons.append(f"eye {eye}: {path.name} matches no PNG the sim wrote")
if not sources:
    reasons.append("the sim's own writers produced nothing: the byte argument is empty")
for why in reasons:
    print(f"  reason: {why}")
if reasons:
    raise SystemExit(1)
counts = ", ".join(
    f"eye {eye}: {len(sorted(host.glob(f'frame_{eye}_*.png')))} files" for eye, _ in store_dirs
)
print(f"  {counts}, every one byte-identical to its sim original")
PY

# The yard rests where the fleet rests: nothing of ours may linger afterwards.
bash "$REPO/scripts/fleet.sh" down >/dev/null 2>&1 || true

echo
echo "TWO-EYES CHECK OK: both eyes stream, each answers only for itself,"
echo "and the bird flies the same trajectory it did with one."
