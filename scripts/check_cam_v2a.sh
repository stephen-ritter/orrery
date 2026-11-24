#!/usr/bin/env bash
# CAM v2a acceptance: the command surface, flown twice, digests compared.
#
# One scenario (camera_ops - the bird with the rendered eye), two flights:
#
#   control   ground sends NOTHING; the eye runs as TOML built it.
#   spam      the v2a cadence (sim/cam_probe.py): close + rerate after the
#             boot stream, reopen, two app refusals (the B1 zero, a shutter
#             of 2), a ghost-eye order the app must accept and only the sim
#             door refuses, restore - while CAM_HK and CamState are watched
#             from the ground.
#
# The argument is the PAIR: trajectory digests must be byte-identical, because
# commanding an eye must never touch physics (the v1 law, carried forward).
# The witnesses are the two views meeting: CAM_HK (commanded) and the landed
# camstate row (confirmed) converge on every accept, and the refusals move
# only the error counter - each named on EVS. FIT must also have grown: five
# rows claimed on this bird, CamState by name.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
bash "$REPO/scripts/firmware.sh" check
RUNTIME="${RUNTIME:-docker}"
BOOT_TIMEOUT="${BOOT_TIMEOUT:-120}"
SIM_DURATION="${SIM_DURATION:-60}"
PROBE_DURATION="${PROBE_DURATION:-56}"
SCENARIO="$REPO/sim/scenarios/camera_ops.toml"
WORK="${WORK:-/tmp/opencode/cam_v2a}"
mkdir -p "$WORK"

INSTANCE=0
BEACON=$((5050 + 2 * INSTANCE))
UPLINK=$((BEACON + 1))
SENSOR=$((5580 + 20 * INSTANCE))
LINK_CMD=$((SENSOR + 1))

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

# fly_cycle <mode> -> sets CYCLE_RC (probe), CYCLE_DIGEST, CYCLE_LOG (console)
fly_cycle() {
    local mode="$1"
    CYCLE_RC=0

    CONTAINER="$("$RUNTIME" run -d --network host --user "$(id -u):$(id -g)" -e HOME=/tmp \
        -e ORRERY_TLM_PORT="$BEACON" -e ORRERY_CMD_PORT="$UPLINK" \
        -e ORRERY_ZMQ_SENSOR_ENDPOINT="tcp://127.0.0.1:$SENSOR" \
        -e ORRERY_ZMQ_CMD_ENDPOINT="tcp://127.0.0.1:$LINK_CMD" \
        -v "$REPO:/work" orrery-cfs:dev \
        bash -c "cd /work/cfs_mission/mission/build/exe/cpu1 && ./container-start")"

    echo "=== [$mode] waiting for gs_tcp + fcs + cam (max ${BOOT_TIMEOUT}s) ==="
    local SECONDS=0
    while [ "$SECONDS" -lt "$BOOT_TIMEOUT" ]; do
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
    done
    logs="$("$RUNTIME" logs "$CONTAINER" 2>&1 || true)"
    if ! grep -q "CAM App Initialized" <<<"$logs"; then
        echo "[$mode] FAILED (boot markers not seen within ${BOOT_TIMEOUT}s)"
        return 1
    fi
    grep "CAM App Initialized" <<<"$logs" | tail -1 | sed 's/^/    /'

    echo "=== [$mode] staging the patient renderer ==="
    RUNTIME="$RUNTIME" bash "$REPO/scripts/renderer.sh" stage "$SCENARIO" "$RENDERER"

    echo "=== [$mode] camera world (lockstep, paced, ${SIM_DURATION} sim-s) ==="
    PYTHONUNBUFFERED=1 uv run python -m sim.forge_scenario --scenario "$SCENARIO" \
        --duration "$SIM_DURATION" --zmq --pace >"$WORK/sim_$mode.log" 2>&1 &
    SIM_PID=$!
    sleep 3
    if ! kill -0 "$SIM_PID" 2>/dev/null; then
        echo "[$mode] world died at startup: a host link port is already taken?" >&2
        tail -3 "$WORK/sim_$mode.log" >&2 || true
        return 1
    fi

    echo "=== [$mode] ground probe (${PROBE_DURATION}s) ==="
    CYCLE_RC=0
    uv run python -m sim.cam_probe --mode "$mode" --duration "$PROBE_DURATION" \
        >"$WORK/probe_$mode.log" 2>&1 || CYCLE_RC=$?
    tail -6 "$WORK/probe_$mode.log" | sed 's/^/    /'

    wait "$SIM_PID"; SIM_PID=""
    CYCLE_DIGEST="$(grep -oE 'trajectory sha256 [0-9a-f]{64}' "$WORK/sim_$mode.log" | tail -1 | awk '{print $3}')"
    [ -n "$CYCLE_DIGEST" ] || { echo "[$mode] FAILED: no digest printed"; return 1; }

    CYCLE_LOG="$WORK/console_$mode.log"
    "$RUNTIME" logs "$CONTAINER" >"$CYCLE_LOG" 2>&1 || true
    "$RUNTIME" stop "$CONTAINER" >/dev/null 2>&1 || true; CONTAINER=""
    "$RUNTIME" rm -f "$RENDERER" >/dev/null 2>&1 || true
    echo "=== [$mode] digest ${CYCLE_DIGEST:0:16}... probe rc=$CYCLE_RC ==="
}

# --- control leg: nothing commanded, the link merely alive ------------------
fly_cycle control || { echo "CAM V2A ACCEPTANCE FAILED (control leg)"; exit 1; }
CONTROL_DIGEST="$CYCLE_DIGEST"; [ "$CYCLE_RC" -eq 0 ] || { echo "CAM V2A ACCEPTANCE FAILED (control probe)"; exit 1; }

# --- spam leg: the cadence, witnessed from both views -----------------------
fly_cycle spam || { echo "CAM V2A ACCEPTANCE FAILED (spam leg)"; exit 1; }
SPAM_DIGEST="$CYCLE_DIGEST"; [ "$CYCLE_RC" -eq 0 ] || { echo "CAM V2A ACCEPTANCE FAILED (spam probe)"; exit 1; }

# --- the console receipts (EVs are the FSW's own testimony) -----------------
fail=0
need() {  # need <pattern> <min count> — EV lines the spam flight must contain
    local n
    n="$(grep -cE "$1" "$WORK/console_spam.log" || true)"
    if [ "$n" -lt "$2" ]; then
        echo "  missing: '$1' seen $n, want >= $2"; fail=1
    else
        echo "  ok: $(grep -E "$1" "$WORK/console_spam.log" | tail -1 | sed 's/^.*App[0-9]*//')"
    fi
}
echo "=== EVS and FIT receipts (spam flight) ==="
need "CAM App Initialized" 1
need "camera 1 configured" 3
need "SET_CONFIG rejected" 2
need "cam config forwarded" 4
need "camera 7 configured" 1
need "FIT MATCHED" 1
need "CamState" 1
# The sim door's own testimony: the ghost-eye order passed FSW (it cannot
# know the fit) and died at the device - CAM_HK said camera=7, the module
# door answered "not fitted". This line is the divergence pair's receipt.
if ! grep -q "eye 7 not fitted" "$WORK/sim_spam.log"; then
    echo "  missing: sim-side refusal of the ghost eye (the divergence exhibit)"; fail=1
else
    echo "  ok: sim refused the ghost eye"
fi

echo "=== digest pair ==="
echo "  control ${CONTROL_DIGEST}"
echo "  spam    ${SPAM_DIGEST}"
if [ "$CONTROL_DIGEST" != "$SPAM_DIGEST" ]; then
    echo "  the commanding MOVED the physics: that is the one sin"; fail=1
fi

if [ "$fail" -eq 0 ]; then echo "CAM V2A ACCEPTANCE OK"; else echo "CAM V2A ACCEPTANCE FAILED"; exit 1; fi
