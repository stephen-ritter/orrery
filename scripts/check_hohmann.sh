#!/usr/bin/env bash
# Hohmann-rung acceptance: fuel budgets a transfer and the orbit is the verdict.
#
# Same bones as check_fuel.sh with ONE deliberate difference: NO --pace.
# A Hohmann coast is half an orbit (~48 sim-minutes) and the physics will
# not hurry; pacing would make this a 48-minute bench for no added evidence.
# Unpaced, lockstep still owns causality (the world advances only on acks,
# the check trips on a stale frame exactly as paced) and runs ~2x wall:
# expect roughly 25-30 minutes for the whole transfer, the longest bench
# row by design. The client is sim/hohmann_probe.py: prograde leg -> MEASURE
# the raised apoapsis and catch the radial-velocity flip on the truth stream
# -> circularize at the detected apoapsis -> confirmation coast holds r ->
# an over-priced third ask the surviving fuel cannot buy meets the dry door
# while the claim ledger keeps rising.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# The yard must fly what this tree built: refuse a stale mission binary
# before the container starts (scripts/firmware.sh).
bash "$REPO/scripts/firmware.sh" check
RUNTIME="${RUNTIME:-docker}"
BOOT_TIMEOUT="${BOOT_TIMEOUT:-120}"
SIM_DURATION="${SIM_DURATION:-inf}"  # perpetual: the BENCH is the verdict (detumble
                                     # convergence is not this world's story - the
                                     # loop never ENABLEs; the probe is the judge)
SIM_MODULE="${SIM_MODULE:-sim.forge_scenario}"
SIM_ARGS="${SIM_ARGS:---scenario sim/scenarios/hohmann.toml}"
LOG="${LOG:-/tmp/opencode/hohmann_probe_scenario.log}"

INSTANCE=0
BEACON=$((5050 + 2 * INSTANCE))
UPLINK=$((BEACON + 1))
SENSOR=$((5580 + 20 * INSTANCE))
LINK_CMD=$((SENSOR + 1))

cd "$REPO"

# A quiet yard is a precondition: this bench's world binds the classic port
# block and its craft boot on the union wake list, so a standing fleet - or
# its world alone, holding 5580 while every craft container is gone - kills
# the plant at bind time (this bench's own first flight died exactly there,
# two days before the doctrine commit named it). fleet down is idempotent.
bash "$REPO/scripts/fleet.sh" down >/dev/null 2>&1 || true
stale="$("$RUNTIME" ps -q --filter ancestor=orrery-cfs:dev || true)"
[ -n "$stale" ] && "$RUNTIME" rm -f $stale >/dev/null || true

CONTAINER=""
SIM_PID=""
cleanup() {
    [ -n "$SIM_PID" ] && kill "$SIM_PID" >/dev/null 2>&1 || true
    [ -n "$CONTAINER" ] && "$RUNTIME" stop "$CONTAINER" >/dev/null 2>&1 || true
}
trap cleanup EXIT

CONTAINER="$("$RUNTIME" run -d --network host --user "$(id -u):$(id -g)" -e HOME=/tmp \
    -e ORRERY_TLM_PORT="$BEACON" -e ORRERY_CMD_PORT="$UPLINK" \
    -e ORRERY_ZMQ_SENSOR_ENDPOINT="tcp://127.0.0.1:$SENSOR" \
    -e ORRERY_ZMQ_CMD_ENDPOINT="tcp://127.0.0.1:$LINK_CMD" \
    -v "$REPO:/work" orrery-cfs:dev \
    bash -c "cd /work/cfs_mission/mission/build/exe/cpu1 && ./container-start")"

echo "--- waiting for gs_tcp + fcs (max ${BOOT_TIMEOUT}s) ---"
SECONDS=0
while [ "$SECONDS" -lt "$BOOT_TIMEOUT" ]; do
    LOGS="$("$RUNTIME" logs "$CONTAINER" 2>&1 || true)"
    if grep -q "gs_tcp downlink :$BEACON uplink :$UPLINK" <<<"$LOGS" && grep -q "FCS App Initialized" <<<"$LOGS"; then
        break
    fi
    if grep -qE "(GS_TCP|FCS|BSK_LINK).*(fail|DEAD)|Could not load file|ES Load Module Error" <<<"$LOGS"; then
        echo "HOHMANN RUN FAILED (app init error)"
        grep -E "GS_TCP|FCS|BSK_LINK|LoadModule|Could not load|EVS Port1.*66/1/ES" <<<"$LOGS" | tail -12
        exit 1
    fi
    sleep 1
done

LOGS="$("$RUNTIME" logs "$CONTAINER" 2>&1 || true)"
if ! grep -q "gs_tcp downlink :$BEACON uplink :$UPLINK" <<<"$LOGS" || ! grep -q "FCS App Initialized" <<<"$LOGS"; then
    echo "HOHMANN RUN FAILED (boot markers not seen within ${BOOT_TIMEOUT}s)"
    grep -E "GS_TCP|FCS|BSK_LINK|Could not load" <<<"$LOGS" | tail -12
    exit 1
fi

echo "--- hohmann plant (lockstep, UNPACED, ${SIM_DURATION} sim-s) ---"
uv run python -m "$SIM_MODULE" --duration "$SIM_DURATION" --zmq $SIM_ARGS >"$LOG" 2>&1 &
SIM_PID=$!
sleep 3

echo "--- hohmann acceptance ground client ---"
rc=0
uv run python -m sim.hohmann_probe --instance "$INSTANCE" || rc=$?

sim_rc=0
# Like the burn bench: the run mutates the plant on purpose (it burns it DRY),
# so no digest artifact is waited for. A scenario WE stopped is not a
# failure; only its own exit codes (dead check, crash, LOCKSTEP STALL) count.
sleep 2
sim_alive=0
if kill -0 "$SIM_PID" 2>/dev/null; then
    sim_alive=1
    kill "$SIM_PID" 2>/dev/null || true
fi
wait "$SIM_PID" 2>/dev/null && sim_rc=0 || sim_rc=$?
if [ "$sim_alive" -eq 1 ]; then sim_rc=0; fi
SIM_PID=""
echo "--- scenario (rc=$sim_rc) ---"
if [ "$sim_rc" -eq 2 ]; then
    echo "LOCKSTEP GATE DEAD: burn frames must never gate the barrier (see mission console)"
fi
if [ "$rc" -eq 0 ] && [ "$sim_rc" -ne 0 ]; then rc="$sim_rc"; fi

echo "--- mission console tail (the craft's own words) ---"
"$RUNTIME" logs "$CONTAINER" 2>&1 | grep -E "FCS |burn" | tail -12

if [ "$rc" -eq 0 ]; then
    echo "HOHMANN RUN OK"
else
    echo "HOHMANN RUN FAILED"
fi
exit "$rc"
