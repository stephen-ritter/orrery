#!/usr/bin/env bash
# Fuel-rung acceptance: the tank sells, the world withholds, both ledgers tell it.
#
# Same bones as check_burn.sh (mission up, plant in lockstep, ground client
# drives, trap cleans up) but the plant is sim/scenarios/fuel.toml - the burn
# vehicle WITH a declared 50 g tank - and the client is sim/fuel_probe.py:
# full-service burn while fuel is plentiful -> the over-ask the tank cannot
# price -> EXACT zero stock, throttled then starved ticks on the truth side,
# an unbroken claim on the craft side -> a fresh quota against the dry tank
# that the world refuses while the craft's clock still closes it. The control
# loop stays DISABLED throughout: this bench grades fuel physics, and the
# burn door is the ground's, exactly as the Rung-1 bench established.
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
SIM_ARGS="${SIM_ARGS:---scenario sim/scenarios/fuel.toml}"
LOG="${LOG:-/tmp/opencode/fuel_probe_scenario.log}"

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
        echo "FUEL RUN FAILED (app init error)"
        grep -E "GS_TCP|FCS|BSK_LINK|LoadModule|Could not load|EVS Port1.*66/1/ES" <<<"$LOGS" | tail -12
        exit 1
    fi
    sleep 1
done

LOGS="$("$RUNTIME" logs "$CONTAINER" 2>&1 || true)"
if ! grep -q "gs_tcp downlink :$BEACON uplink :$UPLINK" <<<"$LOGS" || ! grep -q "FCS App Initialized" <<<"$LOGS"; then
    echo "FUEL RUN FAILED (boot markers not seen within ${BOOT_TIMEOUT}s)"
    grep -E "GS_TCP|FCS|BSK_LINK|Could not load" <<<"$LOGS" | tail -12
    exit 1
fi

echo "--- basilisk fuel plant (lockstep, paced, ${SIM_DURATION} sim-s) ---"
uv run python -m "$SIM_MODULE" --duration "$SIM_DURATION" --zmq --pace $SIM_ARGS >"$LOG" 2>&1 &
SIM_PID=$!
sleep 3

echo "--- fuel acceptance ground client ---"
rc=0
uv run python -m sim.fuel_probe --instance "$INSTANCE" || rc=$?

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
    echo "FUEL RUN OK"
else
    echo "FUEL RUN FAILED"
fi
exit "$rc"
