#!/usr/bin/env bash
# Rung-1 burn acceptance: the quota door, the per-tick stream, both ledgers.
#
# Same bones as check_modes.sh (mission up, plant in lockstep, ground client
# drives, trap cleans up) but the plant is sim/scenarios/burn.toml - the
# chief WITH four 5 N CoM-plane nozzles - and the client is
# sim/burn_probe.py: refuse nonsense -> open -> refuse overlap -> watch the
# claim (HK/beacon) and the applied ledger (truth) rise together -> BURNSTOP
# -> refuse an idle stop -> a clock-closed burn -> the honest divergence of
# an over-authority ask. The control loop stays DISABLED the whole run:
# burn authority is the ground's door, not a side effect of the law's mode.
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
SIM_ARGS="${SIM_ARGS:---scenario sim/scenarios/burn.toml}"
LOG="${LOG:-/tmp/opencode/burn_probe_scenario.log}"

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
        echo "BURN RUN FAILED (app init error)"
        grep -E "GS_TCP|FCS|BSK_LINK|LoadModule|Could not load|EVS Port1.*66/1/ES" <<<"$LOGS" | tail -12
        exit 1
    fi
    sleep 1
done

LOGS="$("$RUNTIME" logs "$CONTAINER" 2>&1 || true)"
if ! grep -q "gs_tcp downlink :$BEACON uplink :$UPLINK" <<<"$LOGS" || ! grep -q "FCS App Initialized" <<<"$LOGS"; then
    echo "BURN RUN FAILED (boot markers not seen within ${BOOT_TIMEOUT}s)"
    grep -E "GS_TCP|FCS|BSK_LINK|Could not load" <<<"$LOGS" | tail -12
    exit 1
fi

echo "--- basilisk burn plant (lockstep, paced, ${SIM_DURATION} sim-s) ---"
uv run python -m "$SIM_MODULE" --duration "$SIM_DURATION" --zmq --pace $SIM_ARGS >"$LOG" 2>&1 &
SIM_PID=$!
sleep 3

echo "--- burn acceptance ground client ---"
rc=0
uv run python -m sim.burn_probe --instance "$INSTANCE" || rc=$?

sim_rc=0
# Like the modes bench: the run mutates the plant on purpose (it is a burn!),
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
    echo "BURN RUN OK"
else
    echo "BURN RUN FAILED"
fi
exit "$rc"
