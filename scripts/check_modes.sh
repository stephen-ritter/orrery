#!/usr/bin/env bash
# P4b-2 modes run: the mode supervisor trips, refuses, and recovers, live.
#
# Same bones as fly_world.sh (mission up, plant in lockstep, ground client
# drives, trap cleans up) but the default plant is the chief TOML - the
# supervisor feeds on ONBOARD frames, which only that plant publishes - and
# the client is sim/p4b_modes_test.py: ENABLE -> SET_WHEEL_LIMIT 10 -> SAFE
# (bus silent, echo zero, gate alive) -> ENABLE refused -> limit 600 + ENABLE
# -> law-exact torque again. A dead lockstep gate still exits the scenario
# with rc=2: SAFE stops the LAW, never the ACKS, and that is what this proves
# as much as the mode machine itself.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# The yard must fly what this tree built: refuse a stale mission binary
# before the container starts (scripts/firmware.sh).
bash "$REPO/scripts/firmware.sh" check
RUNTIME="${RUNTIME:-docker}"
BOOT_TIMEOUT="${BOOT_TIMEOUT:-120}"
SIM_DURATION="${SIM_DURATION:-120}"
SIM_MODULE="${SIM_MODULE:-sim.forge_scenario}"
SIM_ARGS="${SIM_ARGS:---scenario sim/scenarios/chief.toml}"
LOG="${LOG:-/tmp/opencode/p4b_modes_scenario.log}"

# D10: this harness flies ONE vehicle on the legacy port block (modes are
# proven on the flight loop itself); the D9 BIRD=<slug> manifest mode died
# with the manifest layer - fleet work is scripts/fly_fleet.sh.
INSTANCE=0
BEACON=$((5050 + 2 * INSTANCE))
UPLINK=$((BEACON + 1))
SENSOR=$((5580 + 20 * INSTANCE))
LINK_CMD=$((SENSOR + 1))

cd "$REPO"

stale="$("$RUNTIME" ps -q --filter ancestor=orrery-cfs:dev || true)"
[ -n "$stale" ] && "$RUNTIME" stop $stale >/dev/null || true

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
        echo "P4b MODES FAILED (app init error)"
        grep -E "GS_TCP|FCS|BSK_LINK|LoadModule|Could not load|EVS Port1.*66/1/ES" <<<"$LOGS" | tail -12
        exit 1
    fi
    sleep 1
done

LOGS="$("$RUNTIME" logs "$CONTAINER" 2>&1 || true)"
if ! grep -q "gs_tcp downlink :$BEACON uplink :$UPLINK" <<<"$LOGS" || ! grep -q "FCS App Initialized" <<<"$LOGS"; then
    echo "P4b MODES FAILED (boot markers not seen within ${BOOT_TIMEOUT}s)"
    grep -E "GS_TCP|FCS|BSK_LINK|Could not load" <<<"$LOGS" | tail -12
    exit 1
fi

echo "--- basilisk scenario (lockstep, paced, ${SIM_DURATION} sim-s) ---"
uv run python -m "$SIM_MODULE" --duration "$SIM_DURATION" --zmq --pace $SIM_ARGS >"$LOG" 2>&1 &
SIM_PID=$!
sleep 3

echo "--- mode-supervisor ground client ---"
rc=0
uv run python -m sim.p4b_modes_test --instance "$INSTANCE" || rc=$?

sim_rc=0
# A modes run mutates the plant on purpose, so no digest artifact is waited
# for: give a dead gate two seconds to surface as the scenario's own rc=2,
# then stop it. The gate MUST stay alive through SAFE - that is half the
# proof - and rc=2 on any path says it was not. A scenario WE stopped is not
# a failure; only its own exit codes (dead gate, crash) count.
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
    echo "LOCKSTEP GATE DEAD: SAFE must stop the law, never the acks (see mission console)"
fi
if [ "$rc" -eq 0 ] && [ "$sim_rc" -ne 0 ]; then rc="$sim_rc"; fi

echo "--- mission console tail (the supervisor's own words) ---"
"$RUNTIME" logs "$CONTAINER" 2>&1 | grep -E "FCS " | tail -8

if [ "$rc" -eq 0 ]; then
    echo "P4b MODES RUN OK"
else
    echo "P4b MODES RUN FAILED"
fi
exit "$rc"
