#!/usr/bin/env bash
# P6 round trip: a time-tagged command executed by the flight, frame-exact.
#
# Same rig as fly_world.sh (paced lockstep, chief operations plant), with
# the closed-loop acceptance replaced by sim/scheduled_flight.py: the ground
# uploads schedules anchored to ABSOLUTE sim frames (FCS ENABLE at T=40,
# SET_GAIN at T=70, a late entry, three refusals), then watches the beacon
# prove the queue fired on the first frame >= T and nothing moved before it.
# The digest is therefore a function of the sim timeline alone - run this
# script twice and the trajectory sha256 must be identical, which is the P6
# determinism exit ("schedules make the future reproducible, not racy").
#
# Run it TWICE and compare digests:
#   bash scripts/fly_scheduled.sh 2>&1 | tee /tmp/opencode/scheduled_run1.log
#   bash scripts/fly_scheduled.sh 2>&1 | tee /tmp/opencode/scheduled_run2.log
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# The yard must fly what this tree built: refuse a stale mission binary
# before the container starts (scripts/firmware.sh).
bash "$REPO/scripts/firmware.sh" check
RUNTIME="${RUNTIME:-docker}"
BOOT_TIMEOUT="${BOOT_TIMEOUT:-120}"
SIM_DURATION="${SIM_DURATION:-100}"
LOG="${LOG:-/tmp/opencode/scheduled_scenario_forge.log}"

# One vehicle on the legacy port block (same convention as fly_world.sh).
INSTANCE=0
BEACON=$((5050 + 2 * INSTANCE))
UPLINK=$((BEACON + 1))
SENSOR=$((5580 + 20 * INSTANCE))
LINK_CMD=$((SENSOR + 1))
SCENARIO="${SCENARIO:-sim/scenarios/chief.toml}"

cd "$REPO"

stale="$("$RUNTIME" ps -q --filter ancestor=orrery-cfs:dev || true)"
[ -n "$stale" ] && "$RUNTIME" stop $stale >/dev/null || true

# A stopping container keeps its bound ports for docker's stop grace - the
# mission's listeners retry, the host sim's bind does not. This sweep also
# takes down an operational fleet (same image, same legacy port block: that
# is the shared-port trade D9 accepted; restore with scripts/fleet.sh up).
# Wait for the ports to actually free before anything binds.
wait_port_free() {
    local port="$1" _
    for _ in $(seq 1 30); do
        timeout 1 bash -c "exec 3<>/dev/tcp/127.0.0.1/$port" 2>/dev/null || return 0
        sleep 1
    done
    echo "port $port still occupied after 30s (another mission holds the block?)" >&2
    return 1
}
wait_port_free "$BEACON"
wait_port_free "$SENSOR"

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

echo "--- waiting for gs_tcp + fcs + sps (max ${BOOT_TIMEOUT}s) ---"
SECONDS=0
while [ "$SECONDS" -lt "$BOOT_TIMEOUT" ]; do
    LOGS="$("$RUNTIME" logs "$CONTAINER" 2>&1 || true)"
    if grep -q "gs_tcp downlink :$BEACON uplink :$UPLINK" <<<"$LOGS" \
        && grep -q "FCS App Initialized" <<<"$LOGS" \
        && grep -q "SPS App Initialized" <<<"$LOGS"; then
        break
    fi
    if grep -qE "(GS_TCP|FCS|BSK_LINK|SPS).*(fail|DEAD)|Could not load file|ES Load Module Error" <<<"$LOGS"; then
        echo "P6 SCHEDULED ROUND TRIP FAILED (app init error)"
        grep -E "GS_TCP|FCS|BSK_LINK|SPS|LoadModule|Could not load|EVS Port1.*66/1/ES" <<<"$LOGS" | tail -12
        exit 1
    fi
    sleep 1
done

LOGS="$("$RUNTIME" logs "$CONTAINER" 2>&1 || true)"
if ! grep -q "gs_tcp downlink :$BEACON uplink :$UPLINK" <<<"$LOGS" \
    || ! grep -q "SPS App Initialized" <<<"$LOGS"; then
    echo "P6 SCHEDULED ROUND TRIP FAILED (boot markers not seen within ${BOOT_TIMEOUT}s)"
    grep -E "GS_TCP|FCS|BSK_LINK|SPS|Could not load" <<<"$LOGS" | tail -12
    exit 1
fi

echo "--- app console ---"
"$RUNTIME" logs "$CONTAINER" 2>&1 | grep -E "GS_TCP|FCS |BSK_LINK|SPS " | head -10

echo "--- basilisk scenario (lockstep, paced, ${SIM_DURATION} sim-s, $SCENARIO) ---"
uv run python -m sim.forge_scenario --duration "$SIM_DURATION" --zmq --pace --scenario "$SCENARIO" \
    >"$LOG" 2>&1 &
SIM_PID=$!
sleep 3  # let the sim bind and pace a couple of ticks before ground connects

echo "--- scheduled-execution ground client ---"
rc=0
uv run python -m sim.scheduled_flight --instance "$INSTANCE" || rc=$?

# On a PASS let the scenario FINISH: the trajectory sha256 is the P6
# determinism artifact, and a killed run has none.
sim_rc=0
if kill -0 "$SIM_PID" 2>/dev/null; then
    if [ "$rc" -eq 0 ]; then
        echo "--- schedules proven; pacing the scenario to its finish (digest) ---"
        deadline=$((SECONDS + SIM_DURATION + 30))
        while kill -0 "$SIM_PID" 2>/dev/null && [ "$SECONDS" -lt "$deadline" ]; do sleep 1; done
    else
        kill "$SIM_PID" 2>/dev/null || true
    fi
fi
wait "$SIM_PID" && sim_rc=0 || sim_rc=$?
SIM_PID=""
echo "--- scenario (rc=$sim_rc) ---"
grep -E "link:|trajectory|DETUMBLE" "$LOG" | tail -3
if [ "$sim_rc" -eq 2 ]; then
    echo "LOCKSTEP GATE DEAD: fcs stopped answering frames (see mission console)"
fi
if [ "$rc" -eq 0 ] && [ "$sim_rc" -ne 0 ]; then rc="$sim_rc"; fi

echo "--- mission console tail (SPS/FCS lifecycle) ---"
"$RUNTIME" logs "$CONTAINER" 2>&1 | grep -E "SPS |scheduled|schedule refused|FCS .*ENABLED|FCS .*DISABLED|control loop" | tail -16

if [ "$rc" -eq 0 ]; then
    grep -E "trajectory" "$LOG" | tail -1
    echo "P6 SCHEDULED ROUND TRIP OK"
else
    echo "P6 SCHEDULED ROUND TRIP FAILED"
fi
exit "$rc"
