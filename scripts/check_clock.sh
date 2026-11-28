#!/usr/bin/env bash
# Clock run: the mission clock is the sensor feed's, at both paces.
#
# Same bones as check_modes.sh (mission up, plant in lockstep, ground client
# reads, trap cleans up) but the client measures instead of commanding: the
# OBC's housekeeping pairs each sensor frame's SimTimeS with the mission
# clock as CFE_TIME reads it in that frame's OWN wake, so the verdict is the
# wake-time skew every frame consumer inherits - the number the SC arc will
# schedule on. Two legs because the law must hold at both paces: paced (wall
# = sim, the fleet's default) and UNPACED (the hohmann pace, ~2x, where a
# wall-drifting clock would show). sync-after-publish fails BOTH by a whole
# tick; ms-level skews pass both.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# The yard must fly what this tree built: refuse a stale mission binary
# before the container starts (scripts/firmware.sh).
bash "$REPO/scripts/firmware.sh" check
RUNTIME="${RUNTIME:-docker}"
BOOT_TIMEOUT="${BOOT_TIMEOUT:-120}"
SIM_DURATION="${SIM_DURATION:-45}"
SIM_MODULE="${SIM_MODULE:-sim.forge_scenario}"
SIM_ARGS="${SIM_ARGS:---scenario sim/scenarios/chief.toml}"
LOG="${LOG:-/tmp/opencode/clock_probe_scenario.log}"

# One vehicle on the legacy port block; a live fleet squats on the same
# numbers, so start from a quiet sky.
bash "$REPO/scripts/fleet.sh" down sim/scenarios/twin_ops.toml >/dev/null 2>&1 || true

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
        echo "CLOCK FAILED (app init error)"
        grep -E "GS_TCP|FCS|BSK_LINK|LoadModule|Could not load|EVS Port1.*66/1/ES" <<<"$LOGS" | tail -12
        exit 1
    fi
    sleep 1
done

LOGS="$("$RUNTIME" logs "$CONTAINER" 2>&1 || true)"
if ! grep -q "gs_tcp downlink :$BEACON uplink :$UPLINK" <<<"$LOGS" || ! grep -q "FCS App Initialized" <<<"$LOGS"; then
    echo "CLOCK FAILED (boot markers not seen within ${BOOT_TIMEOUT}s)"
    grep -E "GS_TCP|FCS|BSK_LINK|Could not load" <<<"$LOGS" | tail -12
    exit 1
fi

# Two legs, one mission: the law does not get to choose its pace.
for pace_flag in "--pace" ""; do
    [ -n "$pace_flag" ] && leg="paced" || leg="UNPACED"
    echo "--- basilisk scenario ($leg, ${SIM_DURATION} sim-s) ---"
    uv run python -m "$SIM_MODULE" --duration "$SIM_DURATION" --zmq $pace_flag $SIM_ARGS >"$LOG" 2>&1 &
    SIM_PID=$!
    sleep 3

    echo "--- clock acceptance client ($leg) ---"
    rc=0
    uv run python -m sim.clock_probe --instance "$INSTANCE" || rc=$?

    sim_rc=0
    # The scenario is ours to stop when the client finishes or fails; only a
    # scenario that died on its own (dead lockstep gate, crash) counts.
    sleep 2
    sim_alive=0
    if kill -0 "$SIM_PID" 2>/dev/null; then
        sim_alive=1
        kill "$SIM_PID" 2>/dev/null || true
    fi
    wait "$SIM_PID" 2>/dev/null && sim_rc=0 || sim_rc=$?
    if [ "$sim_alive" -eq 1 ]; then sim_rc=0; fi
    SIM_PID=""
    echo "--- scenario ($leg, rc=$sim_rc) ---"
    if [ "$sim_rc" -eq 2 ]; then
        echo "LOCKSTEP GATE DEAD (see mission console)"
    fi
    if [ "$rc" -ne 0 ]; then
        echo "--- mission console tail (the link's own words) ---"
        "$RUNTIME" logs "$CONTAINER" 2>&1 | grep -E "BSK_LINK|TIME" | tail -8
        exit "$rc"
    fi
    if [ "$sim_rc" -ne 0 ]; then exit "$sim_rc"; fi
done

echo "--- mission console tail (the link's own words) ---"
"$RUNTIME" logs "$CONTAINER" 2>&1 | grep -E "BSK_LINK|TIME" | tail -6

echo "CLOCK RUN OK"
