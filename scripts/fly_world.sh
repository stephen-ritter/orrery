#!/usr/bin/env bash
# P3f round trip: Basilisk sim <-> live mission (bsk_link + fcs) <-> ground.
#
# Starts the (already built) mission, waits for both the gs_tcp listeners and
# the FCS init event, launches the Basilisk scenario in LOCKSTEP (--zmq: the
# sim owns the clock, every sensor frame gates on its fcs StepAck; --pace puts
# 1 sim-s per real-s on top so the ground's sampling windows are wall-clock
# sane), then drives the closed-loop acceptance from the host with
# sim/fcs_probe.py: sense (zero answers = inert) -> ENABLE -> exact-law
# torque frame-matched to its sensor frame -> sim echo proves the effector
# felt it -> SET_GAIN live retune. Container and scenario are ALWAYS cleaned
# up by the trap. A dead gate exits the scenario with rc=2 and fails here.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# The yard must fly what this tree built: refuse a stale mission binary
# before the container starts (scripts/firmware.sh).
bash "$REPO/scripts/firmware.sh" check
RUNTIME="${RUNTIME:-docker}"
BOOT_TIMEOUT="${BOOT_TIMEOUT:-120}"
# Paced lockstep: 1 sim-s ~= 1 real-s. The ground test passes well inside this
# window, and the scenario then FINISHES on its own, so its trajectory sha256
# (the P3e determinism artifact) is in the log, not killed mid-run.
SIM_DURATION="${SIM_DURATION:-60}"
# Which plant drives the mission. Default is the hand-written rigid body;
# SIM_MODULE=sim.forge_scenario runs the bskforge-TOML plant through the exact
# same closed loop (W3: same rig, same acceptance, different build path).
SIM_MODULE="${SIM_MODULE:-sim.detumble_scenario}"
# Extra switches for the scenario module, e.g. SIM_ARGS="--scenario
# sim/scenarios/chief.toml" flies the P4a operations plant (wheels + IMU)
# through the exact same closed loop.
SIM_ARGS="${SIM_ARGS:-}"
# Positional scenario (fly_fleet symmetry): fly_world.sh sim/scenarios/x.toml
# runs the bskforge plant through this exact closed loop - and a scenario
# carrying [[camera]] gets the rendered eye (staged below, same law).
if [ -n "${1:-}" ]; then SIM_MODULE="sim.forge_scenario"; SIM_ARGS="--scenario $1"; fi
LOG="${LOG:-/tmp/opencode/world_scenario_$(basename "$SIM_MODULE").log}"

# D10: this harness flies ONE vehicle on the legacy port block. Fleet work
# (N worlds-gated-together missions) lives in scripts/fly_fleet.sh; the
# D9 BIRD=<slug> manifest mode died with the manifest layer.
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
RENDERER=""
cleanup() {
    [ -n "$SIM_PID" ] && kill "$SIM_PID" >/dev/null 2>&1 || true
    [ -n "$RENDERER" ] && "$RUNTIME" rm -f "$RENDERER" >/dev/null 2>&1 || true
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
        echo "P3c ROUND TRIP FAILED (app init error)"
        grep -E "GS_TCP|FCS|BSK_LINK|LoadModule|Could not load|EVS Port1.*66/1/ES" <<<"$LOGS" | tail -12
        exit 1
    fi
    sleep 1
done

# Boot markers confirmed or the timeout expired; expiring MUST fail here, not
# surface as a confusing downstream "stuck in sense" 3 minutes later.
LOGS="$("$RUNTIME" logs "$CONTAINER" 2>&1 || true)"
if ! grep -q "gs_tcp downlink :$BEACON uplink :$UPLINK" <<<"$LOGS" || ! grep -q "FCS App Initialized" <<<"$LOGS"; then
    echo "P3c ROUND TRIP FAILED (boot markers not seen within ${BOOT_TIMEOUT}s)"
    grep -E "GS_TCP|FCS|BSK_LINK|Could not load" <<<"$LOGS" | tail -12
    exit 1
fi

echo "--- app console ---"
"$RUNTIME" logs "$CONTAINER" 2>&1 | grep -E "GS_TCP|FCS |BSK_LINK" | head -10

# The rendered eye (fly_fleet symmetry): a scenario carrying [[camera]]
# stages the patient direct-exec renderer BEFORE the world starts - the
# shared law lives in scripts/renderer.sh, the squat guard included.
SCENARIO="${1:-}"
if [ -z "$SCENARIO" ] && [[ "$SIM_ARGS" == *--scenario* ]]; then SCENARIO="${SIM_ARGS#*--scenario }"; fi
if [ -n "$SCENARIO" ] && grep -qE '^[[:space:]]*\[\[camera\]\]' "$SCENARIO" 2>/dev/null; then
    RUNTIME="$RUNTIME" bash "$REPO/scripts/renderer.sh" stage "$SCENARIO" orrery-world-viz
    RENDERER="orrery-world-viz"
fi

echo "--- basilisk scenario (lockstep, paced, ${SIM_DURATION} sim-s) ---"
# unbuffered: a SWIG-side abort loses a block-buffered log
PYTHONUNBUFFERED=1 uv run python -m "$SIM_MODULE" --duration "$SIM_DURATION" --zmq --pace $SIM_ARGS >"$LOG" 2>&1 &
SIM_PID=$!
sleep 3  # let the sim bind and pace a couple of ticks before ground connects
# A host ZMQ bind collision (a standing fleet owns 5580+) kills the world at
# startup, silently, while the mission - a CONNECT client - boots happy.
# Name the real cause before the ground test starts measuring zeroes.
if ! kill -0 "$SIM_PID" 2>/dev/null; then
    echo "world died at startup: a host link port is already taken?" >&2
    echo "  (sweep the yard: scripts/fleet.sh down) last lines:" >&2
    tail -3 "$LOG" >&2 || true
    exit 1
fi

echo "--- closed-loop ground client ---"
rc=0
# Optional absolute-frame anchors (see sim/fcs_probe.py): the ops-plant
# baseline flights pin ENABLE and SET_GAIN so the digest repeats by
# CONSTRUCTION, not by poll-phase luck. Empty (default) keeps the legacy
# count-driven cadence - the historical witness baseline needs it.
CLIENT_ARGS=()
[ -n "${START_FRAME:-}" ] && CLIENT_ARGS+=(--start-frame "$START_FRAME")
[ -n "${GAIN_FRAME:-}" ] && CLIENT_ARGS+=(--gain-frame "$GAIN_FRAME")
uv run python -m sim.fcs_probe --instance "$INSTANCE" "${CLIENT_ARGS[@]}" || rc=$?

# A lockstep gate that died exits the scenario with rc=2 within ack-timeout;
# surface that as the headline, not as a downstream "stuck in stage" timeout.
# On a PASS the scenario is deliberately LET FINISH: its final trajectory
# sha256 is the P3e determinism artifact, and a killed run has none.
sim_rc=0
if kill -0 "$SIM_PID" 2>/dev/null; then
    if [ "$rc" -eq 0 ]; then
        echo "--- closed loop proven; pacing the scenario to its finish (digest) ---"
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
if [ "$rc" -ne 0 ]; then
    # the greps above are silent when the scenario died at STARTUP (bad args,
    # import error) - and that looked exactly like a world refusing to turn.
    # On failure the raw tail belongs in the output; a stranger should not
    # have to know this log file exists to see why there are no frames.
    [ -s "$LOG" ] && tail -4 "$LOG" | sed 's/^/    sim| /'
fi
if [ "$sim_rc" -eq 2 ]; then
    echo "LOCKSTEP GATE DEAD: fcs stopped answering frames (see mission console)"
fi
if [ "$rc" -eq 0 ] && [ "$sim_rc" -ne 0 ]; then rc="$sim_rc"; fi

echo "--- mission console tail ---"
"$RUNTIME" logs "$CONTAINER" 2>&1 | grep -E "FCS |BSK_LINK|TIME.*MET|slaved to link" | tail -8

if [ "$rc" -eq 0 ]; then
    echo "P3 ROUND TRIP OK"
else
    echo "P3 ROUND TRIP FAILED"
fi
exit "$rc"
