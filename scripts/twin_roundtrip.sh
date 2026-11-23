#!/usr/bin/env bash
# Twin fleet round trip (D10 / R2): ONE Basilisk world, N mission containers.
#
# The scenario file IS the fleet: every [[spacecraft]] block gets a container
# (named for the vehicle) on its derived port block, and the world steps only
# when EVERY container acks - so one dead mission must surface as a named
# LOCKSTEP STALL (sim rc=2), never as silent coasting.
#
#   scripts/twin_roundtrip.sh [scenario.toml]     default: sim/scenarios/twin_ops.toml
#
# Per vehicle the closed loop is the same P3 acceptance (sense -> ENABLE ->
# exact-law frame-matched torque -> echo proof -> retune), driven SEQUENTIALLY:
# while vehicle A is tested the others sit in sense mode answering zero torque,
# which is exactly what the shared gate tolerates. The sim runs to its finish
# so the per-vehicle trajectory digests (the determinism artifact) are printed.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# The yard must fly what this tree built: refuse a stale mission binary
# before the container starts (scripts/firmware.sh).
bash "$REPO/scripts/firmware.sh" check
RUNTIME="${RUNTIME:-docker}"
BOOT_TIMEOUT="${BOOT_TIMEOUT:-120}"
LOG_DIR="${LOG_DIR:-/tmp/opencode}"

cd "$REPO"
SCENARIO="${1:-sim/scenarios/twin_ops.toml}"

# The p3 stage sequence advances on TELEMETRY COUNTS, so its ENABLE/SET_GAIN
# frames repeat only if the client's whole run is frame-anchored: every
# vehicle passes --start-frame, the client holds ENABLE until that ABSOLUTE
# sim frame appears on its own beacon, pinning the entire count-driven
# sequence to the same frames every flight. An anchor the client cannot
# reach in time is a hard failure inside the client.
#
# KN2 amendment (2026-11-22) - vehicle 0 is anchored too, and the old
# "chief's digest is the classic-cadence artifact and stays so" stance is
# retired BY MEASUREMENT: with the plant receiver aboard and the host busy
# right after a restart, chief's ENABLE landed on a different sim frame and
# the digest silently moved (01c03187... -> 46a773d...); three quiet flights
# brought the old number back exactly. A wall-clock ENABLE is stable by
# luck, not construction - the same disease the deputy caught at KN, so the
# same cure. Anchors now step by START_EVERY per vehicle so each client
# joins BEFORE its own frame (they run sequentially).
FIRST_START=60
START_EVERY=70

# KN doctrine - the anchor alone was half the story: the p3 client's
# SET_GAIN used to ride the arrival of the 7.8 s FCS_HK poll (a wall-clock
# cadence), which left ONE act to chance and made the deputy baseline
# bistable (85241e3a... / d5c2dbb6..., 50/50). Every vehicle therefore
# sends SET_GAIN at GAIN_AFTER sim frames past its own ENABLE anchor
# (--gain-frame, the p3-solo cure).
GAIN_AFTER=7

# Derived fleet (D10): index = instance = port block, name = container suffix.
mapfile -t FACTS < <(uv run python -c "
from sim.fleet import scenario_vehicles
for v in scenario_vehicles('$SCENARIO'):
    p = v.ports
    print(v.name, v.instance, p.beacon, p.uplink, p.sensor, p.link_cmd)
")
if [ "${#FACTS[@]}" -lt 2 ]; then
    echo "twin_roundtrip: $SCENARIO needs >= 2 [[spacecraft]] blocks (single-vehicle flies via p3_roundtrip.sh)" >&2
    exit 2
fi

# Anchor plan: vehicle i anchors ENABLE at frame FIRST_START +
# START_EVERY*i (sequential clients join well before their own frame).
# That sequence needs ~60 sim-s past its anchor, so the world must
# outlive the last one.
LAST_INDEX=$((${#FACTS[@]} - 1))
LAST_START=$((FIRST_START + START_EVERY * LAST_INDEX))
NEED_DURATION=$((LAST_START + 90))
SIM_DURATION="${SIM_DURATION:-$((LAST_START + 120))}"
if [ "$SIM_DURATION" -lt "$NEED_DURATION" ]; then
    echo "twin_roundtrip: SIM_DURATION=$SIM_DURATION ends before the last anchored sequence finishes (${NEED_DURATION}s)" >&2
    exit 2
fi

CONTAINERS=()
SIM_PID=""
cleanup() {
    [ -n "$SIM_PID" ] && kill "$SIM_PID" >/dev/null 2>&1 || true
    for name in "${CONTAINERS[@]:-}"; do
        [ -n "$name" ] && "$RUNTIME" stop "$name" >/dev/null 2>&1 || true
    done
}
trap cleanup EXIT

echo "--- fleet from $SCENARIO: ${#FACTS[@]} vehicles, one shared world ---"
for fact in "${FACTS[@]}"; do
    read -r name instance beacon uplink sensor link_cmd <<<"$fact"
    echo "--- [$name] instance $instance: beacon :$beacon uplink :$uplink link :$sensor/$link_cmd ---"
    "$RUNTIME" rm -f "orrery-$name" >/dev/null 2>&1 || true
    "$RUNTIME" run -d --name "orrery-$name" --network host \
        --user "$(id -u):$(id -g)" -e HOME=/tmp \
        -e ORRERY_TLM_PORT="$beacon" -e ORRERY_CMD_PORT="$uplink" \
        -e ORRERY_ZMQ_SENSOR_ENDPOINT="tcp://127.0.0.1:$sensor" \
        -e ORRERY_ZMQ_CMD_ENDPOINT="tcp://127.0.0.1:$link_cmd" \
        -v "$REPO:/work" orrery-cfs:dev \
        bash -c "cd /work/cfs_mission/mission/build/exe/cpu1 && ./container-start" >/dev/null
    CONTAINERS+=("orrery-$name")
done

# Every container must boot before the world starts: a missing one would look
# like a gate stall three ticks in, and the operator deserves the named truth.
for fact in "${FACTS[@]}"; do
    read -r name instance beacon uplink sensor link_cmd <<<"$fact"
    SECONDS=0
    booted=0
    while [ "$SECONDS" -lt "$BOOT_TIMEOUT" ]; do
        LOGS="$("$RUNTIME" logs "orrery-$name" 2>&1 || true)"
        if grep -q "gs_tcp downlink :$beacon uplink :$uplink" <<<"$LOGS" &&
            grep -q "FCS App Initialized" <<<"$LOGS"; then
            booted=1
            break
        fi
        if grep -qE "(GS_TCP|FCS|BSK_LINK).*(fail|DEAD)|Could not load file|ES Load Module Error" <<<"$LOGS"; then
            echo "TWIN ROUND TRIP FAILED ([$name] app init error)"
            grep -E "GS_TCP|FCS|BSK_LINK|LoadModule|Could not load" <<<"$LOGS" | tail -8
            exit 1
        fi
        sleep 1
    done
    if [ "$booted" -ne 1 ]; then
        echo "TWIN ROUND TRIP FAILED ([$name] boot markers not seen within ${BOOT_TIMEOUT}s)"
        exit 1
    fi
    echo "[$name] mission up"
done

LOG="$LOG_DIR/twin_sim.log"
echo "--- basilisk fleet sim (lockstep, paced, ${SIM_DURATION} sim-s) ---"
uv run python -m sim.forge_scenario --scenario "$SCENARIO" --duration "$SIM_DURATION" \
    --zmq --pace >"$LOG" 2>&1 &
SIM_PID=$!
sleep 3
# a standing fleet holds the link block: the host sim dies silently while
# the missions (CONNECT clients) boot happy and measure their own starvation
if ! kill -0 "$SIM_PID" 2>/dev/null; then
    echo "world died at startup: a host link port is already taken?" >&2
    echo "  (sweep the yard: scripts/fleet.sh down) last lines:" >&2
    tail -3 "$LOG" >&2 || true
    exit 1
fi

index=0
for fact in "${FACTS[@]}"; do
    read -r name instance beacon uplink sensor link_cmd <<<"$fact"
    start_frame=$((FIRST_START + START_EVERY * index))
    gain_frame=$((start_frame + GAIN_AFTER))
    echo "--- closed-loop ground client: $name (instance $instance, start-frame $start_frame, gain-frame $gain_frame) ---"
    if ! uv run python -m sim.p3_fcs_test --instance "$instance" --start-frame "$start_frame" \
        --gain-frame "$gain_frame"; then
        echo "TWIN ROUND TRIP FAILED (closed loop: $name)"
        [ -n "$SIM_PID" ] && kill "$SIM_PID" 2>/dev/null || true
        exit 1
    fi
    index=$((index + 1))
done

# All gates proven live; let the paced world run to its finish for digests.
echo "--- fleet gates proven; pacing to the finish (digests) ---"
deadline=$((SECONDS + SIM_DURATION + 60))
while kill -0 "$SIM_PID" 2>/dev/null && [ "$SECONDS" -lt "$deadline" ]; do sleep 1; done
sim_rc=0
wait "$SIM_PID" && sim_rc=0 || sim_rc=$?
SIM_PID=""
echo "--- fleet sim (rc=$sim_rc) ---"
grep -E "link:|sha256|FLEET RUN|STALL" "$LOG" | tail -6
if [ "$sim_rc" -eq 2 ]; then
    echo "TWIN ROUND TRIP FAILED (LOCKSTEP STALL: a container stopped acking - named in the log)"
    exit 1
fi
if [ "$sim_rc" -ne 0 ]; then
    echo "TWIN ROUND TRIP FAILED (sim rc=$sim_rc, see $LOG)"
    exit 1
fi

# Digest discipline (policy after the 2026-11-22 wipe AND the d5c2dbb6
# bistable era): a printed digest nobody compares is folklore. Every
# vehicle's digest is checked against tests/baselines/twin_digest_<name>.txt
# - the first flight of a new world FINDS the baseline (visible in the log),
# any later move FAILS loudly and demands a re-found plus a journal line.
BASE="tests/baselines"
mkdir -p "$BASE"
for fact in "${FACTS[@]}"; do
    read -r name instance _ <<<"$fact"
    dig="$(grep -oE "${name} trajectory sha256 [0-9a-f]{64}" "$LOG" | tail -1 | cut -d' ' -f4)"
    if [ -z "$dig" ]; then
        echo "TWIN ROUND TRIP FAILED (no digest printed for $name)"
        exit 1
    fi
    file="$BASE/twin_digest_${name}.txt"
    if [ ! -f "$file" ]; then
        printf '%s\n' "$dig" >"$file"
        echo "[$name] baseline FOUND at $file (first flight of this world)"
    elif [ "$(cat "$file")" != "$dig" ]; then
        echo "[$name] digest MOVED: baseline $(cat "$file") -> flight $dig"
        echo "    anchored cadence must not move - if physics legitimately changed, re-found and journal it."
        exit 1
    else
        echo "[$name] digest $dig matches baseline"
    fi
done
echo "TWIN ROUND TRIP OK"
