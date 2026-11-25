#!/usr/bin/env bash
# Regenerated-defs wrap: the REGENERATED defs (current gem) in live COSMOS CE.
#
# Prereq: scripts/cosmos_stack.sh has loaded its GEM_VERSION (it rebuilds the
# gem from cosmos_plugin/ and `cli load`s it). This header deliberately names no
# number: it used to claim 0.1.4 while cosmos_stack.sh loaded 0.1.3, which is
# the same class of lie this repo exists to catch. Starts the mission, waits for
# the app markers (gs_tcp + FCS + OBC HK - an unloaded obc_hk must fail HERE, not
# as a missing-packet timeout three minutes later), launches the operations
# plant (chief.toml) paced so ground sampling windows are wall-clock sane,
# then runs sim/beacon_tables.py against the COSMOS JSON-RPC API. Finishes
# by counting UNKNOWN/length faults the operator logged SINCE the client
# started: the beacon table and the defs must match byte-for-byte, both ways.
# SCENARIO picks the world (default chief.toml); a [[camera]] world also
# proves the OPNAV frame decodes while the eye streams.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# The yard must fly what this tree built: refuse a stale mission binary
# before the container starts (scripts/firmware.sh).
bash "$REPO/scripts/firmware.sh" check
COSMOS_HOME="${COSMOS_HOME:-$HOME/Projects/cosmos}"
export OPENC3_API_PASSWORD="${OPENC3_API_PASSWORD:-orrery-dev}"
SIM_DURATION="${SIM_DURATION:-90}"
# Which world feeds the beacon. The default keeps the historic wrap
# (chief.toml, every instrument but the eye); a [[camera]] scenario like
# camera_ops.toml also proves the OPNAV frame decodes with REAL knowledge -
# it gets the staged renderer (scripts/renderer.sh) exactly like fly_world.
SCENARIO="${SCENARIO:-sim/scenarios/chief.toml}"
cd "$REPO"

if ! docker ps --format '{{.Names}}' | grep -q '^cosmos-openc3-operator-1$'; then
    echo "COSMOS CE not running - run scripts/cosmos_stack.sh first" >&2
    exit 1
fi

stale="$(docker ps -q --filter ancestor=orrery-cfs:dev || true)"
[ -n "$stale" ] && docker stop $stale >/dev/null || true

CONTAINER=""
SIM_PID=""
RENDERER=""
cleanup() {
    [ -n "$SIM_PID" ] && kill "$SIM_PID" >/dev/null 2>&1 || true
    [ -n "$RENDERER" ] && docker rm -f "$RENDERER" >/dev/null 2>&1 || true
    [ -n "$CONTAINER" ] && docker stop "$CONTAINER" >/dev/null 2>&1 || true
}
trap cleanup EXIT

CONTAINER="$(docker run -d --network host --user "$(id -u):$(id -g)" -e HOME=/tmp \
    -v "$REPO:/work" orrery-cfs:dev \
    bash -c "cd /work/cfs_mission/mission/build/exe/cpu1 && ./container-start")"

echo "--- waiting for gs_tcp + FCS + OBC HK (max 120s) ---"
SECONDS=0
while [ "$SECONDS" -lt 120 ]; do
    LOGS="$(docker logs "$CONTAINER" 2>&1 || true)"
    if grep -q "gs_tcp downlink :5050 uplink :5051" <<<"$LOGS" &&
        grep -q "FCS App Initialized" <<<"$LOGS" &&
        grep -q "OBC HK App Initialized" <<<"$LOGS"; then
        break
    fi
    if grep -qE "Could not load file|ES Load Module Error" <<<"$LOGS"; then
        echo "COSMOS WRAP FAILED (module load error)"
        grep -E "Could not load|undefined symbol" <<<"$LOGS" | tail -4
        exit 1
    fi
    sleep 1
done
LOGS="$(docker logs "$CONTAINER" 2>&1 || true)"
if ! grep -q "OBC HK App Initialized" <<<"$LOGS"; then
    echo "COSMOS WRAP FAILED (boot markers not seen within 120s)"
    grep -E "GS_TCP|FCS |OBC_HK|Could not load" <<<"$LOGS" | tail -8
    exit 1
fi

# The rendered eye: a [[camera]] world stages the patient renderer BEFORE
# the world starts (shared law: scripts/renderer.sh) so the OPNAV frame
# carries real knowledge for the decode wrap below.
if grep -qE '^[[:space:]]*\[\[camera\]\]' "$SCENARIO" 2>/dev/null; then
    bash "$REPO/scripts/renderer.sh" stage "$SCENARIO" orrery-beacon-viz
    RENDERER="orrery-beacon-viz"
fi

echo "--- basilisk operations plant (paced, ${SIM_DURATION} sim-s, $SCENARIO) ---"
PYTHONUNBUFFERED=1 \
    uv run python -m sim.forge_scenario --duration "$SIM_DURATION" --zmq --pace \
    --scenario "$SCENARIO" >/tmp/opencode/beacon_wrap_scenario.log 2>&1 &
SIM_PID=$!
sleep 3
# a standing fleet holds 5580+: the host sim dies silently while the mission
# (a CONNECT client) boots happy and measures its own starvation
if ! kill -0 "$SIM_PID" 2>/dev/null; then
    echo "world died at startup: a host link port is already taken?" >&2
    echo "  (sweep the yard: scripts/fleet.sh down) last lines:" >&2
    tail -3 /tmp/opencode/beacon_wrap_scenario.log >&2 || true
    exit 1
fi

echo "--- COSMOS-side acceptance (JSON-RPC only) ---"
START_TS="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
rc=0
uv run python -m sim.beacon_tables || rc=$?

echo "--- COSMOS log hygiene (since client start) ---"
# The real signature is the interface microservice WARN "UNKNOWN packet
# length: N starting with: <msgid>" - it fires for undef'd ids AND for
# length drift on known ids. Do NOT grep the bare word "unknown": OpenC3's
# always-present catch-all loggers are named DEFAULT__*_LOG__UNKNOWN and the
# periodic operator stdout enumerates them - a guaranteed false positive.
unknown="$(docker logs --since "$START_TS" cosmos-openc3-operator-1 2>&1 \
    | grep -c "UNKNOWN packet" || true)"
echo "UNKNOWN packet lines: $unknown"
[ "$unknown" -gt 0 ] && rc=1

if [ "$rc" -eq 0 ]; then
    echo "COSMOS WRAP RUN OK"
else
    echo "COSMOS WRAP RUN FAILED"
fi
exit "$rc"
