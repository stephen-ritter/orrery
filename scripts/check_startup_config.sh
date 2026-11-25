#!/usr/bin/env bash
# The wake list is a scenario fact, flown: two worlds, one command sentence.
#
#   never   chief.toml - no eye declared. The yard door lands a table with no
#           cam; the container records that it booted that table; ES never
#           initializes the app; the ORDER still goes out on the operator's
#           own port and the beacon stays silent to it - while every other
#           frame streams (selective silence, not a dead port); and the
#           store's manifest file never appears.
#   awake   camera_ops.toml - the eye declared. The same door lands a table
#           that includes cam, ES initializes it, and the same order is
#           answered: CAM_HK on the wire, counters and all.
#
# The absence verdict reads no scenario file and no table: it listens on the
# ground port, the witness an operator actually has (docs/what_the_scenario_rules
# .md, the law that at least one witness is never derived from the
# declaration). Renderer staged only for the awake flight; digests are the
# standing ones, untouched by what the table does.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
bash "$REPO/scripts/firmware.sh" check
RUNTIME="${RUNTIME:-docker}"
BOOT_TIMEOUT="${BOOT_TIMEOUT:-180}"
SIM_DURATION="${SIM_DURATION:-60}"
WORK="${WORK:-/tmp/opencode/startup_config}"
CF_DIR="$REPO/cfs_mission/mission/build/exe/cpu1/cf"
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

# --- the operator's witness: an order out, the wire listened to ---------------
cat >"$WORK/witness.py" <<'PYEOF'
"""Order the camera's housekeeping; report what the beacon answers."""

import json
import sys
import time

from sim.ground_probe import GroundClient, build_cmd

HK_MID = 0x08D2
SEND_HK = 0x18D3


def mid_of(packet: bytes) -> int:
    return ((packet[0] & 0x3F) << 8) | packet[1]


def main() -> int:
    want = sys.argv[1]  # "answer" (awake) or "silence" (never)
    client = GroundClient(tlm_port=BEACON, cmd_port=UPLINK)
    deadline = time.time() + 15
    client.send_packet(build_cmd(SEND_HK, 0))
    hk = None
    others = 0
    while time.time() < deadline:
        packet = client.recv_packet(deadline)
        if packet and len(packet) > 7:
            if mid_of(packet) == HK_MID:
                hk = len(packet)
            else:
                others += 1
        if hk and others >= 5:
            break
    report = {"want": want, "cam_hk_bytes": hk, "other_frames": others}
    print(json.dumps(report))
    if want == "answer":
        return 0 if hk else 1
    return 0 if (hk is None and others >= 5) else 1


BEACON, UPLINK = int(sys.argv[2]), int(sys.argv[3])
raise SystemExit(main())
PYEOF

craft_wait() {  # craft_wait <pattern> [count]: plain counter, never local SECONDS
    local pattern="$1" want="${2:-1}" waited=0 count
    while [ "$waited" -lt "$BOOT_TIMEOUT" ]; do
        count="$("$RUNTIME" logs "$CONTAINER" 2>&1 | grep -c "$pattern" || true)"
        [ "${count:-0}" -ge "$want" ] && return 0
        sleep 1
        waited=$((waited + 1))
    done
    echo "craft never logged '$pattern' (want $want) within ${BOOT_TIMEOUT}s"
    return 1
}

run_craft() {
    CONTAINER="$("$RUNTIME" run -d --network host --user "$(id -u):$(id -g)" -e HOME=/tmp \
        -e ORRERY_VEHICLE=chief \
        -e ORRERY_TLM_PORT="$BEACON" -e ORRERY_CMD_PORT="$UPLINK" \
        -e ORRERY_ZMQ_SENSOR_ENDPOINT="tcp://127.0.0.1:$SENSOR" \
        -e ORRERY_ZMQ_CMD_ENDPOINT="tcp://127.0.0.1:$LINK_CMD" \
        -v "$REPO:/work" orrery-cfs:dev \
        bash -c "cd /work/cfs_mission/mission/build/exe/cpu1 && ./container-start")"
}

start_world() {  # start_world <mode> <scenario> [stage-renderer]
    local mode="$1" scenario="$2" viz="${3:-}"
    if [ -n "$viz" ]; then
        RUNTIME="$RUNTIME" bash "$REPO/scripts/renderer.sh" stage "$scenario" "$RENDERER"
    fi
    PYTHONUNBUFFERED=1 uv run python -m sim.forge_scenario --scenario "$scenario" \
        --duration "$SIM_DURATION" --zmq --pace >"$WORK/sim_$mode.log" 2>&1 &
    SIM_PID=$!
    sleep 3
    kill -0 "$SIM_PID" 2>/dev/null || {
        echo "[$mode] world died at startup: a host link port is already taken?" >&2
        tail -3 "$WORK/sim_$mode.log" >&2 || true
        return 1
    }
}

stop_world() {
    wait "$SIM_PID" 2>/dev/null || true
    SIM_PID=""
    [ -z "$CONTAINER" ] || "$RUNTIME" stop "$CONTAINER" >/dev/null 2>&1 || true
    [ -z "$CONTAINER" ] || "$RUNTIME" rm -f "$CONTAINER" >/dev/null 2>&1 || true
    CONTAINER=""
    "$RUNTIME" rm -f "$RENDERER" >/dev/null 2>&1 || true
}

# --- leg one: declared nothing, wakes nothing ---------------------------------
echo "=== [never] chief.toml: no eye declared, no authority booted ==="
NEVER_SCENARIO="$REPO/sim/scenarios/chief.toml"
uv run python -m codegen.emit_startup --scenario "$NEVER_SCENARIO" --write "$CF_DIR"
if grep -q ", cam," "$CF_DIR/cfe_es_startup.chief.scr"; then
    echo "[never] the landed table woke cam on a vehicle that declared no eye" >&2
    exit 1
fi
rm -f "$CF_DIR/manifest.txt"
run_craft
craft_wait "startup table: scenario door (chief)"
craft_wait "gs_tcp downlink :$BEACON uplink :$UPLINK"
logs="$("$RUNTIME" logs "$CONTAINER" 2>&1 || true)"
if grep -q "startup table: build default" <<<"$logs"; then
    echo "[never] the record says build default - the door's table did not boot" >&2
    exit 1
fi
if grep -q "CAM App Initialized" <<<"$logs"; then
    echo "[never] ES initialized an app the wake list never named" >&2
    exit 1
fi
start_world never "$NEVER_SCENARIO"
echo "--- [never] the order goes out; the wire answers with everything except ---"
if ! PYTHONPATH="$REPO" uv run python "$WORK/witness.py" silence "$BEACON" "$UPLINK" >"$WORK/witness_never.json"; then
    cat "$WORK/witness_never.json" >&2 || true
    echo "[never] the beacon answered a never-woke app, or heard nothing at all" >&2
    exit 1
fi
cat "$WORK/witness_never.json"
if [ -e "$CF_DIR/manifest.txt" ]; then
    echo "[never] a manifest appeared from an app that was never awake" >&2
    exit 1
fi
stop_world

# --- leg two: declared the eye, woken the authority ---------------------------
echo "=== [awake] camera_ops.toml: the eye declared, the authority answers ==="
AWAKE_SCENARIO="$REPO/sim/scenarios/camera_ops.toml"
uv run python -m codegen.emit_startup --scenario "$AWAKE_SCENARIO" --write "$CF_DIR"
if ! grep -q ", cam," "$CF_DIR/cfe_es_startup.chief.scr"; then
    echo "[awake] the landed table omits cam on a vehicle that declared the eye" >&2
    exit 1
fi
run_craft
craft_wait "startup table: scenario door (chief)"
craft_wait "CAM App Initialized"
start_world awake "$AWAKE_SCENARIO" stage
echo "--- [awake] the same order; this time CAM_HK rides the beacon ---"
if ! PYTHONPATH="$REPO" uv run python "$WORK/witness.py" answer "$BEACON" "$UPLINK" >"$WORK/witness_awake.json"; then
    cat "$WORK/witness_awake.json" >&2 || true
    echo "[awake] the declared authority refused to answer" >&2
    exit 1
fi
cat "$WORK/witness_awake.json"
stop_world

echo "STARTUP CHECK OK: the scenario's word is the core's wake list - silence undeclared, CAM_HK declared, both from the operator's own port"
