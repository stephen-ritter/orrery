#!/usr/bin/env bash
# The scenario ships the organs it grows - flown on the build plane itself.
#
#   cut     chief.toml (no eye): the door's fragment names no cam, the mission
#         installs a plane with NO cam object file, and the image's own
#         build-time table carries no cam record either. The craft boots that
#         plane on the build default and the beacon streams everything except
#         CAM_HK - a plane missing an organ boots as cleanly as one that never
#         had to carry it.
#   whole   camera_ops.toml (eye declared): the fragment names cam, the object
#         file is back, the build table wakes it, ES initializes it, and the
#         same order on the same port is answered with CAM_HK.
#   restored  the fragment swept, the union reinstalled: a quiet yard means
#         every organ again, and the stamp agrees with the tree.
#
# The witnesses are the operator's: raw ports and container records - never a
# scenario file (docs/what_the_scenario_rules.md). Craft boot WITHOUT
# ORRERY_VEHICLE on purpose: the build default IS the artifact under test -
# the table the build wrote from the list the scenario earned.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
bash "$REPO/scripts/firmware.sh" check
RUNTIME="${RUNTIME:-docker}"
BOOT_TIMEOUT="${BOOT_TIMEOUT:-180}"
SIM_DURATION="${SIM_DURATION:-60}"
WORK="${WORK:-/tmp/opencode/scenario_image}"
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
RENDERER="orrery-cam-viz-scenario"

restore() {
    echo "=== restoring the union plane (fragment swept, mission reinstalled) ==="
    uv run python -m codegen.emit_applist --sweep &&
        bash "$REPO/scripts/mission_build.sh" install >/tmp/opencode/scenario_image/install.log 2>&1
}
cleanup() {
    [ -n "$SIM_PID" ] && kill "$SIM_PID" >/dev/null 2>&1 || true
    "$RUNTIME" rm -f "$RENDERER" >/dev/null 2>&1 || true
    [ -n "$CONTAINER" ] && "$RUNTIME" stop "$CONTAINER" >/dev/null 2>&1 || true
}
trap 'cleanup; restore' EXIT

stale="$("$RUNTIME" ps -q --filter ancestor=orrery-cfs:dev || true)"
[ -n "$stale" ] && "$RUNTIME" stop $stale >/dev/null || true

# --- the operator's witness (same sentence check_startup_config.sh speaks) ----
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
    want = sys.argv[1]  # "answer" (whole) or "silence" (cut)
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

run_craft_build_default() {
    # no ORRERY_VEHICLE: the craft flies the table its own build wrote - the
    # exact artifact the scenario's build list produced.
    CONTAINER="$("$RUNTIME" run -d --network host --user "$(id -u):$(id -g)" -e HOME=/tmp \
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

# --- leg one: the scenario grows no eye, the plane carries none ---------------
echo "=== [cut] chief.toml: no eye in the manifest, no object file in the plane ==="
CUT_SCENARIO="$REPO/sim/scenarios/chief.toml"
uv run python -m codegen.emit_applist --scenario "$CUT_SCENARIO"
bash "$REPO/scripts/mission_build.sh" install >/dev/null
if [ -e "$CF_DIR/cam.so" ]; then
    echo "[cut] the cut plane still carries cam.so - the list did not reach the install" >&2
    exit 1
fi
if grep -q ", cam," "$CF_DIR/cfe_es_startup.scr"; then
    echo "[cut] the build's own table wakes an organ the build never compiled" >&2
    exit 1
fi
run_craft_build_default
craft_wait "startup table: build default"
craft_wait "gs_tcp downlink :$BEACON uplink :$UPLINK"
start_world cut "$CUT_SCENARIO"
echo "--- [cut] the beacon streams on a plane that never carried an eye ---"
if ! PYTHONPATH="$REPO" uv run python "$WORK/witness.py" silence "$BEACON" "$UPLINK" >"$WORK/witness_cut.json"; then
    cat "$WORK/witness_cut.json" >&2 || true
    echo "[cut] the beacon answered CAM_HK from a plane with no cam, or heard nothing" >&2
    exit 1
fi
cat "$WORK/witness_cut.json"
stop_world

# --- leg two: the scenario grows the eye, the plane carries it -----------------
echo "=== [whole] camera_ops.toml: the eye declared, the object file and all ---"
WHOLE_SCENARIO="$REPO/sim/scenarios/camera_ops.toml"
uv run python -m codegen.emit_applist --scenario "$WHOLE_SCENARIO"
bash "$REPO/scripts/mission_build.sh" install >/dev/null
if [ ! -e "$CF_DIR/cam.so" ]; then
    echo "[whole] the declared eye's object file never arrived in the plane" >&2
    exit 1
fi
if ! grep -q ", cam," "$CF_DIR/cfe_es_startup.scr"; then
    echo "[whole] the build's own table forgot the organ the scenario grew" >&2
    exit 1
fi
run_craft_build_default
craft_wait "startup table: build default"
craft_wait "CAM App Initialized"
start_world whole "$WHOLE_SCENARIO" stage
echo "--- [whole] the same order; CAM_HK rides the beacon ---"
if ! PYTHONPATH="$REPO" uv run python "$WORK/witness.py" answer "$BEACON" "$UPLINK" >"$WORK/witness_whole.json"; then
    cat "$WORK/witness_whole.json" >&2 || true
    echo "[whole] the declared authority refused to answer" >&2
    exit 1
fi
cat "$WORK/witness_whole.json"
stop_world

# --- the quiet yard means the union again -------------------------------------
restore
trap cleanup EXIT
if [ ! -e "$CF_DIR/cam.so" ]; then
    echo "[restore] the union did not come back - the yard stays a scenario's" >&2
    exit 1
fi
bash "$REPO/scripts/firmware.sh" check

echo "SCENARIO IMAGE CHECK OK: the manifest decides which organs exist at all -" \
    "no file and no table entry undeclared, both and CAM_HK declared, the union restored"
