#!/usr/bin/env bash
# CAM record acceptance: image bytes across the airgap, byte for byte, twice flown.
#
#   record   camera_record.toml (record_rate = 2): nobody commands anything;
#            the store outlet must fill unprompted - Basilisk PNGs in a sim
#            temp dir, IMG chunks across ZMQ, reassembled files under
#            /cf/img, ledgers on CamStorage / CamState / CAM_HK agreeing.
#   control  camera_ops.toml (no outlet): zero files, zero promises, and
#            the same trajectory digest as the record leg - the v1 law that
#            pixels never touch physics, measured, not asserted.
#
# The byte argument is the file hash, not a checksum of a re-encode: every
# /cf/img PNG must be sha256-identical to some PNG the sim's own writer
# produced, and it must still decode as the image it claims to be. The
# ledger lines (console exit, probe's last HK) must agree with the file
# count to the unit, and FramesTransferred must stay at its honest zero.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
bash "$REPO/scripts/firmware.sh" check
RUNTIME="${RUNTIME:-docker}"
BOOT_TIMEOUT="${BOOT_TIMEOUT:-120}"
SIM_DURATION="${SIM_DURATION:-60}"
PROBE_DURATION="${PROBE_DURATION:-56}"
RECORD_SCENARIO="$REPO/sim/scenarios/camera_record.toml"
CONTROL_SCENARIO="$REPO/sim/scenarios/camera_ops.toml"
WORK="${WORK:-/tmp/opencode/cam_record}"
mkdir -p "$WORK"
HOST_IMG="$REPO/cfs_mission/mission/build/exe/cpu1/cf/img"

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

# fly_cycle <mode> <scenario> -> CYCLE_RC (probe), CYCLE_DIGEST, CYCLE_LOG,
# CYCLE_STORE_DIR (record leg: where the sim's own PNG writer wrote)
fly_cycle() {
    local mode="$1" scenario="$2"
    CYCLE_RC=0
    CYCLE_STORE_DIR=""
    rm -rf "$HOST_IMG"

    CONTAINER="$("$RUNTIME" run -d --network host --user "$(id -u):$(id -g)" -e HOME=/tmp \
        -e ORRERY_TLM_PORT="$BEACON" -e ORRERY_CMD_PORT="$UPLINK" \
        -e ORRERY_ZMQ_SENSOR_ENDPOINT="tcp://127.0.0.1:$SENSOR" \
        -e ORRERY_ZMQ_CMD_ENDPOINT="tcp://127.0.0.1:$LINK_CMD" \
        -v "$REPO:/work" orrery-cfs:dev \
        bash -c "cd /work/cfs_mission/mission/build/exe/cpu1 && ./container-start")"

    echo "=== [$mode] waiting for gs_tcp + fcs + cam (max ${BOOT_TIMEOUT}s) ==="
    # a plain counter: `local SECONDS` stops counting, and a wait loop that
    # cannot age is a timeout that never fires
    local waited=0
    while [ "$waited" -lt "$BOOT_TIMEOUT" ]; do
        local logs
        logs="$("$RUNTIME" logs "$CONTAINER" 2>&1 || true)"
        if grep -q "gs_tcp downlink :$BEACON uplink :$UPLINK" <<<"$logs" \
            && grep -q "FCS App Initialized" <<<"$logs" \
            && grep -q "CAM App Initialized" <<<"$logs"; then
            break
        fi
        if grep -qE "(GS_TCP|FCS|BSK_LINK|CAM).*(fail|DEAD)|Could not load file|ES Load Module Error" <<<"$logs"; then
            echo "[$mode] FAILED (app init error)"; grep -E "GS_TCP|FCS|BSK_LINK|CAM |Could not load" <<<"$logs" | tail -12
            return 1
        fi
        sleep 1
        waited=$((waited + 1))
    done
    logs="$("$RUNTIME" logs "$CONTAINER" 2>&1 || true)"
    if ! grep -q "CAM App Initialized" <<<"$logs"; then
        echo "[$mode] FAILED (boot markers not seen within ${BOOT_TIMEOUT}s)"
        return 1
    fi

    echo "=== [$mode] staging the patient renderer ==="
    RUNTIME="$RUNTIME" bash "$REPO/scripts/renderer.sh" stage "$scenario" "$RENDERER"

    echo "=== [$mode] camera world (${scenario##*/}, ${SIM_DURATION} sim-s) ==="
    PYTHONUNBUFFERED=1 uv run python -m sim.forge_scenario --scenario "$scenario" \
        --duration "$SIM_DURATION" --zmq --pace >"$WORK/sim_$mode.log" 2>&1 &
    SIM_PID=$!
    sleep 3
    if ! kill -0 "$SIM_PID" 2>/dev/null; then
        echo "[$mode] world died at startup: a host link port is already taken?" >&2
        tail -3 "$WORK/sim_$mode.log" >&2 || true
        return 1
    fi

    echo "=== [$mode] ground probe (${PROBE_DURATION}s) ==="
    CYCLE_RC=0
    uv run python -m sim.cam_probe --mode "$mode" --duration "$PROBE_DURATION" \
        >"$WORK/probe_$mode.log" 2>&1 || CYCLE_RC=$?
    tail -4 "$WORK/probe_$mode.log" | sed 's/^/    /'

    wait "$SIM_PID"; SIM_PID=""
    CYCLE_DIGEST="$(grep -oE 'trajectory sha256 [0-9a-f]{64}' "$WORK/sim_$mode.log" | tail -1 | awk '{print $3}')"
    [ -n "$CYCLE_DIGEST" ] || { echo "[$mode] FAILED: no digest printed"; return 1; }

    CYCLE_LOG="$WORK/console_$mode.log"
    "$RUNTIME" stop "$CONTAINER" >/dev/null 2>&1 || true
    "$RUNTIME" logs "$CONTAINER" >"$CYCLE_LOG" 2>&1 || true
    CONTAINER=""
    "$RUNTIME" rm -f "$RENDERER" >/dev/null 2>&1 || true

    if [ "$mode" = record ]; then
        CYCLE_STORE_DIR="$(grep -oE 'camera[0-9]+ store outlet ON \(every [0-9]+th capture -> [^)]+\)' \
            "$WORK/sim_record.log" | head -1 | sed -E 's/.*-> ([^)]+)\)/\1/')"
    fi
    echo "=== [$mode] digest ${CYCLE_DIGEST:0:16}... probe rc=$CYCLE_RC ==="
}

# --- record leg: the outlet must fill with nobody touching it ----------------
fly_cycle record "$RECORD_SCENARIO" || { echo "CAM RECORD CHECK FAILED (record leg)"; exit 1; }
RECORD_DIGEST="$CYCLE_DIGEST"; RECORD_LOG="$CYCLE_LOG"; RECORD_STORE_DIR="$CYCLE_STORE_DIR"
[ "$CYCLE_RC" -eq 0 ] || { echo "CAM RECORD CHECK FAILED (record probe)"; exit 1; }
[ -n "$RECORD_STORE_DIR" ] || { echo "CAM RECORD FAILED: sim never announced its store dir"; exit 1; }

echo "=== byte-for-byte: /cf/img vs the sim's own PNG writer ==="
[ -d "$HOST_IMG" ] || { echo "no /cf/img on the host: nothing crossed the airgap"; exit 1; }
uv run python - "$RECORD_STORE_DIR" "$HOST_IMG" <<'PY'
import hashlib
import io
import sys
from pathlib import Path

sources = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in Path(sys.argv[1]).glob("*.png")}
host = sorted(Path(sys.argv[2]).glob("*.png"))
assert sources, "the sim's store dir holds no PNGs: the writers never ran"
assert host, "no PNG under /cf/img: the reassembler never completed a file"
wanted = set(sources.values())
foreign = [p.name for p in host if hashlib.sha256(p.read_bytes()).hexdigest() not in wanted]
assert not foreign, f"host files with no byte-identical sim source: {foreign}"
from PIL import Image  # decode law: a reassembly that renders is a reassembly

for p in host:
    Image.open(io.BytesIO(p.read_bytes())).verify()
print(f"byte law OK: {len(host)} host files, every one sha256-identical to a sim source and PNG-valid")
PY

echo "=== ledgers: beacon ledger vs probe's last HK vs the files ==="
RECORD_FILES="$(find "$HOST_IMG" -name '*.png' | wc -l | tr -d ' ')"
uv run python - "$RECORD_LOG" "$WORK/probe_record.log" "$HOST_IMG" <<'PY'
import re
import sys
from pathlib import Path

console = Path(sys.argv[1]).read_text()
probe = Path(sys.argv[2]).read_text()
storage = re.findall(r"storage t=\s*[\d.]+s files=(\d+) bytes=(\d+)", probe)
assert storage, "the CamStorage TLM never reached the beacon: no ledger on the bus"
files, nbytes = (int(x) for x in storage[-1])
host = sorted(Path(sys.argv[3]).glob("*.png"))
assert files == len(host), f"ledger says {files} files, disk says {len(host)}"
assert nbytes == sum(p.stat().st_size for p in host), "ledger bytes disagree with file sizes"
hk = re.findall(r"promises=(\d+)/(\d+)/(\d+)", probe)
assert hk, "the probe never saw a CAM_HK"
recorded, stored, transferred = (int(x) for x in hk[-1])
assert recorded > 0, "CamState never reported a store-outlet send"
assert stored == files, f"CAM_HK stored={stored} but the ledger completed {files}"
assert transferred == 0, "FramesTransferred moved without a transfer: a lie, not a feature"
assert "cam stored" in console, "no EVS testimony for a single completed file"
assert "BSK_LINK 24" not in console, "the store raised an error event: damage was counted"
print(f"ledger law OK: files={files} bytes={nbytes} promises={recorded}/{stored}/{transferred}")
PY

# --- control leg: no outlet, zero promises, identical physics ----------------
fly_cycle control "$CONTROL_SCENARIO" || { echo "CAM RECORD CHECK FAILED (control leg)"; exit 1; }
CONTROL_DIGEST="$CYCLE_DIGEST"; CONTROL_LOG="$CYCLE_LOG"
[ "$CYCLE_RC" -eq 0 ] || { echo "CAM RECORD CHECK FAILED (control probe)"; exit 1; }

echo "=== control leg: absence, verified ==="
LEFT="$(find "$HOST_IMG" -name '*.png' 2>/dev/null | wc -l | tr -d ' ')"
[ "$LEFT" = 0 ] || { echo "control leg wrote $LEFT files: an absent outlet still shipped"; exit 1; }
grep -q "storage t=" "$WORK/probe_control.log" && { echo "a ledger moved with no outlet"; exit 1; } || true
grep -q "cam stored" "$CONTROL_LOG" && { echo "EVS recorded a file the outlet never sent"; exit 1; } || true
# CAM_HK is request-only telemetry and the control leg sends no uplink, so
# the absence is sworn by the DEVICE side: the one CamState view the probe
# prints must say recorded=0 (a store tally with no outlet is impossible).
grep -qE "camstate .*recorded=0 snaps=0$" "$WORK/probe_control.log" || {
    echo "control CamState shows no honest recorded=0"; exit 1;
}

if [ "$RECORD_DIGEST" != "$CONTROL_DIGEST" ]; then
    echo "CAM RECORD FAILED: digests disagree - image traffic touched physics"
    echo "  record  $RECORD_DIGEST"
    echo "  control $CONTROL_DIGEST"
    exit 1
fi

echo "CAM RECORD CHECK OK"
echo "  digest pair byte-identical: ${RECORD_DIGEST:0:16}..."
echo "  files crossed: $RECORD_FILES, every one byte-identical to a sim source, ledger exact"
echo "  receipts: $WORK"
