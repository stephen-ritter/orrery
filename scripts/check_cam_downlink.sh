#!/usr/bin/env bash
# Ground-requested files cross byte-identical: the downlink acceptance.
#
#   request   the downlink plant flies; the ground asks the store what it has
#             (CAM_MANIFEST), downlinks the manifest like any file, picks a
#             frame BY MANIFEST NAME, and receives it with the craft's own
#             EOT verdict (state, txn_stat, fsize) and CF's transmit-time
#             checksum re-verified over the landed bytes. The manifest is
#             then rebuilt and must list a file that arrived mid-flight.
#   generic   a random binary that is no picture at all crosses byte-exact:
#             the transport serves FILES, not an image format.
#   heals     the engine's link is dropped and redialed mid-flight; the next
#             request lands with no craft restart anywhere in the script.
#   silence   the same plant, no request ever sent: the landing folder stays
#             empty. Eligibility is ground-side, and absence is its proof.
#   control   camera_ops (no record, no downlink): identical trajectory
#             digest - pixels and requests never touch physics.
#
# Sequencing law, encoded: wait for "engine connected" before any request;
# one transaction in flight; every landing carries this run's prefix and is
# swept from the bucket at the end.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
bash "$REPO/scripts/firmware.sh" check
RUNTIME="${RUNTIME:-docker}"
BOOT_TIMEOUT="${BOOT_TIMEOUT:-120}"
SIM_DURATION="${SIM_DURATION:-75}"
BUCKET="${BUCKET:-cosmos-openc3-buckets-1}"
BUCKET_DIR="/data/config/cfdp"
DOWNLINK_SCENARIO="$REPO/sim/scenarios/camera_downlink.toml"
CONTROL_SCENARIO="$REPO/sim/scenarios/camera_ops.toml"
RUNID="$(date +%s)$$"
PREFIX="camdown_${RUNID}_"
WORK="${WORK:-/tmp/opencode/cam_downlink}"
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
    "$RUNTIME" exec "$BUCKET" sh -c "rm -f $BUCKET_DIR/${PREFIX}*" >/dev/null 2>&1 || true
}
trap cleanup EXIT

"$RUNTIME" exec "$BUCKET" sh -c "ls $BUCKET_DIR" >/dev/null 2>&1 || {
    echo "bucket $BUCKET:$BUCKET_DIR unreachable: bring the station up first (scripts/cosmos_stack.sh)"
    exit 1
}

stale="$("$RUNTIME" ps -q --filter ancestor=orrery-cfs:dev || true)"
if [ -n "$stale" ]; then
    echo "=== releasing the yard: stopping stale craft containers ==="
    "$RUNTIME" stop -t 5 $stale >/dev/null
    for _ in $(seq 1 30); do
        [ -z "$("$RUNTIME" ps -q --filter ancestor=orrery-cfs:dev || true)" ] && break
        sleep 2
    done
    [ -z "$("$RUNTIME" ps -q --filter ancestor=orrery-cfs:dev || true)" ] || {
        echo "a craft container refuses to leave the yard (ports still claimed)"
        exit 1
    }
fi

# --- the ground's voice: commands out, end-of-transaction verdicts back ------
cat >"$WORK/dl.py" <<'PYEOF'
"""One action per invocation: a request or receipt on the raw ground port."""

import json
import struct
import sys
import time

from sim.ground_probe import GroundClient, build_cmd

EOT_MID, HK_MID = 0x08B3, 0x08D2

# Patience is the engine's law, not a hope. CF notices a peer that died
# mid-transaction only when the channel's inactivity timer expires
# (cFS/apps/cf/fsw/tables/cf_def_config.c: 30 s), and a request that queues
# behind that stalled transaction cannot answer before it recycles. So the
# wait is that limit plus margin: the heals leg then tests "the craft heals
# on its own, inside its own limits" instead of "the craft got lucky inside
# 20 s" - which is how a healthy engine once read as a broken one here.
EOT_WAIT_S = 45


def mid_of(packet: bytes) -> int:
    return ((packet[0] & 0x3F) << 8) | packet[1]


def wait_mid(client, mid: int, want: bytes | None, deadline: float) -> bytes | None:
    while time.time() < deadline:
        packet = client.recv_packet(deadline)
        if packet and len(packet) > 16 and mid_of(packet) == mid:
            if want is None or want in packet:  # fnames ride the EOT body
                return packet
    return None


def main() -> int:
    action, tlm, cmd = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
    client = GroundClient(tlm_port=tlm, cmd_port=cmd)
    try:
        if action == "manifest":
            client.send_packet(build_cmd(0x18D2, 1))  # CAM_CMD FC 1: inventory
            print(json.dumps({"sent": "CAM_MANIFEST"}))
            return 0
        if action == "hk":
            client.send_packet(build_cmd(0x18D3, 0))
            deadline = time.time() + 10
            packet = wait_mid(client, HK_MID, None, deadline)
            if not packet:
                print(json.dumps({"error": "no CAM_HK"}))
                return 1
            recorded, stored, transferred = struct.unpack("<3I", packet[-12:])
            print(json.dumps({"recorded": recorded, "stored": stored, "transferred": transferred}))
            return 0
        if action == "request":
            src, dst = sys.argv[4].encode(), sys.argv[5].encode()
            payload = struct.pack("<IB3x", 23, 1) + src.ljust(64, b"\0") + dst.ljust(64, b"\0")
            client.send_packet(build_cmd(0x18D6, 0, payload))
            deadline = time.time() + EOT_WAIT_S
            packet = wait_mid(client, EOT_MID, dst, deadline)
            if not packet:
                print(json.dumps({"error": f"no EOT for {dst.decode()}"}))
                return 1
            # EOT payload @16: seq, channel, dir, state, txn_stat, src, peer,
            # fsize, crc - then the two 64 B names. Measured on the wire.
            words = struct.unpack("<9I", packet[16:52])
            print(
                json.dumps(
                    {
                        "dst": dst.decode(),
                        "state": words[3],
                        "txn_stat": words[4],
                        "fsize": words[7],
                        "crc": words[8],
                    }
                )
            )
            return 0
        raise SystemExit(f"unknown action {action}")
    finally:
        client.close()


main()
PYEOF

dl() { PYTHONPATH="$REPO" uv run python "$WORK/dl.py" "$@"; }
request() { dl request "$BEACON" "$UPLINK" "$1" "$2"; }

bucket_sha() { "$RUNTIME" exec "$BUCKET" sha256sum "$BUCKET_DIR/$1" | awk '{print $1}'; }
bucket_cat() { "$RUNTIME" exec "$BUCKET" cat "$BUCKET_DIR/$1"; }

dl_expect() { # <src> <dst> -> the EOT verdict on stdout, gated on txn_stat
    local json
    json="$(request "$1" "$2")" || true
    grep -q '"error"' <<<"$json" && { echo "downlink failed: $json" >&2; return 1; }
    grep -q '"txn_stat": 0' <<<"$json" || { echo "EOT txn_stat not NO_ERROR: $json" >&2; return 1; }
    echo "$json"
}

craft_wait() { # <pattern> <timeout-s> [min-count]
    # A plain counter, never `local SECONDS`: a localized SECONDS stops
    # counting (bash 5.2 proven), and a wait loop that cannot age is a
    # timeout that never fires.
    local pattern="$1" timeout="$2" want="${3:-1}" waited=0
    while [ "$waited" -lt "$timeout" ]; do
        local count
        count="$("$RUNTIME" logs "$CONTAINER" 2>&1 | grep -c "$pattern" || true)"
        [ "${count:-0}" -ge "$want" ] && return 0
        sleep 1
        waited=$((waited + 1))
    done
    echo "craft never logged '$pattern' (want $want) within ${timeout}s"
    return 1
}

# --- world plumbing (probe-free: the ground work IS the witness) -------------
start_world() {
    local mode="$1" scenario="$2"
    rm -rf "$HOST_IMG"
    CONTAINER="$("$RUNTIME" run -d --network host --user "$(id -u):$(id -g)" -e HOME=/tmp \
        -e ORRERY_TLM_PORT="$BEACON" -e ORRERY_CMD_PORT="$UPLINK" \
        -e ORRERY_ZMQ_SENSOR_ENDPOINT="tcp://127.0.0.1:$SENSOR" \
        -e ORRERY_ZMQ_CMD_ENDPOINT="tcp://127.0.0.1:$LINK_CMD" \
        -v "$REPO:/work" orrery-cfs:dev \
        bash -c "cd /work/cfs_mission/mission/build/exe/cpu1 && ./container-start")"
    echo "=== [$mode] waiting for gs_tcp + fcs + cam (max ${BOOT_TIMEOUT}s) ==="
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
    # here-string, never `docker logs | grep -q`: under pipefail, a grep that
    # matches early exits and the docker CLI dies of SIGPIPE (141), so the
    # pipeline fails even though the marker matched - a race, not a verdict
    grep -q "CAM App Initialized" <<<"$logs" || {
        echo "[$mode] FAILED (boot markers not seen within ${BOOT_TIMEOUT}s)"
        "$RUNTIME" logs --tail 10 "$CONTAINER" 2>&1 | sed 's/^/    /' || true
        return 1
    }
    echo "=== [$mode] the CFDP engine must be dialled in before any request ==="
    # A previous run's heals leg (or a station restart) can leave CFDP_INT
    # disconnected: COSMOS does not redial a deliberately cut interface, so
    # the ladder asks once, then holds the craft to the same 90-s promise.
    if ! craft_wait "cfdp engine connected" 20; then
        echo "--- CFDP_INT idle: asking the station to dial ---"
        uv run python - <<'PY'
from sim.cosmos_api import login, rpc

tok = login()
status = {row[0]: row[1] for row in rpc(tok, "get_all_interface_info")}
if status.get("CFDP_INT") != "CONNECTED":
    rpc(tok, "connect_interface", "CFDP_INT")
print("dial requested")
PY
        craft_wait "cfdp engine connected" 70
    fi

    echo "=== [$mode] staging the patient renderer ==="
    RUNTIME="$RUNTIME" bash "$REPO/scripts/renderer.sh" stage "$scenario" "$RENDERER"

    echo "=== [$mode] camera world (${scenario##*/}, ${SIM_DURATION} sim-s) ==="
    PYTHONUNBUFFERED=1 uv run python -m sim.forge_scenario --scenario "$scenario" \
        --duration "$SIM_DURATION" --zmq --pace >"$WORK/sim_$mode.log" 2>&1 &
    SIM_PID=$!
    sleep 3
    kill -0 "$SIM_PID" 2>/dev/null || {
        echo "[$mode] world died at startup: a host link port is already taken?" >&2
        tail -3 "$WORK/sim_$mode.log" >&2 || true; return 1
    }
}

stop_world() {
    local mode="$1"
    wait "$SIM_PID"; SIM_PID=""
    WORLD_DIGEST="$(grep -oE 'trajectory sha256 [0-9a-f]{64}' "$WORK/sim_$mode.log" | tail -1 | awk '{print $3}')"
    [ -n "$WORLD_DIGEST" ] || { echo "[$mode] FAILED: no digest printed"; return 1; }
    "$RUNTIME" stop "$CONTAINER" >/dev/null 2>&1 || true
    "$RUNTIME" logs "$CONTAINER" >"$WORK/console_$mode.log" 2>&1 || true
    "$RUNTIME" rm -f "$CONTAINER" >/dev/null 2>&1 || true
    CONTAINER=""
    "$RUNTIME" rm -f "$RENDERER" >/dev/null 2>&1 || true
    echo "=== [$mode] digest ${WORLD_DIGEST:0:16}... ==="
}

wait_frames() { # <count> <timeout-s>: the store must have pictures before it can promise them
    local want="$1" timeout="$2" waited=0
    while [ "$waited" -lt "$timeout" ]; do
        local n
        n="$(find "$HOST_IMG" -name '*.png' 2>/dev/null | wc -l | tr -d ' ')"
        [ "${n:-0}" -ge "$want" ] && return 0
        sleep 2
        waited=$((waited + 2))
    done
    echo "store never reached $want PNGs within ${timeout}s"
    return 1
}

# ===========================================================================
# leg request: ask what exists, ask for one of them, receive the truth twice
# ===========================================================================
echo "=== REQUEST LEG: the store answers, the ground picks, bytes arrive ==="
start_world request "$DOWNLINK_SCENARIO"
wait_frames 2 120

dl manifest "$BEACON" "$UPLINK" >/dev/null
sleep 2  # the inventory is written on CAM's task tick, not in the command call
MANIFEST="${PREFIX}manifest.txt"
dl_expect /cf/manifest.txt "$MANIFEST" >/dev/null
bucket_cat "$MANIFEST" | tee "$WORK/manifest.txt"
PICK="$(sed -n '2p' "$WORK/manifest.txt" | cut -d, -f1)"  # full path, earliest-created frame
[ -n "$PICK" ] || { echo "manifest lists no frame: $(head -3 "$WORK/manifest.txt")"; exit 1; }

echo "=== request $PICK by manifest path ==="
FRAME_LAND="${PREFIX}frame.png"
# Queue law: this delivery will delete the craft's copy seconds after its
# EOT. Every witness of the SOURCE is therefore taken before the request,
# and the byte law is a chain: store sha (pre-request) == landed bytes ==
# ground sha == CF's own checksum recomputed over what crossed.
PICK_SHA="$(sha256sum "$HOST_IMG/$(basename "$PICK")" | awk '{print $1}')"
dl_expect "$PICK" "$FRAME_LAND" >"$WORK/eot_frame.json"
FRAME_SHA="$(bucket_sha "$FRAME_LAND")"
bucket_cat "$FRAME_LAND" >"$WORK/frame.landed"

uv run python - "$WORK/eot_frame.json" "$WORK/manifest.txt" \
    "$(basename "$PICK")" "$FRAME_SHA" "$PICK_SHA" "$WORK/frame.landed" <<'PY'
import csv
import hashlib
import json
import re
import struct
import sys
from pathlib import Path

eot = json.loads(Path(sys.argv[1]).read_text())
reader = csv.DictReader(Path(sys.argv[2]).read_text().splitlines())
assert reader.fieldnames == ["path", "size_bytes", "created_utc"], (
    f"manifest header is {reader.fieldnames}; the CSV contract says path,size_bytes,created_utc")
rows = list(reader)
store_name = sys.argv[3]
landed_sha = sys.argv[4]
store_sha = sys.argv[5]
raw = Path(sys.argv[6]).read_bytes()

# The column laws, witnessed on every flight: the path is the FULL path (a
# reader must not rebuild the store layout from a bare name), every line
# carries an ISO-8601 UTC creation stamp, the lines run in creation order
# (a directory walk answers in filesystem hash order - the old listing read
# as random; capture order is the order a reader can act on), and no line
# lists an empty file.
assert rows, "manifest lists no files"
assert all(r["path"].startswith("/cf/img/") for r in rows), "the path column is not the full path"
created = [r["created_utc"] for r in rows]
assert created == sorted(created), "manifest lines are not in creation order"
assert all(re.fullmatch(r"\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}Z", c) for c in created), (
    f"created_utc is not ISO-8601 Z: {created[:3]}")
assert all(int(r["size_bytes"]) > 0 for r in rows), "an inventory line lists a zero-byte file"

row = next(r for r in rows if Path(r["path"]).name == store_name)
size_listed = int(row["size_bytes"])
assert len(raw) == size_listed, f"manifest size {size_listed} disagrees with what crossed ({len(raw)})"
assert len(raw) == eot["fsize"], "EOT fsize disagrees with what crossed"


def cfdp_checksum(data: bytes) -> int:  # CF's transmit-time witness, recomputed
    pad = (-len(data)) % 4
    total = 0
    for i in range(0, len(data) + pad, 4):
        word = (data + b"\0" * pad)[i : i + 4]
        total = (total + struct.unpack(">I", word)[0]) & 0xFFFFFFFF
    return total


assert eot["crc"] == cfdp_checksum(raw), "CF's own checksum disagrees with the landed bytes"
assert hashlib.sha256(raw).hexdigest() == landed_sha, "landed bytes differ from the ground's own hash"
assert landed_sha == store_sha, "landed file is not byte-identical to the store as it stood before the ask"
print(f"byte law OK: {store_name} {len(raw)} B, sha256 {landed_sha[:16]}... store -> airgap -> ground identical")
PY

echo "=== CAM_HK: the four-number promise now includes a transfer ==="
HK_JSON="$(dl hk "$BEACON" "$UPLINK")" || true
grep -q '"error"' <<<"$HK_JSON" && { echo "CAM_HK unanswered: $HK_JSON"; exit 1; }
echo "$HK_JSON" >&2
FILES="$(find "$HOST_IMG" -name '*.png' | wc -l | tr -d ' ')"
uv run python - "$HK_JSON" "$FILES" <<'PY'
import json
import sys

hk = json.loads(sys.argv[1])
files = int(sys.argv[2])
assert hk["recorded"] > 0 and hk["stored"] > 0, "no recording promise on a recording world"
# Queue law: a frame delivered under class 2 is recycled the moment its
# receipt lands, so what STANDS under /cf/img can be fewer than what was
# recorded since boot. It can never be more: bytes arrive before they leave.
assert hk["stored"] >= files >= 1, f"standing files {files} outside (1, stored={hk['stored']})"
assert hk["transferred"] >= 1, "a file crossed and FramesTransferred stayed silent"
print(f"promise law OK: recorded={hk['recorded']} stored={hk['stored']} transferred={hk['transferred']}")
PY

# --- generic: the transport serves files, not a picture format ---------------
echo "=== GENERIC LEG: a random binary with no image meaning crosses byte-exact ==="
# The name carries RUNID's last digits only: /cf resolves to a 43 B local
# path and OS_MAX_PATH_LEN is 64, so store names live at frame scale
# (14 B). A longer name is not stored - and now says so on the console.
CRAFT_NAME="cargo_${RUNID: -4}.bin"
head -c 7777 /dev/urandom >"$HOST_IMG/$CRAFT_NAME"
CARGO_SHA="$(sha256sum "$HOST_IMG/$CRAFT_NAME" | awk '{print $1}')"
sleep 2  # let the store settle before the manifest walks it
dl manifest "$BEACON" "$UPLINK" >/dev/null
sleep 2  # CAM regenerates on its 1 Hz task: ask WHAT exists only once it answered
MANIFEST2="${PREFIX}manifest2.txt"
dl_expect /cf/manifest.txt "$MANIFEST2" >/dev/null
MANIFEST2_TEXT="$(bucket_cat "$MANIFEST2")"
grep -q "^/cf/img/$CRAFT_NAME,7777," <<<"$MANIFEST2_TEXT" || {
    echo "rebuilt manifest does not list the mid-flight file"; exit 1;
}
CARGO_LAND="${PREFIX}cargo.bin"
dl_expect "/cf/img/$CRAFT_NAME" "$CARGO_LAND" >/dev/null
[ "$(bucket_sha "$CARGO_LAND")" = "$CARGO_SHA" ] || { echo "cargo bytes changed"; exit 1; }
echo "generic law OK: 7777 B of noise listed, requested by name, byte-identical"

# --- heals: drop the engine's link, dial again, keep shipping ---------------
echo "=== HEALS LEG: the engine leaves and returns; the craft never reboots ==="
# Baselines counted with the SAME patterns the waits watch: a baseline for
# "peer closed" (which every beacon-client parting also logs) against a wait
# for "cfdp link closed (peer closed)" is arithmetic that can never be right.
TOTAL_BEFORE="$("$RUNTIME" logs "$CONTAINER" 2>&1 | grep -c "engine connected" || true)"
CLOSED_BEFORE="$("$RUNTIME" logs "$CONTAINER" 2>&1 | grep -c "cfdp link closed (peer closed)" || true)"
STARTED_BEFORE="$("$RUNTIME" inspect -f '{{.State.StartedAt}}' "$CONTAINER")"
uv run python - <<'PY'
from sim.cosmos_api import login, rpc

rpc(login(), "disconnect_interface", "CFDP_INT")
print("disconnect issued")
PY
craft_wait "cfdp link closed (peer closed)" 30 $((CLOSED_BEFORE + 1))
uv run python - <<'PY'
import time
from sim.cosmos_api import login, rpc

tok = login()
rpc(tok, "connect_interface", "CFDP_INT")
deadline = time.time() + 30
while time.time() < deadline:
    if any(r[0] == "CFDP_INT" and r[1] == "CONNECTED" for r in rpc(tok, "get_all_interface_info")):
        print("CFDP_INT CONNECTED")
        break
    time.sleep(2)
else:
    raise SystemExit("CFDP_INT never re-connected")
PY
craft_wait "engine connected" 60 $((TOTAL_BEFORE + 1))
HEALED_LAND="${PREFIX}healed.txt"
dl_expect /cf/manifest.txt "$HEALED_LAND" >/dev/null
[ "$("$RUNTIME" inspect -f '{{.State.StartedAt}}' "$CONTAINER")" = "$STARTED_BEFORE" ] || {
    echo "the craft restarted: heals is a lie"; exit 1;
}
echo "heals law OK: link re-formed and served a request with no craft restart"

stop_world request || exit 1
REQUEST_DIGEST="$WORLD_DIGEST"
CRAFT_LOG="$WORK/console_request.log"
grep -q "second cfdp dialer refused" "$CRAFT_LOG" && {
    echo "the bridge refused a dialer: EOF release regressed"; exit 1;
} || true
"$RUNTIME" exec "$BUCKET" sh -c "rm -f $BUCKET_DIR/${PREFIX}*" >/dev/null
# the silence leg reads the same bucket: it must start from a swept folder
# or this run's own landings would masquerade as unrequested transfers

# ===========================================================================
# leg silence: the same requestable world, nobody asks, nothing lands
# ===========================================================================
echo "=== SILENCE LEG: requestable is not automatic ==="
start_world silence "$DOWNLINK_SCENARIO"
wait_frames 1 120
SILENCE_LANDINGS="$("$RUNTIME" exec "$BUCKET" sh -c "ls $BUCKET_DIR | grep -c '^${PREFIX}' || true")"
[ "${SILENCE_LANDINGS:-0}" -eq 0 ] || {
    echo "the bucket holds $SILENCE_LANDINGS files although nobody requested one"; exit 1;
}
sleep 20  # a transfer nobody asked for would have landed by now
SILENCE_LANDINGS="$("$RUNTIME" exec "$BUCKET" sh -c "ls $BUCKET_DIR | grep -c '^${PREFIX}' || true")"
[ "${SILENCE_LANDINGS:-0}" -eq 0 ] || { echo "a late unrequested file landed"; exit 1; }
stop_world silence || exit 1
SILENCE_DIGEST="$WORLD_DIGEST"

# ===========================================================================
# leg control: no record, no downlink, identical physics
# ===========================================================================
echo "=== CONTROL LEG: the payload never touches the dynamics ==="
start_world control "$CONTROL_SCENARIO"
sleep 25
LEFT="$(find "$HOST_IMG" -name '*.png' 2>/dev/null | wc -l | tr -d ' ')"
[ "${LEFT:-0}" -eq 0 ] || { echo "control world wrote $LEFT files: the flag is a lie"; exit 1; }
stop_world control || exit 1
CONTROL_DIGEST="$WORLD_DIGEST"

for pair in "request $CONTROL_DIGEST" "silence $CONTROL_DIGEST"; do
    set -- $pair
    other="$REQUEST_DIGEST"; [ "$1" = silence ] && other="$SILENCE_DIGEST"
    [ "$other" = "$2" ] || {
        echo "CAM DOWNLINK FAILED: digests disagree - file traffic touched physics"
        echo "  $1 $other"; echo "  control $2"
        exit 1
    }
done

"$RUNTIME" exec "$BUCKET" sh -c "rm -f $BUCKET_DIR/${PREFIX}*" >/dev/null
echo "CAM DOWNLINK CHECK OK"
echo "  manifest -> request by name -> byte-identical, EOT clean, CF checksum re-verified"
echo "  generic 7777 B cargo crossed too; the rebuilt manifest listed it"
echo "  link healed mid-flight without touching the craft"
echo "  silence: a requestable store shipped nothing unprompted"
echo "  digest pair byte-identical: ${CONTROL_DIGEST:0:16}..."
echo "  receipts: $WORK"