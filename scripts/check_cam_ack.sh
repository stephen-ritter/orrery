#!/usr/bin/env bash
# Acknowledged-transfer acceptance: the receipt, the refusal to forget
# without one, and the recycling a receipt triggers on the spot.
#
# The recorder obeys the queue law: a frame's life ends at delivery. The
# proof of delivery is the vehicle's own witness - CF was handed a class 2
# transaction, and CF's end-of-transaction report for that exact path
# closed clean - and when that witness speaks, the frame is recycled
# immediately, pressure or not. This bench flies all three edges of that
# law against a six-file recorder:
#
#   receipt   frame A crosses under class 2, CF's EOT says TX/NO_ERROR -
#             and the ledger reacts on the spot: FilesEvicted moves within
#             ~2.5 s of the EOT (the next capture is ~4 s away, so the
#             clock itself says it was the receipt, not a capture), the
#             count stands one file below the cap with nobody asking, and
#             the recycled line names A, the oldest proven frame.
#
#   dead pass frame B is requested while the engine's link is cut. The
#             transaction cannot complete - ground never answers - so B
#             never becomes proven. The acceptance is the silence around
#             B: TransfersAcked does not move, and no recycled line ever
#             names B, now or at the end of the flight.
#
#   recovery  the link re-forms with no craft restart, frame C crosses,
#             acked and evicted move again, same signature, same moment.
#
#   inventory the manifest lists exactly the files standing under /cf/img
#             at fetch time - B still there, A and C not - and nobody
#             asked: the store publishes its own inventory at the boot
#             census and at every mutation, and the command that once
#             opened the ledger is retired (a stray FC 1 is refused).
#
# The six-file recorder (bytes slack: the FILE bound is what binds, so the
# bench is frame-size independent) fills unprompted a few seconds into the
# flight; every capture after that is refused until a receipt frees a
# place. Usage: scripts/check_cam_ack.sh   (SIM_DURATION= default 240)
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUNTIME="${RUNTIME:-docker}"
BOOT_TIMEOUT="${BOOT_TIMEOUT:-120}"
SIM_DURATION="${SIM_DURATION:-240}"
BUCKET="${BUCKET:-cosmos-openc3-buckets-1}"
BUCKET_DIR="/data/config/cfdp"
BASE_SCENARIO="$REPO/sim/scenarios/camera_downlink.toml"
BENCH_NAME="bench_ack.toml"
BENCH="$REPO/sim/scenarios/$BENCH_NAME"
CAP_FILES=6
RUNID="$(date +%s)$$"
PREFIX="camack_${RUNID}_"
WORK="${WORK:-/tmp/opencode/cam_ack}"
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
    rm -f "$BENCH"
    # the tree back to the union default: a dead bench's recorder sizes
    # must not bias the next build (same law as the yard door's sweep)
    uv run python -m codegen.emit_store --sweep >/dev/null 2>&1 || true
    bash "$REPO/scripts/mission_build.sh" install >/dev/null 2>&1 || \
        echo "[cleanup] union reinstall FAILED - the tree still carries the bench's sizes" >&2
}
trap cleanup EXIT

"$RUNTIME" exec "$BUCKET" sh -c "ls $BUCKET_DIR" >/dev/null 2>&1 || {
    echo "bucket $BUCKET:$BUCKET_DIR unreachable: bring the station up first (scripts/cosmos_stack.sh)" >&2
    exit 1
}
stale="$("$RUNTIME" ps -q --filter ancestor=orrery-cfs:dev || true)"
[ -n "$stale" ] && "$RUNTIME" stop $stale >/dev/null || true

echo "=== bench scenario: ${CAP_FILES}-file recorder, bytes slack ==="
sed -e "s/^capacity_files = .*/capacity_files = $CAP_FILES   # bench: the file bound is what binds/" \
    -e "s/^capacity_bytes = .*/capacity_bytes = 1073741824  # bench: 1 GiB, never the binding edge/" \
    "$BASE_SCENARIO" > "$BENCH"
uv run python -m codegen.emit_store --scenario "$BENCH" >/dev/null
bash "$REPO/scripts/mission_build.sh" install > "$WORK/build.log" 2>&1 || {
    echo "bench build failed: tail $WORK/build.log" >&2; tail -5 "$WORK/build.log" >&2; exit 1
}
bash "$REPO/scripts/firmware.sh" check

# --- the ground client: requests, EOT verdicts, and the ledger read ---------
cat > "$WORK/dl.py" <<'PYEOF'
"""One-shot CFDP client for the acknowledged-transfer bench.

Same length-framed uplink as every other probe; two actions:
request (FCS_CFDP_DOWN, class 2 = CF's 1) and storage (watch CamStorage
and print the recorder half). The manifest needs no command anymore -
the store publishes it at boot and at every mutation, so a fetch finds
it current. EOT parsing is the measured wire: nine native u32s at
payload offset 16, then names.
"""

import json
import struct
import sys
import time

from sim.ground_probe import GroundClient, build_cmd

EOT_MID, STORE_MID = 0x08B3, 0x08D5
# CF notices a dead peer on its inactivity timer (30 s, cf_def_config.c);
# the wait is that limit plus margin - patience is the engine's law.
EOT_WAIT_S = 45
# CamStorage payload, little-endian sequential: d + 4Q + 2I + 5Q + B + 7 pad
STORE_FMT = "<dQQQQIIQQQQQB"


def mid_of(packet: bytes) -> int:
    return ((packet[0] & 0x3F) << 8) | packet[1]


def wait_mid(client, mid: int, want: bytes | None, deadline: float) -> bytes | None:
    while time.time() < deadline:
        packet = client.recv_packet(deadline)
        if packet and len(packet) > 16 and mid_of(packet) == mid:
            if want is None or want in packet:
                return packet
    return None


def main() -> int:
    action, tlm, cmd = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
    client = GroundClient(tlm_port=tlm, cmd_port=cmd)
    try:
        if action == "request":
            src, dst = sys.argv[4].encode(), sys.argv[5].encode()
            # DestEntityId, CfdpClass: CF's numbering - 1 means class 2,
            # the acknowledged one; the receipt is what this bench is for.
            payload = struct.pack("<IB3x", 23, 1) + src.ljust(64, b"\0") + dst.ljust(64, b"\0")
            client.send_packet(build_cmd(0x18D6, 0, payload))
            evicted_before = None  # cumulative counters, tracked relative
            deadline = time.time() + EOT_WAIT_S
            packet = None
            # Same wait as the archive era, but sniffing CamStorage on the
            # way: the evictions standing BEFORE this receipt are the
            # baseline the queue's reaction is measured against.
            while time.time() < deadline:
                p = client.recv_packet(deadline)
                if p and len(p) >= 16 + struct.calcsize(STORE_FMT) and mid_of(p) == STORE_MID:
                    f = struct.unpack(STORE_FMT, p[16 : 16 + struct.calcsize(STORE_FMT)])
                    if evicted_before is None:
                        evicted_before = f[9]
                elif p and len(p) > 16 and mid_of(p) == EOT_MID and dst in p:
                    packet = p
                    break
            if not packet:
                print(json.dumps({"error": f"no EOT for {dst.decode()}"}))
                return 1
            words = struct.unpack("<9I", packet[16:52])
            # The queue-law witness: stay on THIS connection after the EOT.
            # A receipt must move FilesEvicted within milliseconds (bsk_link
            # recycles in the EOT handler); the old archive law would not
            # evict until the next capture, ~4 s away on this bench. The
            # gap plus the files_now dip are what only the queue produces.
            t_eot = time.monotonic()
            evicted_after = evicted_before
            min_files = None
            t_evict = None
            while time.monotonic() - t_eot < 3.0:
                p = client.recv_packet(time.monotonic() + 0.5)
                if p and len(p) >= 16 + struct.calcsize(STORE_FMT) and mid_of(p) == STORE_MID:
                    f = struct.unpack(STORE_FMT, p[16 : 16 + struct.calcsize(STORE_FMT)])
                    files_now, evicted = f[6], f[9]
                    if min_files is None or files_now < min_files:
                        min_files = files_now
                    if evicted_after is None or evicted > evicted_after:
                        evicted_after = evicted
                        if t_evict is None:
                            t_evict = time.monotonic() - t_eot
            print(
                json.dumps(
                    {
                        "dst": dst.decode(),
                        "direction": words[2],
                        "state": words[3],
                        "txn_stat": words[4],
                        "fsize": words[7],
                        "evicted_before": evicted_before,
                        "evicted_after": evicted_after,
                        "evict_gap_s": round(t_evict, 2) if t_evict is not None else None,
                        "min_files_after": min_files,
                    }
                )
            )
            return 0
        if action == "storage":
            watch = float(sys.argv[4]) if len(sys.argv) > 4 else 12.0
            deadline = time.time() + watch
            last = None
            while time.time() < deadline:
                packet = client.recv_packet(deadline)
                if packet and len(packet) >= 16 + struct.calcsize(STORE_FMT):
                    if mid_of(packet) == STORE_MID:
                        last = packet
            if last is None:
                print(json.dumps({"error": "no CamStorage during the watch"}))
                return 1
            fields = struct.unpack(STORE_FMT, last[16 : 16 + struct.calcsize(STORE_FMT)])
            names = (
                "sim_time_s frame files_stored bytes_stored capacity_bytes capacity_files "
                "files_now used_bytes generation files_evicted captures_lost "
                "transfers_acked state"
            ).split()
            print(json.dumps(dict(zip(names, fields))))
            return 0
        raise SystemExit(f"unknown action {action}")
    finally:
        client.close()


main()
PYEOF

dl() { PYTHONPATH="$REPO" uv run python "$WORK/dl.py" "$@"; }
request() { dl request "$BEACON" "$UPLINK" "$1" "$2"; }
storage() { dl storage "$BEACON" "$UPLINK" "${1:-12}"; }
field() { grep -o "\"$1\": [0-9-]*" <<<"$2" | head -1 | grep -o '[0-9-]*$'; }
num() { grep -o "\"$1\": [0-9.]*" <<<"$2" | head -1 | grep -o '[0-9.]*$'; }

receipt_proves_queue() { # <json> <want-evicted-after> <slack-files>
    # The queue law's signature, three ways: the eviction count moved
    # (the receipt was spent), it moved WITHIN 2.5 s of the EOT (the next
    # capture is ~4 s away, so NOT a capture forced it), and the ledger
    # showed one frame standing below the cap that nobody asked to free.
    local json="$1" want="$2" slack="$3" gap
    [ "$(field evicted_after "$json")" = "$want" ] || {
        echo "the receipt did not recycle on the spot (evicted_after=$(field evicted_after "$json"), want $want): $json"
        return 1
    }
    gap="$(num evict_gap_s "$json")"
    [ -n "$gap" ] || { echo "no eviction frame seen after the EOT at all: $json"; return 1; }
    awk -v g="$gap" 'BEGIN{exit !(g < 2.5)}' || {
        echo "the eviction came $g s after the receipt - that is a capture, not a queue: $json"
        return 1
    }
    [ "$(field min_files_after "$json")" = "$slack" ] || {
        echo "the ledger never showed ${slack} files (the moment of slack the queue creates): $json"
        return 1
    }
}

dl_expect() { # <src> <dst> -> EOT verdict on stdout, gated on txn_stat
    local json
    json="$(request "$1" "$2")" || true
    grep -q '"error"' <<<"$json" && { echo "downlink failed: $json" >&2; return 1; }
    grep -q '"txn_stat": 0' <<<"$json" || { echo "EOT txn_stat not NO_ERROR: $json" >&2; return 1; }
    echo "$json"
}

craft_wait() { # <pattern> <timeout-s> [min-count]
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

start_world() {
    rm -rf "$HOST_IMG"
    CONTAINER="$("$RUNTIME" run -d --network host --user "$(id -u):$(id -g)" -e HOME=/tmp \
        -e ORRERY_VEHICLE=chief \
        -e ORRERY_TLM_PORT="$BEACON" -e ORRERY_CMD_PORT="$UPLINK" \
        -e ORRERY_ZMQ_SENSOR_ENDPOINT="tcp://127.0.0.1:$SENSOR" \
        -e ORRERY_ZMQ_CMD_ENDPOINT="tcp://127.0.0.1:$LINK_CMD" \
        -v "$REPO:/work" orrery-cfs:dev \
        bash -c "cd /work/cfs_mission/mission/build/exe/cpu1 && ./container-start")"
    echo "=== waiting for gs_tcp + fcs + cam + the recorder (max ${BOOT_TIMEOUT}s) ==="
    local waited=0
    while [ "$waited" -lt "$BOOT_TIMEOUT" ]; do
        local logs
        logs="$("$RUNTIME" logs "$CONTAINER" 2>&1 || true)"
        if grep -q "gs_tcp downlink :$BEACON uplink :$UPLINK" <<<"$logs" \
            && grep -q "FCS App Initialized" <<<"$logs" \
            && grep -q "CAM App Initialized" <<<"$logs" \
            && grep -q "recorder chief/cam" <<<"$logs"; then
            break
        fi
        if grep -qE "(GS_TCP|FCS|BSK_LINK|CAM).*(fail|DEAD)|Could not load file|ES Load Module Error" <<<"$logs"; then
            echo "FAILED (app init error)"; grep -E "GS_TCP|FCS|BSK_LINK|CAM |Could not load" <<<"$logs" | tail -12
            return 1
        fi
        sleep 1
        waited=$((waited + 1))
    done
    logs="$("$RUNTIME" logs "$CONTAINER" 2>&1 || true)"
    grep -q "recorder chief/cam: 1073741824 B, $CAP_FILES files" <<<"$logs" || {
        echo "FAILED (the craft did not claim the bench recorder within ${BOOT_TIMEOUT}s)"
        "$RUNTIME" logs --tail 10 "$CONTAINER" 2>&1 | sed 's/^/    /' || true
        return 1
    }
    echo "=== the CFDP engine must be dialled in before any request ==="
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

    echo "=== staging the patient renderer ==="
    RUNTIME="$RUNTIME" bash "$REPO/scripts/renderer.sh" stage "$BENCH" "$RENDERER"

    echo "=== camera world (${BENCH_NAME}, ${SIM_DURATION} sim-s) ==="
    PYTHONUNBUFFERED=1 uv run python -m sim.forge_scenario --scenario "$BENCH" \
        --duration "$SIM_DURATION" --zmq --pace >"$WORK/sim.log" 2>&1 &
    SIM_PID=$!
    sleep 3
    kill -0 "$SIM_PID" 2>/dev/null || {
        echo "world died at startup: a host link port is already taken?" >&2
        tail -3 "$WORK/sim.log" >&2 || true; return 1
    }
}

wait_store_full() { # the recorder announces the bound; that is the gate
    craft_wait "recorder FULL" 150 || return 1
}

# ===========================================================================
echo "=== FLIGHT: the six-file recorder fills, then receipts buy memory ==="
start_world
wait_store_full

echo "=== baseline: nothing received, nothing recycled ==="
BASE="$(storage 12)" || { echo "ledger deaf: $BASE"; exit 1; }
echo "baseline: $BASE"
[ "$(field transfers_acked "$BASE")" = "0" ] || {
    echo "a receipt existed before any request: $(field transfers_acked "$BASE")"; exit 1;
}
[ "$(field files_evicted "$BASE")" = "0" ] || { echo "bytes vanished before any receipt"; exit 1; }
[ "$(field files_now "$BASE")" = "$CAP_FILES" ] || {
    echo "the full recorder reports files_now=$(field files_now "$BASE")"; exit 1;
}
[ "$(field state "$BASE")" = "2" ] || { echo "a refused recorder must report FULL (2)"; exit 1; }

echo "=== inventory first: the bench picks frames BY MANIFEST NAME, no command sent ==="
MANIFEST="${PREFIX}manifest0.txt"
dl_expect /cf/manifest.txt "$MANIFEST" >/dev/null
"$RUNTIME" exec "$BUCKET" cat "$BUCKET_DIR/$MANIFEST" > "$WORK/manifest0.csv"
PICK_A="$(sed -n '2p' "$WORK/manifest0.csv" | cut -d, -f1)"
PICK_B="$(sed -n '3p' "$WORK/manifest0.csv" | cut -d, -f1)"
PICK_C="$(sed -n '4p' "$WORK/manifest0.csv" | cut -d, -f1)"
[ -n "$PICK_A" ] && [ -n "$PICK_B" ] && [ -n "$PICK_C" ] || {
    echo "the inventory listed fewer than three frames"; exit 1;
}
echo "A=$PICK_A  B=$PICK_B  C=$PICK_C"

echo "=== LEG receipt: A crosses under class 2; the receipt IS the recycle ==="
A_JSON="$(dl_expect "$PICK_A" "${PREFIX}A.png")"
echo "receipt reaction: $A_JSON"
receipt_proves_queue "$A_JSON" 1 $((CAP_FILES - 1)) || exit 1
ACKED_A="$(storage 12)"
[ "$(field transfers_acked "$ACKED_A")" -ge 1 ] || {
    echo "A arrived byte-exact but TransfersAcked stayed $(field transfers_acked "$ACKED_A")"; exit 1;
}
craft_wait "recycled" 30 1 || {
    echo "the wire said recycled and the event bus never spoke it"; exit 1;
}

echo "=== LEG dead pass: B is asked of a link that is not there ==="
TOTAL_BEFORE="$("$RUNTIME" logs "$CONTAINER" 2>&1 | grep -c "engine connected" || true)"
CLOSED_BEFORE="$("$RUNTIME" logs "$CONTAINER" 2>&1 | grep -c "cfdp link closed (peer closed)" || true)"
uv run python - <<'PY'
from sim.cosmos_api import login, rpc

rpc(login(), "disconnect_interface", "CFDP_INT")
print("disconnect issued")
PY
craft_wait "cfdp link closed (peer closed)" 30 $((CLOSED_BEFORE + 1))
B_JSON="$(request "$PICK_B" "${PREFIX}B.png")" || true
echo "dead-pass verdict: ${B_JSON:-no json}"
grep -q '"txn_stat": 0' <<<"$B_JSON" && {
    echo "an interrupted transfer reported a clean close: the receipt means nothing"; exit 1;
} || true
ACKED_B="$(storage 14)"
[ "$(field transfers_acked "$ACKED_B")" = "$(field transfers_acked "$ACKED_A")" ] || {
    echo "TransfersAcked moved on a transfer ground never answered"; exit 1;
}

echo "=== LEG recovery: the station redials; C crosses and earns its place ==="
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
C_JSON="$(dl_expect "$PICK_C" "${PREFIX}C.png")"
echo "recovery reaction: $C_JSON"
receipt_proves_queue "$C_JSON" 2 $((CAP_FILES - 1)) || exit 1
craft_wait "recycled" 30 2 || {
    echo "C was received and proven, yet the recorder never spent that receipt"; exit 1;
}

echo "=== landing: the disk and the wire must agree ==="
FINAL="$(storage 14)"
echo "final: $FINAL"
[ "$(field transfers_acked "$FINAL")" -eq 2 ] || {
    echo "TransfersAcked=$(field transfers_acked "$FINAL"), want 2 (A and C only)"; exit 1;
}
[ "$(field files_evicted "$FINAL")" -eq 2 ] || {
    echo "FilesEvicted=$(field files_evicted "$FINAL"), want exactly 2"; exit 1;
}
[ "$(field files_now "$FINAL")" -le "$CAP_FILES" ] || { echo "occupancy passed the declared cap"; exit 1; }

"$RUNTIME" logs "$CONTAINER" > "$WORK/craft.log" 2>&1
RECYCLED_LINES="$(grep -c "recycled" "$WORK/craft.log" || true)"
[ "${RECYCLED_LINES:-0}" -eq 2 ] || {
    echo "expected 2 recycled lines, found $RECYCLED_LINES"; grep "recycled" "$WORK/craft.log" | sed 's/^/    /' >&2; exit 1;
}
grep "recycled" "$WORK/craft.log" | grep -q "$(basename "$PICK_B")" && {
    echo "B was recycled on a pass that died: unproven bytes deleted"; exit 1;
}
grep "recycled" "$WORK/craft.log" | grep -q "$(basename "$PICK_A")" || {
    echo "A, the oldest proven frame, was not the first recycled"; exit 1;
}
grep "recycled" "$WORK/craft.log" | grep -q "$(basename "$PICK_C")" || {
    echo "C earned a receipt the recorder ignored"; exit 1;
}

[ -f "$HOST_IMG/$(basename "$PICK_B")" ] || { echo "B vanished from /cf/img"; exit 1; }
[ ! -f "$HOST_IMG/$(basename "$PICK_A")" ] || { echo "A should have been recycled"; exit 1; }

echo "=== inventory law: the store's index is true at fetch, and boot signed it ==="
MANIFEST1="${PREFIX}manifest1.txt"
dl_expect /cf/manifest.txt "$MANIFEST1" >/dev/null
"$RUNTIME" exec "$BUCKET" cat "$BUCKET_DIR/$MANIFEST1" > "$WORK/manifest1.csv"
{ grep -q "$(basename "$PICK_B")" "$WORK/manifest1.csv" && ! grep -q "$(basename "$PICK_A")" "$WORK/manifest1.csv"; } || {
    echo "the final inventory disagrees with the store:"; head -8 "$WORK/manifest1.csv" >&2; exit 1;
}
# Every line must name a file really standing: the manifest may lag a
# capture that arrived mid-fetch (the store moves), but it may never lead
# with a file that does not exist. The store never promised a command;
# it owes only this honesty.
tail -n +2 "$WORK/manifest1.csv" | cut -d, -f1 | while read -r listed; do
    [ -f "$HOST_IMG/$(basename "$listed")" ] || {
        echo "inventory lists $listed - no such file stands"; exit 1;
    }
done
# Dump first: docker logs | grep -q dies on SIGPIPE+pipefail (grep quits
# early, docker eats 141, and the pipeline reports failure ON A MATCH).
"$RUNTIME" logs "$CONTAINER" > "$WORK/craft_final.log" 2>&1 || true
if ! grep -q "inventory published" "$WORK/craft_final.log"; then
    echo "boot signed no inventory: the census never published /cf/manifest.txt" >&2
    exit 1
fi
# And the retired door stays shut: the old clerk's line must never appear.
if grep -q "manifest written" "$WORK/craft_final.log"; then
    echo "the camera app still writes inventories: the clerk was not retired" >&2
    exit 1
fi

echo "CAM ACK ACCEPTANCE OK: A+C proven and recycled, B unproven and untouched, inventory true"
