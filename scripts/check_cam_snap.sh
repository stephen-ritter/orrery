#!/usr/bin/env bash
# Shutter-pulse acceptance: "look now" buys exactly one NEW frame, or nothing.
#
#   The bench scenario is camera_snap.toml: its outlet door is DECLARED
#   pulse-only (record_mode = "snap"), so no cadence door exists - an eye
#   that streams forever still files nothing but SNAP deliveries. Every
#   frame that reaches the store on this flight was paid for by a
#   SNAP, and the bench grades what a pulse is:
#
#   next-frame law   an ask is cashed by the capture AFTER the command,
#                    never by an old file handed over early - the probe
#                    prints what the device had captured just before each
#                    pulse, and the delivered capture ordinal must be the
#                    next one (one render of wire lag is allowed).
#
#   shutter law      the first pulse fires while the eye streams; the eye
#                    is then CLOSED, and the second pulse must open it,
#                    deliver exactly one frame, and close it again - the
#                    final CamState view says CLOSED.
#
#   nothing-else law the store holds exactly two files, the world log
#                    shows exactly two sends and both say "shutter pulse",
#                    CAM_HK asked for two and CamState delivered two -
#                    and each landed file is byte-identical to the PNG the
#                    sim's own writer rendered for that capture.
#
# Usage: scripts/check_cam_snap.sh        (SIM_DURATION= / PROBE_DURATION=)
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
bash "$REPO/scripts/firmware.sh" check
RUNTIME="${RUNTIME:-docker}"
SIM_DURATION="${SIM_DURATION:-85}"
PROBE_DURATION="${PROBE_DURATION:-70}"
BENCH_NAME="bench_snap.toml"
BENCH="$REPO/sim/scenarios/$BENCH_NAME"
WORK="${WORK:-/tmp/opencode/cam_snap}"
mkdir -p "$WORK"
HOST_IMG="$REPO/cfs_mission/mission/build/exe/cpu1/cf/img"

cd "$REPO"
cleanup() {
    bash "$REPO/scripts/fleet.sh" down "sim/scenarios/$BENCH_NAME" >/dev/null 2>&1 || true
    rm -f "$BENCH"
    rm -rf "$HOST_IMG"
}
trap cleanup EXIT

left="$("$RUNTIME" ps --format '{{.Names}}' | grep -E '^orrery-(world|chief|deputy)$' || true)"
[ -z "$left" ] || { echo "check_cam_snap: yard busy: $left" >&2; exit 1; }

# The only law the scenario carries for this bench: the outlet door is
# declared pulse-only (record_mode = "snap") - the declaration the yard
# used to fake with record_rate 1000. Everything else - capacity, orbit,
# eye - is the record scenario's, already flown.
cp "$REPO/sim/scenarios/camera_snap.toml" "$BENCH"
rm -rf "$HOST_IMG"

echo "=== bench world up (${BENCH_NAME}, ${SIM_DURATION} sim-s, cadence door closed by declaration) ==="
DURATION="$SIM_DURATION" bash "$REPO/scripts/fleet.sh" up "sim/scenarios/$BENCH_NAME" 2>&1 |
    tail -3 | tee "$WORK/up.log"

for craft in orrery-world orrery-chief; do
    for _ in $(seq 1 "${BOOT_WAIT_ITERS:-20}"); do
        "$RUNTIME" logs "$craft" 2>&1 | grep -qE "store outlet ON|recorder chief/cam" && break
        sleep 2
    done
done
"$RUNTIME" logs orrery-chief > "$WORK/chief_early.log" 2>&1
grep -q "recorder chief/cam" "$WORK/chief_early.log" || {
    echo "check_cam_snap: the craft never claimed its recorder" >&2
    exit 1
}

echo "=== probe: two pulses from the ground (${PROBE_DURATION} s) ==="
set +e
timeout "$((PROBE_DURATION + 20))" uv run python -m sim.cam_probe --mode snap \
    --duration "$PROBE_DURATION" >"$WORK/probe.log" 2>&1
PROBE_RC=$?
set -e
tail -14 "$WORK/probe.log"
grep -q "CAM SNAP OK" "$WORK/probe.log" || {
    echo "check_cam_snap: the wire verdict was not CAM SNAP OK (rc=$PROBE_RC)" >&2
    exit 1
}

echo "=== sim log: the pulse's own bookkeeping ==="
"$RUNTIME" logs orrery-world > "$WORK/world.log" 2>&1
"$RUNTIME" logs orrery-chief > "$WORK/chief.log" 2>&1
[ "$(grep -c 'shutter pulse armed' "$WORK/world.log")" = "2" ] || {
    echo "check_cam_snap: want two armed pulses in the sim log:" >&2
    grep 'shutter pulse' "$WORK/world.log" >&2 || true
    exit 1
}
# The strongest single line on this bench: the cadence door never opened,
# so EVERY send out of the store outlet carries the pulse note. A third
# "store: capture" line, or one without the note, is a frame that landed
# on a rumor instead of an order.
SENDS="$(grep 'store: capture' "$WORK/world.log" || true)"
[ "$(wc -l <<<"$SENDS")" = "2" ] || {
    echo "check_cam_snap: the store outlet sent $(wc -l <<<"$SENDS") frames, want exactly the 2 pulses" >&2
    printf '%s\n' "$SENDS" >&2
    exit 1
}
grep -q "shutter pulse" <<<"$SENDS" || {
    echo "check_cam_snap: a send left without the pulse note - the cadence door opened?" >&2
    exit 1
}
[ "$(grep -c '(shutter pulse)' <<<"$SENDS")" = "2" ] || {
    echo "check_cam_snap: only one pulse's delivery was booked:" >&2
    printf '%s\n' "$SENDS" >&2
    exit 1
}

echo "=== the next-frame law: the pulse pays with what came AFTER the ask ==="
# World-log ordinals are the sim's capture numbers; the probe's pre-pulse
# line is the device's captured count as the ground last saw it. Wire lag
# (one render) means the delivered ordinal is +1 or +2, never +0 (an old
# file handed over) and never more (a frame the ask did not buy).
DELIVERED_1="$(grep '(shutter pulse)' "$WORK/world.log" | head -1 | grep -oE 'capture [0-9]+' | grep -oE '[0-9]+')"
DELIVERED_2="$(grep '(shutter pulse)' "$WORK/world.log" | tail -1 | grep -oE 'capture [0-9]+' | grep -oE '[0-9]+')"
C1="$(grep 'SNAP (pulse-open-leg)' "$WORK/probe.log" | grep -oE 'captured=[0-9]+' | grep -oE '[0-9]+')"
C2="$(grep 'SNAP (pulse-closed-leg)' "$WORK/probe.log" | grep -oE 'captured=[0-9]+' | grep -oE '[0-9]+')"
[ "$DELIVERED_2" -gt "$DELIVERED_1" ] || {
    echo "check_cam_snap: the closed-shutter pulse did not force a frame the stream had not already made" >&2
    exit 1
}
if [ "$DELIVERED_1" -le "$C1" ] || [ $((DELIVERED_1 - C1)) -gt 2 ]; then
    echo "check_cam_snap: pulse 1 delivered capture $DELIVERED_1 from a device at captured=$C1 (want the next frame)" >&2
    exit 1
fi
if [ "$DELIVERED_2" -le "$C2" ] || [ $((DELIVERED_2 - C2)) -gt 2 ]; then
    echo "check_cam_snap: pulse 2 delivered capture $DELIVERED_2 from a device at captured=$C2 (want the next frame)" >&2
    exit 1
fi

echo "=== the store: exactly two files, named by their captures ==="
LANDED="$(find "$HOST_IMG" -name 'frame_*.png' -printf '%f\n' | sort)"
EXPECT="$(printf 'frame_1_%s.png\nframe_1_%s.png' "$DELIVERED_1" "$DELIVERED_2" | sort)"
if [ "$LANDED" != "$EXPECT" ]; then
    echo "check_cam_snap: /cf/img holds [$(tr '\n' ' ' <<<"$LANDED")], want the two pulse captures $DELIVERED_1 and $DELIVERED_2" >&2
    exit 1
fi
if grep -q "refused: recorder full" "$WORK/chief.log"; then
    echo "check_cam_snap: a capture was refused against a slack recorder" >&2
    exit 1
fi

echo "=== byte law: each landed file is the PNG the sim's writer rendered ==="
# The WORLD's own announcement names this run's outlet directory. Globbing
# /tmp is wrong: a reused container carries dead runs' stages, and
# lexicographic "first" grades the byte law against whichever ghost speaks
# loudest (2026-11-27: a dead run's dir, whose pulse landed on a rendered
# tick, made a live capture look byte-less).
SIM_DIR="$(sed -nE 's|.*store outlet ON \([^)]*-> (/tmp/cam_store_[^)]+)\).*|\1|p' \
    "$WORK/world.log" | tail -1)"
[ -n "$SIM_DIR" ] || { echo "check_cam_snap: the sim announced no store directory" >&2; exit 1; }
# And the destination must be empty: docker cp NESTS a directory copy into an
# existing one, so a sim_src left over from a previous leg would grade THIS
# run's frames against the previous run's PNGs sitting in the outer layer
# (the 2026-11-27 scar: seven nested ghosts under cam_snap/sim_src, each a
# faithful copy, none of them what the byte law was reading).
rm -rf "$WORK/sim_src"
"$RUNTIME" cp "orrery-world:$SIM_DIR" "$WORK/sim_src" >/dev/null
uv run python - "$WORK/sim_src" "$HOST_IMG" "$WORK/world.log" <<'PY'
import hashlib
import re
import sys
from pathlib import Path

src_dir, landed_dir, world_log = Path(sys.argv[1]), Path(sys.argv[2]), Path(sys.argv[3])
# The outlet's own log line names the source file it read
# ("capture 21 [43.000000.png] -> ... (shutter pulse)"): the binding is the
# sim's claim, not the bench's guess. The directory holds one PNG per RENDER
# - finer than capture ordinals at recorder granularity - so position in a
# sorted listing would be a guess, and guesses are not acceptance.
pairs = re.findall(
    r"capture (\d+) \[([^\]]+\.png)\][^\n]*\(shutter pulse\)", world_log.read_text()
)
assert len(pairs) == 2, f"want two pulse sends in the sim log, found {pairs}"
for ordinal_s, src_name in pairs:
    landed = landed_dir / f"frame_1_{ordinal_s}.png"
    src = src_dir / src_name
    a = hashlib.sha256(landed.read_bytes()).hexdigest()
    b = hashlib.sha256(src.read_bytes()).hexdigest()
    assert a == b, f"capture {ordinal_s}: landed {landed.name} {a[:16]}... != sim {src.name} {b[:16]}..."
    print(f"byte law OK: capture {ordinal_s} {src.name} -> {landed.name}, sha256 {a[:16]}... identical")
PY

echo "CAM SNAP ACCEPTANCE OK: two asks, two NEW frames, one out of cadence and one out of a closed shutter,"
echo "the store holds exactly those two files, each byte-identical to its render, and the shutter came back closed"
