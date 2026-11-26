#!/usr/bin/env bash
# Recorder acceptance: a full store says no, and spends only real receipts.
#
#   The bench scenario is camera_record.toml with the recorder sized to
#   five files and a byte bound that never binds. And the store is NOT
#   empty at boot: four files stand under /cf/img with a catalog proving
#   the ground holds two of them - exactly what a restart looks like. The
#   bench grades two laws at once:
#
#   inheritance   the boot carries the receipts ("proven=2"), the recorder
#   spends them the way the queue law says - the next captures recycle the
#   two inherited proven files, oldest first - and no more: the unproven
#   inherited files stand, untouched.
#
#   no-rumor      nothing is REQUESTED on this bench, so no live frame
#   earns a receipt this boot, and no recycled line may ever name a
#   frame_*.png. A live frame deleted here means the craft erased bytes
#   it only hoped the ground held. The wire verdict, the transitions on
#   the event bus, and the files on disk must all agree.
#
# Usage: scripts/check_cam_store.sh          (SIM_DURATION= seconds, default 90)
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
bash "$REPO/scripts/firmware.sh" check
RUNTIME="${RUNTIME:-docker}"
SIM_DURATION="${SIM_DURATION:-90}"
PROBE_DURATION="${PROBE_DURATION:-55}"
BENCH_NAME="bench_store.toml"
BENCH="$REPO/sim/scenarios/$BENCH_NAME"
WORK="${WORK:-/tmp/opencode/cam_store}"
mkdir -p "$WORK"
HOST_IMG="$REPO/cfs_mission/mission/build/exe/cpu1/cf/img"
CAPACITY_FILES=5

cd "$REPO"
cleanup() {
    bash "$REPO/scripts/fleet.sh" down "sim/scenarios/$BENCH_NAME" >/dev/null 2>&1 || true
    rm -f "$BENCH"
    rm -rf "$HOST_IMG"  # inherited files are this bench's set; the yard
    #                      must not inherit a dead scenario's store
}
trap cleanup EXIT

# The yard door refuses to boot a world while another scenario owns the
# plane; start from a quiet yard like every other fleet check.
left="$("$RUNTIME" ps --format '{{.Names}}' | grep -E '^orrery-(world|chief|deputy)$' || true)"
[ -z "$left" ] || { echo "check_cam_store: yard busy: $left" >&2; exit 1; }

# The bench scenario: the recording plant with a recorder sized in FILEs,
# bytes deliberately slack, so the file bound - not the PNG budget - is
# what fills. Five is the smallest count that shows the watermark as a
# step (4/5 = 80%) before the bound, not as a skip past it.
sed -e "s/^capacity_files = .*/capacity_files = $CAPACITY_FILES  # bench: the file bound is what binds/" \
    -e "s/^capacity_bytes = .*/capacity_bytes = 1073741824  # bench: 1 GiB, never the binding edge/" \
    "$REPO/sim/scenarios/camera_record.toml" > "$BENCH"
rm -rf "$HOST_IMG"

# ---- inherited store: what a restart looks like, without needing one. ----
# Four files stand under /cf/img and the catalog claims the ground holds
# two: a and b are PROVEN, c and d are not. The mtimes order them a<b<c<d
# and keep all four older than any live frame, so the queue law's first
# two spends must name a then b, and c, d, and every live frame must
# stand. (Names avoid frame_* so the live-frame count stays countable.)
mkdir -p "$HOST_IMG"
i=1
for n in a b c d; do
    head -c 4096 /dev/urandom >"$HOST_IMG/inherited_$n.png"
    touch -d "@100000000$i" "$HOST_IMG/inherited_$n.png"
    i=$((i + 1))
done
{
    echo "path,size_bytes,created_utc,proven"
    echo "/cf/img/inherited_a.png,4096,1000000001,1"
    echo "/cf/img/inherited_b.png,4096,1000000002,1"
    echo "/cf/img/inherited_c.png,4096,1000000003,0"
    echo "/cf/img/inherited_d.png,4096,1000000004,0"
} >"$HOST_IMG/.catalog"
echo "=== inherited store: 4 files stand, catalog proves 2 ==="

echo "=== bench world up (${BENCH_NAME}, ${SIM_DURATION} sim-s, recorder ${CAPACITY_FILES} files) ==="
DURATION="$SIM_DURATION" bash "$REPO/scripts/fleet.sh" up "sim/scenarios/$BENCH_NAME" 2>&1 |
    tail -3 | tee "$WORK/up.log"

# Both halves of the claim before anything is graded: the sim's outlet is
# open and the craft announced the recorder the scenario sized for it.
for craft in orrery-world orrery-chief; do
    for _ in $(seq 1 "${BOOT_WAIT_ITERS:-20}"); do
        "$RUNTIME" logs "$craft" 2>&1 | grep -qE "store outlet ON|recorder chief/cam" && break
        sleep 2
    done
done
"$RUNTIME" logs orrery-chief > "$WORK/chief_early.log" 2>&1
grep -q "recorder chief/cam: 1073741824 B, $CAPACITY_FILES files" "$WORK/chief_early.log" || {
    echo "check_cam_store: the craft never claimed the recorder the scenario sized" >&2
    exit 1
}

echo "=== probe: the recorder law off the beacon (${PROBE_DURATION} s) ==="
set +e
timeout "$((PROBE_DURATION + 20))" uv run python -m sim.cam_probe --mode store \
    --duration "$PROBE_DURATION" >"$WORK/probe.log" 2>&1
PROBE_RC=$?
set -e
tail -14 "$WORK/probe.log"
grep -q "CAM STORE OK" "$WORK/probe.log" || {
    echo "check_cam_store: the wire verdict was not CAM STORE OK (rc=$PROBE_RC)" >&2
    exit 1
}

echo "=== craft log: receipts carried, receipts spent, and nothing else ==="
"$RUNTIME" logs orrery-chief > "$WORK/chief.log" 2>&1
"$RUNTIME" logs orrery-world > "$WORK/world.log" 2>&1
grep -q "recorder restored: files=4 bytes=.* proven=2 carried" "$WORK/chief.log" || {
    echo "check_cam_store: the boot did not carry the two receipts the catalog claimed" >&2
    grep "recorder restored" "$WORK/chief.log" >&2 || true
    exit 1
}
grep -q "80% full (WARN)" "$WORK/chief.log" || {
    echo "check_cam_store: the watermark never spoke at 4 of $CAPACITY_FILES" >&2
    exit 1
}
grep -q "recorder FULL" "$WORK/chief.log" || {
    echo "check_cam_store: the bound never announced itself" >&2
    exit 1
}
grep -q "refused: recorder full" "$WORK/chief.log" || {
    echo "check_cam_store: no refused capture in ${SIM_DURATION}s against a 5-file recorder" >&2
    exit 1
}
# The recycled lines are allowed to speak - twice, and only the inherited
# proven names. A live frame named here is the bench's whole reason to
# exist failing; a third recycled line is a receipt conjured from nothing.
RECYCLED="$(grep 'recycled ' "$WORK/chief.log" | grep -o '/cf/img/[^ ]*')"
if [ "$(grep -c 'recycled ' "$WORK/chief.log")" != "2" ]; then
    echo "check_cam_store: want exactly 2 recycled lines, found:" >&2
    grep 'recycled ' "$WORK/chief.log" >&2 || true
    exit 1
fi
grep 'recycled ' "$WORK/chief.log" | grep -q "frame_" && {
    echo "check_cam_store: a LIVE frame was recycled although no transfer ever ran this boot" >&2
    exit 1
}
[ "$(head -1 <<<"$RECYCLED")" = "/cf/img/inherited_a.png" ] || {
    echo "check_cam_store: first spend was not the oldest proven file: $RECYCLED" >&2
    exit 1
}
[ "$(tail -1 <<<"$RECYCLED")" = "/cf/img/inherited_b.png" ] || {
    echo "check_cam_store: second spend was not the next proven file: $RECYCLED" >&2
    exit 1
}

# The store itself agrees with both witnesses. Occupancy is live frames
# PLUS the two inherited unproven survivors: together they fill the cap -
# the survivors stand because no receipt ever touched them, and every
# capture after the two spends is refused.
LANDED="$(find "$HOST_IMG" -name 'frame_*.png' | wc -l)"
[ "$LANDED" -eq "$((CAPACITY_FILES - 2))" ] || {
    echo "check_cam_store: /cf/img holds $LANDED live files, want cap minus the 2 inherited survivors" >&2
    exit 1
}
[ ! -f "$HOST_IMG/inherited_a.png" ] && [ ! -f "$HOST_IMG/inherited_b.png" ] || {
    echo "check_cam_store: an inherited PROVEN file still stands - the receipt was never spent" >&2
    exit 1
}
[ -f "$HOST_IMG/inherited_c.png" ] && [ -f "$HOST_IMG/inherited_d.png" ] || {
    echo "check_cam_store: an inherited UNPROVEN file vanished - a claim the catalog never made" >&2
    exit 1
}

echo "CAM STORE ACCEPTANCE OK: receipts carried and spent on the two inherited proven files, no live frame touched, watermark + bound + refusal all spoken, $LANDED live files on disk"
