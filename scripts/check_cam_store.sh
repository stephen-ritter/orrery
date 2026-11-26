#!/usr/bin/env bash
# Recorder acceptance: a full store says no, and says nothing else.
#
#   The bench scenario is camera_record.toml with the recorder sized to
#   five files and a byte bound that never binds. Frame size follows the
#   scene, so only the FILE bound makes this flight deterministic: five
#   files land, the fourth crosses the 80% watermark, the fifth sits at the
#   bound, and every capture after that is refused. The probe reads the law
#   off the wire (CamStorage: capacity, occupancy, state, counters); the
#   craft log attests the transitions; the store directory counts the files.
#
#   The loud absence is the acceptance too: only class 1 transfers have ever
#   flown this yard, so no receipt has ever been PROVEN, and NOTHING may be
#   recycled. One "recycled" line here would mean the craft deleted bytes
#   it only hoped the ground held - the exact failure the recorder exists
#   to make impossible.
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

echo "=== craft log: transitions, and the absence of forgetting ==="
"$RUNTIME" logs orrery-chief > "$WORK/chief.log" 2>&1
"$RUNTIME" logs orrery-world > "$WORK/world.log" 2>&1
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
if grep -q "recycled" "$WORK/chief.log"; then
    echo "check_cam_store: a file was recycled with no acknowledged transfer flown - unproven bytes deleted" >&2
    grep "recycled" "$WORK/chief.log" >&2
    exit 1
fi

# The store itself agrees with both witnesses: exactly the cap's worth of
# files stand in /cf/img - not one over, none of them anyone's rumor.
LANDED="$(find "$HOST_IMG" -name 'frame_*.png' | wc -l)"
[ "$LANDED" -eq "$CAPACITY_FILES" ] || {
    echo "check_cam_store: /cf/img holds $LANDED files, want exactly $CAPACITY_FILES" >&2
    exit 1
}

echo "CAM STORE ACCEPTANCE OK: 5 landed, 80% warned, bound refused, zero recycled, $LANDED files on disk"
