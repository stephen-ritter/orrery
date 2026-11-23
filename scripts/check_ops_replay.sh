#!/usr/bin/env bash
# P4C ops check - RECONSTRUCTED 2026-11-22 (the original died in the
# untracked-files wipe of that morning; nothing tracked was lost, which is
# exactly why this file and its baselines are now git-added).
#
# The scheduled flight (fly_scheduled.sh: the D8 onboard queue answers the
# same uplink the P3 gate used) must:
#   1. REPEAT itself - schedule the future deterministically (P6 promise);
#   2. DISAGREE with the plain anchored flight - a satellite that executes
#      stored schedules is a different world from one that reacts, and its
#      trajectory digest must show it ("separate worlds").
set -uo pipefail
cd "$(dirname "$0")/.."

BASE=tests/baselines
mkdir -p "$BASE"

# The lockstep sim owns the per-instance link ports; a live fleet squats on
# the same numbers, so start from a quiet sky.
bash scripts/fleet.sh down sim/scenarios/twin_ops.toml >/dev/null 2>&1 || true

out="$(bash scripts/fly_scheduled.sh 2>&1)"
rc=$?
digest="$(grep -oE 'sha256 [0-9a-f]{64}' <<<"$out" | tail -1 | cut -d' ' -f2)"
if [ "$rc" -ne 0 ] || [ -z "$digest" ]; then
    echo "[scheduled] FLIGHT FAILED (rc=$rc, digest='${digest:-none}')"
    tail -8 <<<"$out"
    exit 1
fi
echo "[scheduled] digest $digest"

file="$BASE/scheduled_digest.txt"
if [ ! -f "$file" ]; then
    printf '%s\n' "$digest" >"$file"
    echo "[scheduled] baseline FOUND at $file (first flight of this world)"
elif [ "$(cat "$file")" != "$digest" ]; then
    echo "[scheduled] digest MOVED: baseline $(cat "$file") -> flight $digest"
    echo "    if the physics legitimately changed, re-found and journal it."
    exit 1
else
    echo "[scheduled] baseline $file matched"
fi

anchored="$BASE/pair_anchored_digest.txt"
if [ -f "$anchored" ] && [ "$(cat "$anchored")" = "$digest" ]; then
    echo "[scheduled] digest EQUALS the plain anchored flight - the queue" \
        "changed nothing? separate worlds expected"
    exit 1
fi
[ -f "$anchored" ] &&
    echo "[scheduled] differs from anchored $(cut -c1-8 "$anchored")... - separate worlds, as a queue must be"

echo "P4C OPS CHECK PASSED"
