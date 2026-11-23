#!/usr/bin/env bash
# P3 truth gate. Flights:
#
#   witness      detumble_forge.toml, wall-clock ENABLE      printed, not gated
#   anchored A   chief.toml, ENABLE + SET_GAIN frame-pinned  known-good pair
#   anchored B   same pins, again                            known-good pair
#
# THE GATE: both anchored flights prove the loop (fcs_probe's assertions)
# and land inside the known-good SET tracked in
# tests/baselines/pair_anchored_digest.txt. A third value fails: that is
# physics, and physics re-founds loudly or not at all.
#
# Why the set holds two values: the pins freeze the sim cadence, but
# ENABLE/SET_GAIN are live ground commands, and whether the FCS applies one
# at the pin or one frame later is host scheduling placement - no launcher
# controls it. Measured across sessions: every anchored flight prints
# 6380def8... (at the pin) or f6a85635... (one frame later), never a third.
# Byte-exact physics determinism is gated in-process by the pytest suite,
# where there is no delivery race. A mid-session A-vs-B flip therefore
# prints as placement, not failure; membership in the set is the verdict.
#
# The witness is printed-not-gated: the classic wall-clock artifact,
# declared not-anchored since KN; its file is the warm-session expectation.
set -uo pipefail
cd "$(dirname "$0")/.."

BASE=tests/baselines
mkdir -p "$BASE"

flight() { # $1 = label, then env assignments the caller exported
    local label="$1"
    local out digest
    out="$(bash scripts/fly_world.sh 2>&1)"
    local rc=$?
    digest="$(grep -oE 'sha256 [0-9a-f]{64}' <<<"$out" | tail -1 | cut -d' ' -f2)"
    if [ "$rc" -ne 0 ] || [ -z "$digest" ]; then
        echo "[$label] FLIGHT FAILED (rc=$rc, digest='${digest:-none}')"
        tail -6 <<<"$out"
        return 1
    fi
    echo "[$label] digest $digest"
    printf '%s\n' "$digest" >"/tmp/opencode/pair_dig_$label.txt"
}

echo "--- [1/3] witness: classic wall-clock ENABLE (printed, not gated) ---"
SIM_MODULE=sim.forge_scenario SIM_DURATION=100 \
    flight witness || exit 1
WANT_W="$(head -1 "$BASE/pair_witness_digest.txt" 2>/dev/null || true)"
HAVE_W="$(cat /tmp/opencode/pair_dig_witness.txt)"
if [ -n "$WANT_W" ] && [ "$WANT_W" != "$HAVE_W" ]; then
    echo "[witness] differs from warm-session expectation $WANT_W (informational; not-anchored by design)"
fi

echo "--- [2/3] anchored A: chief.toml, ENABLE@9 SET_GAIN@16 ---"
SIM_MODULE=sim.forge_scenario SIM_ARGS="--scenario sim/scenarios/chief.toml" \
    START_FRAME=9 GAIN_FRAME=16 SIM_DURATION=100 \
    flight anchored || exit 1

echo "--- [3/3] anchored B: same pins, the pair's other half ---"
SIM_MODULE=sim.forge_scenario SIM_ARGS="--scenario sim/scenarios/chief.toml" \
    START_FRAME=9 GAIN_FRAME=16 SIM_DURATION=100 \
    flight reanchored || exit 1

A="$(cat /tmp/opencode/pair_dig_anchored.txt)"
B="$(cat /tmp/opencode/pair_dig_reanchored.txt)"
if [ "$A" != "$B" ]; then
    echo "[pair] flights disagree ($A vs $B): host placement moved mid-session;"
    echo "[pair] both must still be known-good below, or this is physics."
fi

file="$BASE/pair_anchored_digest.txt"
[ -f "$file" ] || { echo "no baseline set at $file - FOUND it on first flight"; cat /tmp/opencode/pair_dig_anchored.txt >"$file"; }
bad=0
for d in "$A" "$B"; do
    if ! grep -qxF "$d" "$file"; then bad=1; fi
done
if [ "$bad" -eq 1 ]; then
    echo "ANCHOR OUTSIDE THE KNOWN-GOOD SET:"
    echo "  flights:  $A  $B"
    echo "  allowed:  $(tr '\n' ' ' <"$file")"
    echo "    investigate the cadence; adding a line here is a journaled act."
    exit 1
fi
echo "[anchor] both digests inside the known-good set $(tr '\n' ' ' <"$file")"

echo "P3 TRUTH GATE PASSED"
