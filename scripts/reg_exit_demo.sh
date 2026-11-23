#!/usr/bin/env bash
# REG exit flight (D13): one manifest line lands a registered part, live.
#
#   scripts/reg_exit_demo.sh          (~10 min; COSMOS stack must be up)
#
# The ROADMAP's REG exit criterion, flown rather than asserted offline:
# "adding an already-registered part to a vehicle is one TOML line and a
# regen, zero C edits, and acceptance exhibits arrive for it automatically".
#
# One vehicle, two manifests, derived in a scratch dir from chief.toml:
#
#   probe_bare.toml     the ops plant with its starTracker block removed
#   probe_fitted.toml   probe_bare + the block THIS script writes below
#
# Both fly the same fleet routing (the vehicle keeps the name `chief`, so it
# keeps instance 0 - the legacy port set wire_audit listens on and the targets
# COSMOS already routes). For each manifest the script captures the beacon
# through sim/wire_audit and runs the UNMODIFIED p5 client
# (ORRERY_SCENARIO=<manifest> sim.truth_exhibits). Bare must show the part
# absent on both sides; fitted must show 0x08D0 streaming and p5's attitude
# exhibit judging it. Nothing else is touched: the script finishes by proving
# `git status` is clean, then restores the operational perpetual fleet.
#
# No regen, no rebuild, no gem bump here - and that is the finding, not an
# omission: the part's identity was compiled once into the superset firmware
# (World A), so a manifest gaining the part moves only the WORLD. A new
# VEHICLE name would additionally cost a defs regen + plugin/gem bump, because
# target routing is ground identity - named in the journal, not hidden.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export OPENC3_API_PASSWORD="${OPENC3_API_PASSWORD:-orrery-dev}"
export DURATION="${DURATION:-300}"
WORK="${WORK:-/tmp/opencode/reg_exit}"
OPERATIONAL="sim/scenarios/twin_ops.toml"
cd "$REPO"

if ! docker ps --format '{{.Names}}' | grep -q '^cosmos-openc3-operator-1$'; then
    echo "COSMOS CE not running - run scripts/cosmos_stack.sh first" >&2
    exit 1
fi

mkdir -p "$WORK"
BARE="$WORK/probe_bare.toml"
FITTED="$WORK/probe_fitted.toml"
SRC="sim/scenarios/chief.toml"

# The receipts below include "this flight edited nothing", so the tree's state
# at takeoff is the comparison - the flight is honest mid-worktree too.
GIT_AT_TAKEOFF="$(git status --porcelain)"

# Sweep the whole yard, not just one scenario: a leftover world from another
# file shares this vehicle's ports and its containers, and two worlds on one
# link is not an experiment. (fleet.sh up only clears its OWN scenario.)
echo "--- clearing the yard ---"
bash scripts/fleet.sh down >/dev/null 2>&1 || true

# ---- the two manifests -----------------------------------------------------
# Strip exactly one starTracker block: the filter insists on the count, so a
# chief.toml that stops matching this demo fails the demo instead of silently
# demonstrating something else.
uv run python - "$SRC" "$BARE" <<'PY'
import re
import sys
from pathlib import Path

src, dst = Path(sys.argv[1]), Path(sys.argv[2])
head, *blocks = src.read_text(encoding="utf-8").split("[[spacecraft.sensors]]")
keep, dropped = [], 0
for block in blocks:
    if re.search(r'type\s*=\s*"starTracker"', block):
        dropped += 1
    else:
        keep.append(block)
if dropped != 1:
    raise SystemExit(f"{src}: expected exactly one starTracker block, found {dropped}")
dst.write_text(head + "".join("[[spacecraft.sensors]]" + b for b in keep), encoding="utf-8")
print(f"wrote {dst}: {len(keep)} sensor blocks, starTracker removed")
PY

# ---- THE FIT: everything this exit flight adds to the vehicle --------------
cp "$BARE" "$FITTED"
cat >>"$FITTED" <<'TOML'

# ---- THE PART (written by scripts/reg_exit_demo.sh, nothing else) ----------
[[spacecraft.sensors]]
type = "starTracker"  # <- the part line: registry/parts.toml keys row 'attitude' on it
name = "st"
sigma_SB = [0.0, 0.0, 0.0]
white_noise = 1.0e-4  # the sigma p5's two verdicts are read from
TOML

echo "=== THE ENTIRE DIFF BETWEEN THE TWO MANIFESTS ==="
diff -u "$BARE" "$FITTED" || true
echo

cleanup() {
    bash scripts/fleet.sh down "$1" >/dev/null 2>&1 || true
}

# ---- one manifest, both wires ---------------------------------------------
# $1 = scenario file, $2 = "bare" | "fitted"
phase() {
    local scen="$1" tag="$2" log="$WORK/${2}.log"
    : >"$log"
    echo "=== [$tag] world up: $scen ==="
    if ! bash scripts/fleet.sh up "$scen" 2>&1 | tee -a "$log"; then
        echo "[$tag] FAIL: fleet did not come up (see $log)" >&2
        exit 1
    fi
    sleep 30 # let the beacon reach steady state (boot EVS + first frames)

    echo "--- beacon vs generated defs (sim.wire_audit) ---"
    if ! uv run python -m sim.wire_audit 20 >>"$log" 2>&1; then
        echo "[$tag] FAIL: wire audit rejected the beacon (see $log)" >&2
        tail -12 "$log" >&2
        exit 1
    fi
    grep -E "0x08C[ACDEF]|0x08D0|WIRE AUDIT" "$log" | tail -8 | sed 's/^/    /'

    echo "--- the unmodified truth gate, pointed at this manifest ---"
    if ! ORRERY_SCENARIO="$scen" uv run python -m sim.truth_exhibits >>"$log" 2>&1; then
        echo "[$tag] FAIL: the truth gate did not pass (see $log)" >&2
        tail -12 "$log" >&2
        exit 1
    fi
    grep -E "attitude|starTracker declared|gpsReceiver declared|truth R vs|P5 TRUTH CHECK" \
        "$log" | sed 's/^/    /'

    # verdicts, per phase: the part is either honestly absent or honestly aboard
    if [ "$tag" = "bare" ]; then
        if grep -q "0x08D0" "$log"; then
            echo "[$tag] FAIL: 0x08D0 streamed for a vehicle with no star tracker" >&2
            exit 1
        fi
        if ! grep -q "no starTracker declared" "$log"; then
            echo "[$tag] FAIL: p5 did not declare the missing part (absence must be spoken)" >&2
            exit 1
        fi
        echo "[$tag] OK: frame silent on the beacon, absence declared by the gate"
    else
        if ! grep -qE "0x08D0 BSK_LINK_ATTITUDE_STATE_TLM +[0-9]+ B +ok" "$log"; then
            echo "[$tag] FAIL: 0x08D0 did not stream byte-exactly against its def" >&2
            exit 1
        fi
        if ! grep -q "attitude truth-vs-ATTITUDE" "$log"; then
            echo "[$tag] FAIL: p5 streamed the packet but never judged the part" >&2
            exit 1
        fi
        echo "[$tag] OK: 0x08D0 byte-exact on the beacon, exhibit judged by declared sigma"
    fi
    cleanup "$scen"
    echo
}

phase "$BARE" bare
phase "$FITTED" fitted

# ---- the receipts ----------------------------------------------------------
echo "=== what this flight edited ==="
if [ "$(git status --porcelain)" != "$GIT_AT_TAKEOFF" ]; then
    echo "FAIL: the exit flight changed tracked files - a part fit must not:" >&2
    diff <(printf '%s\n' "$GIT_AT_TAKEOFF") <(git status --porcelain) >&2 || true
    exit 1
fi
echo "git status: unchanged from takeoff (0 C edits, 0 registry edits, 0 defs regenerated)"
echo "evidence: $WORK/probe_bare.toml  $WORK/probe_fitted.toml  $WORK/bare.log  $WORK/fitted.log"

if [ "${RESTORE_FLEET:-1}" = "1" ]; then
    # the operator's yard goes back the way it was found: perpetual, not the
    # demo's finite window
    echo "--- restoring the operational fleet ($OPERATIONAL, perpetual) ---"
    DURATION=inf bash scripts/fleet.sh up "$OPERATIONAL" | tail -1
fi
echo "REG EXIT FLIGHT OK"
