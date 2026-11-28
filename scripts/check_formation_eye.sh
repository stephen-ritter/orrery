#!/usr/bin/env bash
# Formation-eye acceptance: the sister vehicle through the WHOLE loop.
#
# ONE Basilisk world flies BOTH vehicles; BOTH mission containers answer the
# lockstep gate; Vizard paints the chief's narrow nadir eye with the deputy
# hovering 100 m below; COSMOS talks to SAT_CHIEF and SAT_DEPUTY while the
# same flight runs. Four witnesses, none derived from the file they check:
#
#   W1 physics   the chief's trajectory digest is IDENTICAL between the fleet
#                leg and the deputy-less control leg: the sister vehicle is a
#                passive world-body and pixels have no mass.
#   W2 pixel     paired frames (identical chief state => identical Earth disc)
#                differ ONLY by a compact blob near the frame center; the
#                blob's centroid crawl fits geometry: intercept ~0 px (the
#                deputy starts on the boresight) and slope ~0.2 px/s (the
#                along-track lead of |v_d| - |v_c| of a lower circular orbit,
#                read on the frame's Y: this eye puts along-track on Y).
#   W3 fit       both vehicles stamp a FIT MATCHED verdict, ids named.
#   W4 ground    while the flight runs, COSMOS counts received TRAFFIC on BOTH
#                craft interfaces - the station hears the whole fleet, not
#                just the bird it was born with.
#
# The pixel leg is COMMAND-FREE on purpose (cam_probe --mode control): the
# craft stay in sense mode, so fleet and control legs share one physics.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
bash "$REPO/scripts/firmware.sh" check
RUNTIME="${RUNTIME:-docker}"
BOOT_TIMEOUT="${BOOT_TIMEOUT:-120}"
SIM_DURATION="${SIM_DURATION:-160}"
PROBE_DURATION="${PROBE_DURATION:-45}"
WORK="${WORK:-/tmp/opencode/formation_eye}"
RENDERER="orrery-eye-viz"
SCENARIO="$REPO/sim/scenarios/formation_eye.toml"
ABSENT="$WORK/formation_absent.toml"
mkdir -p "$WORK"
cd "$REPO"

CONTAINERS=()
SIM_PID=""
cleanup() {
    [ -n "$SIM_PID" ] && kill "$SIM_PID" >/dev/null 2>&1 || true
    "$RUNTIME" rm -f "$RENDERER" >/dev/null 2>&1 || true
    for c in ${CONTAINERS[@]+"${CONTAINERS[@]}"}; do
        "$RUNTIME" stop "$c" >/dev/null 2>&1 || true
    done
}
trap cleanup EXIT

# A quiet yard is a precondition, not a hope: this bench boots bare craft that
# need organs the standing fleet's vehicles do not carry, and the shared
# cf/cfe_es_startup.scr carries WHATEVER vehicle booted last (2026-11-25's
# deploy-time tables made that file a landing target; formation_eye 2026-11-27
# lost its eye to a stale landing). fleet down removes the yard - including
# stopped containers that hold this bench's chosen --names - and re-lands the
# union; seconds on a quiet tree.
bash "$REPO/scripts/fleet.sh" down >/dev/null 2>&1 || true
stale="$("$RUNTIME" ps -q --filter ancestor=orrery-cfs:dev || true)"
[ -n "$stale" ] && "$RUNTIME" rm -f $stale >/dev/null || true
"$RUNTIME" rm -f "$RENDERER" >/dev/null 2>&1 || true

# Ports and instances: sim/fleet.py is the single authority (block INDEX =
# port slot); the check derives exactly what fleet.sh would launch.
FACTS=()
mapfile -t FACTS < <(uv run python -c "
from sim.fleet import scenario_vehicles
for v in scenario_vehicles('$SCENARIO'):
    p = v.ports
    print(v.name, v.instance, p.beacon, p.uplink, p.sensor, p.link_cmd)
")
[ "${#FACTS[@]}" -eq 2 ] || { echo "FORMATION-EYE: expected 2 vehicles, got ${#FACTS[@]}" >&2; exit 2; }
CHIEF_FACT="${FACTS[0]}"
DEP_FACT="${FACTS[1]}"
read -r C_NAME C_INST C_BEACON C_UPLINK C_SENSOR C_CMD <<<"$CHIEF_FACT"
read -r D_NAME D_INST D_BEACON D_UPLINK D_SENSOR D_CMD <<<"$DEP_FACT"

# --- the control scenario: this file minus the deputy (derived, not stored) --
uv run python - "$SCENARIO" "$ABSENT" <<'EOF'
import sys
from pathlib import Path
text = Path(sys.argv[1]).read_text()
start = text.index("# --- vehicle 1: deputy")
end = text.index("# the rendered eye needs")
Path(sys.argv[2]).write_text(text[:start] + text[end:])
EOF
grep -q '\[\[camera\]\]' "$ABSENT" || { echo "control scenario lost its eye"; exit 1; }
if grep -q 'name = "deputy"' "$ABSENT"; then echo "control scenario still carries a deputy"; exit 1; fi

rx_bytes() {  # the station's received BYTES for one interface (0 if silent)
    uv run python - "$1" <<'EOF'
import sys
try:
    from sim.cosmos_api import login, rpc
    rows = rpc(login(), "get_all_interface_info")
    for row in rows:
        if row and row[0] == sys.argv[1]:
            print(int(row[6] or 0))  # [6] = rx bytes (rxcount at [5] is proven blind here)
            break
    else:
        print(0)
except Exception:
    print(0)
EOF
}

# --- fleet leg: world FIRST (its listeners must exist before any craft dials
#     - sim/lockstep.py's phantom-peer law), then renderer, then both craft ---
echo "=== [fleet] staging the patient renderer ==="
RUNTIME="$RUNTIME" bash "$REPO/scripts/renderer.sh" stage "$SCENARIO" "$RENDERER"

echo "=== [fleet] formation world (${SIM_DURATION} sim-s, both acks gated) ==="
export ORRERY_FIRST_TICK_WAIT_S="${ORRERY_FIRST_TICK_WAIT_S:-150}"
PYTHONUNBUFFERED=1 uv run python -m sim.forge_scenario --scenario "$SCENARIO" \
    --duration "$SIM_DURATION" --zmq --pace >"$WORK/sim_fleet.log" 2>&1 &
SIM_PID=$!

echo "=== [fleet] waiting for the world's channels to come up ==="
waited=0
while [ "$waited" -lt 90 ]; do
    bound=1
    for fact in "${FACTS[@]}"; do
        read -r name instance beacon uplink sensor link_cmd <<<"$fact"
        ss -tln 2>/dev/null | grep -q ":$sensor " || bound=0
    done
    [ "$bound" -eq 1 ] && break
    kill -0 "$SIM_PID" 2>/dev/null || {
        echo "[fleet] world died before binding channels" >&2
        tail -3 "$WORK/sim_fleet.log" >&2 || true
        exit 1
    }
    sleep 1
    waited=$((waited + 1))
done
[ "$bound" -eq 1 ] || { echo "[fleet] world channels unbound after 90s"; exit 1; }
echo "  channels live after ${waited}s: the craft will knock, never shout into a void"

for fact in "${FACTS[@]}"; do
    read -r name instance beacon uplink sensor link_cmd <<<"$fact"
    CONTAINERS+=("orrery-$name")
    "$RUNTIME" run -d --name "orrery-$name" --network host \
        --user "$(id -u):$(id -g)" -e HOME=/tmp \
        -e ORRERY_TLM_PORT="$beacon" -e ORRERY_CMD_PORT="$uplink" \
        -e ORRERY_ZMQ_SENSOR_ENDPOINT="tcp://127.0.0.1:$sensor" \
        -e ORRERY_ZMQ_CMD_ENDPOINT="tcp://127.0.0.1:$link_cmd" \
        -v "$REPO:/work" orrery-cfs:dev \
        bash -c "cd /work/cfs_mission/mission/build/exe/cpu1 && ./container-start" >/dev/null
done

echo "=== [fleet] waiting for both craft to boot (max ${BOOT_TIMEOUT}s) ==="
for fact in "${FACTS[@]}"; do
    read -r name instance beacon uplink sensor link_cmd <<<"$fact"
    waited=0
    booted=0
    while [ "$waited" -lt "$BOOT_TIMEOUT" ]; do
        logs="$("$RUNTIME" logs "orrery-$name" 2>&1 || true)"
        if grep -q "gs_tcp downlink :$beacon uplink :$uplink" <<<"$logs" &&
            grep -q "FCS App Initialized" <<<"$logs"; then
            if [ "$name" = "$C_NAME" ] && ! grep -q "CAM App Initialized" <<<"$logs"; then
                sleep 1; waited=$((waited + 1)); continue
            fi
            booted=1
            break
        fi
        if grep -qE "(GS_TCP|FCS|BSK_LINK|CAM).*(fail|DEAD)|Could not load file|ES Load Module Error" <<<"$logs"; then
            echo "[fleet] $name FAILED (app init error)"
            grep -E "GS_TCP|FCS|BSK_LINK|CAM |Could not load" <<<"$logs" | tail -12
            exit 1
        fi
        sleep 1
        waited=$((waited + 1))
    done
    [ "$booted" -eq 1 ] || { echo "[fleet] $name boot markers unseen in ${BOOT_TIMEOUT}s"; exit 1; }
    echo "  $name booted (: $beacon)"
done

# W4 opens its window BEFORE the world moves: bytes at rest, bytes after.
CHIEF_RX_BEFORE="$(rx_bytes GS_TCP_INT)" || true
DEP_RX_BEFORE="$(rx_bytes GS_TCP2_INT)" || true
# (A cached-HK "heartbeat" read used to ride here informationally; the
# 2026-11-27 purge retired its packet, and the W4 witness never needed it:
# the interface byte counters above ARE the ground-side proof.)

echo "=== [fleet] raw-port id witness (8 s, chief's eye) ==="
uv run python -m sim.id_witness --row camstate --seconds 8 --instance "$C_INST" \
    >"$WORK/witness_fleet.log" 2>&1 || true
tail -2 "$WORK/witness_fleet.log" | sed 's/^/    /'

echo "=== [fleet] send-nothing probe (${PROBE_DURATION}s): the station watches, the craft hold sense mode ==="
uv run python -m sim.cam_probe --mode control --instance "$C_INST" --duration "$PROBE_DURATION" \
    >"$WORK/probe_fleet.log" 2>&1 || {
        echo "[fleet] control probe rc=$? (witnesses do not need its verdict, but it must not fail)"
        tail -4 "$WORK/probe_fleet.log"
        exit 1
    }

echo "=== [fleet] ground witness: packet counters on BOTH interfaces mid-flight ==="
CHIEF_RX_DURING="$(rx_bytes GS_TCP_INT)"
DEP_RX_DURING="$(rx_bytes GS_TCP2_INT)"

wait "$SIM_PID"; SIM_PID=""
CHIEF_FLEET="$(grep -oE "chief trajectory sha256 [0-9a-f]{64}" "$WORK/sim_fleet.log" | tail -1 | awk '{print $NF}' || true)"
DEP_FLEET="$(grep -oE "deputy trajectory sha256 [0-9a-f]{64}" "$WORK/sim_fleet.log" | tail -1 | awk '{print $NF}' || true)"
[ -n "$CHIEF_FLEET" ] && [ -n "$DEP_FLEET" ] || {
    echo "[fleet] FAILED: per-vehicle digests missing"; tail -5 "$WORK/sim_fleet.log"; exit 1; }

CONSOLE_CHIEF="$WORK/console_chief.log"
CONSOLE_DEP="$WORK/console_deputy.log"
"$RUNTIME" stop "orrery-$C_NAME" >/dev/null 2>&1 || true
"$RUNTIME" logs "orrery-$C_NAME" >"$CONSOLE_CHIEF" 2>&1 || true
"$RUNTIME" rm -f "orrery-$C_NAME" >/dev/null 2>&1 || true
"$RUNTIME" stop "orrery-$D_NAME" >/dev/null 2>&1 || true
"$RUNTIME" logs "orrery-$D_NAME" >"$CONSOLE_DEP" 2>&1 || true
"$RUNTIME" rm -f "orrery-$D_NAME" >/dev/null 2>&1 || true
CONTAINERS=()
"$RUNTIME" rm -f "$RENDERER" >/dev/null 2>&1 || true

FLEET_STORE="$(grep -oE 'camera1 store outlet ON \(every [0-9]+th capture -> [^)]+\)' \
    "$WORK/sim_fleet.log" | tail -1 | sed -E 's/.*-> ([^)]+)\)/\1/')"
[ -n "$FLEET_STORE" ] || { echo "[fleet] no store outlet announced"; exit 1; }
echo "=== [fleet] chief ${CHIEF_FLEET:0:16}... deputy ${DEP_FLEET:0:16}... store $FLEET_STORE ==="

# --- control leg: same chief WITH its craft aboard, world WITHOUT the deputy --
# Container against container: both legs fly the chief's own flight software,
# so the ONLY difference this digest comparison sees is the deputy world-body
# and the pixels it paints. (Comparing a controlled flight against a free-
# flying solo world would grade the control law, not the sister vehicle.)
echo "=== [absent] staging the renderer for the control world ==="
RUNTIME="$RUNTIME" bash "$REPO/scripts/renderer.sh" stage "$ABSENT" "$RENDERER"
echo "=== [absent] control world (${SIM_DURATION} sim-s, chief's craft aboard) ==="
PYTHONUNBUFFERED=1 timeout 900 uv run python -m sim.forge_scenario --scenario "$ABSENT" \
    --duration "$SIM_DURATION" --zmq --pace >"$WORK/sim_absent.log" 2>&1 &
SIM_PID=$!
waited=0
while [ "$waited" -lt 90 ]; do
    ss -tln 2>/dev/null | grep -q ":$C_SENSOR " && break
    kill -0 "$SIM_PID" 2>/dev/null || {
        echo "[absent] control world died before binding"; tail -3 "$WORK/sim_absent.log"; exit 1; }
    sleep 1
    waited=$((waited + 1))
done
CONTAINERS+=("orrery-$C_NAME")
"$RUNTIME" run -d --name "orrery-$C_NAME" --network host \
    --user "$(id -u):$(id -g)" -e HOME=/tmp \
    -e ORRERY_TLM_PORT="$C_BEACON" -e ORRERY_CMD_PORT="$C_UPLINK" \
    -e ORRERY_ZMQ_SENSOR_ENDPOINT="tcp://127.0.0.1:$C_SENSOR" \
    -e ORRERY_ZMQ_CMD_ENDPOINT="tcp://127.0.0.1:$C_CMD" \
    -v "$REPO:/work" orrery-cfs:dev \
    bash -c "cd /work/cfs_mission/mission/build/exe/cpu1 && ./container-start" >/dev/null
waited=0
while [ "$waited" -lt "$BOOT_TIMEOUT" ]; do
    logs="$("$RUNTIME" logs "orrery-$C_NAME" 2>&1 || true)"
    grep -q "gs_tcp downlink :$C_BEACON uplink :$C_UPLINK" <<<"$logs" &&
        grep -q "FCS App Initialized" <<<"$logs" && break
    sleep 1
    waited=$((waited + 1))
done
[ "$waited" -lt "$BOOT_TIMEOUT" ] || { echo "[absent] chief never booted"; exit 1; }
wait "$SIM_PID"; SIM_PID=""
"$RUNTIME" stop "orrery-$C_NAME" >/dev/null 2>&1 || true
CONSOLE_ABSENT="$WORK/console_absent_chief.log"
"$RUNTIME" logs "orrery-$C_NAME" >>"$CONSOLE_ABSENT" 2>&1 || true
"$RUNTIME" rm -f "orrery-$C_NAME" >/dev/null 2>&1 || true
CONTAINERS=()
"$RUNTIME" rm -f "$RENDERER" >/dev/null 2>&1 || true
CHIEF_ABSENT="$(grep -oE 'trajectory sha256 [0-9a-f]{64}' "$WORK/sim_absent.log" | tail -1 | awk '{print $NF}' || true)"
[ -n "$CHIEF_ABSENT" ] || { echo "[absent] no digest"; tail -5 "$WORK/sim_absent.log"; exit 1; }
ABSENT_STORE="$(grep -oE 'camera1 store outlet ON \(every [0-9]+th capture -> [^)]+\)' \
    "$WORK/sim_absent.log" | tail -1 | sed -E 's/.*-> ([^)]+)\)/\1/')"
[ -n "$ABSENT_STORE" ] || { echo "[absent] no store outlet"; exit 1; }
echo "=== [absent] chief ${CHIEF_ABSENT:0:16}... store $ABSENT_STORE ==="

# --- W1: the deputy is passive and pixels have no mass ------------------------
echo "=== W1: the sister vehicle never touched the chief's physics ==="
[ "$CHIEF_FLEET" = "$CHIEF_ABSENT" ] || {
    echo "FORMATION-EYE FAILED (W1): chief digest moved with the deputy/pixels:"
    echo "  fleet  $CHIEF_FLEET"; echo "  absent $CHIEF_ABSENT"; exit 1; }
echo "  identical: ${CHIEF_FLEET:0:24}..."

# --- W2: the pixel witness: the blob is where geometry says it must be --------
echo "=== W2: the capture paints the sister vehicle, and it crawls per ephemeris ==="
uv run python - "$FLEET_STORE" "$ABSENT_STORE" "$WORK/blob_evidence.png" "$SCENARIO" <<'EOF'
import sys
from pathlib import Path

import numpy as np
from PIL import Image

fleet_dir, absent_dir, out = Path(sys.argv[1]), Path(sys.argv[2]), Path(sys.argv[3])
SCENARIO_TOML = Path(sys.argv[4])
fa = {p.name: p for p in fleet_dir.glob("*.png")}
aa = {p.name: p for p in absent_dir.glob("*.png")}
common = sorted(set(fa) & set(aa), key=lambda n: float(n.split(".")[0]))
if len(common) < 40:
    sys.exit(f"W2 FAILED: only {len(common)} paired frames (fleet {len(fa)}, absent {len(aa)})")

samples = []
rejected = 0
first = last = None
from collections import deque


def largest_component_fraction(mask_win, n):
    """One pass flood-label: how much of the diff cloud is the BIGGEST piece."""
    h, w = mask_win.shape
    seen = bytearray(h * w)
    best = 0
    flat = mask_win.ravel()
    for idx in range(h * w):
        if not flat[idx] or seen[idx]:
            continue
        seen[idx] = 1
        q = deque([idx])
        size = 0
        while q:
            p = q.popleft()
            size += 1
            py, px = divmod(p, w)
            for dy in (-1, 0, 1):
                ny = py + dy
                if not (0 <= ny < h):
                    continue
                for dx in (-1, 0, 1):
                    nx = px + dx
                    if not (0 <= nx < w):
                        continue
                    np_ = ny * w + nx
                    if flat[np_] and not seen[np_]:
                        seen[np_] = 1
                        q.append(np_)
        if size > best:
            best = size
    return best / n

for name in common:
    t = float(name.split(".")[0])
    if t < 20:  # let the world settle; the capture clock starts at tick one
        continue
    ga = np.asarray(Image.open(aa[name]).convert("L"), dtype=np.int16)
    gb = np.asarray(Image.open(fa[name]).convert("L"), dtype=np.int16)
    h, w = ga.shape
    d = np.abs(gb - ga)
    y0, y1, x0, x1 = h // 2 - 160, h // 2 + 160, w // 2 - 160, w // 2 + 160
    win = d[y0:y1, x0:x1]
    ys, xs = np.nonzero(win > 4)
    n = len(ys)
    if not (150 <= n <= 25000):
        rejected += 1
        continue
    bw, bh = xs.max() - xs.min() + 1, ys.max() - ys.min() + 1
    if bw > 280 or bh > 280:  # a blob is a vehicle, not a whole-hull fill
        rejected += 1
        continue
    largest_frac = largest_component_fraction(win > 4, n)
    cx = xs.mean() - (x1 - x0) / 2
    cy = ys.mean() - (y1 - y0) / 2
    samples.append((t, cx, cy, n, largest_frac))
    if first is None:
        first = (name, ga, gb, (int(xs.mean() + x0), int(ys.mean() + y0)))
    last = (name, ga, gb, (int(xs.mean() + x0), int(ys.mean() + y0)))

if len(samples) < 40:
    sys.exit(f"W2 FAILED: only {len(samples)} frames carried a legible blob ({rejected} rejected)")
ts = np.array([s[0] for s in samples])
cxs = np.array([s[1] for s in samples])
cys = np.array([s[2] for s in samples])
fracs = np.array([s[4] for s in samples])
slope, intercept = np.polyfit(ts, cys, 1)  # along-track rides the frame's Y
med_cx = float(np.median(np.abs(cxs)))
resid = float(np.max(np.abs(cys - (slope * ts + intercept))))
med_frac = float(np.median(fracs))
print(f"  frames={len(samples)} rejected={rejected} slope={slope:.3f}px/s intercept={intercept:+.1f}px")
print(f"  |median cx|={med_cx:.1f}px residual={resid:.1f}px blob area median={int(np.median([s[3] for s in samples]))}")
print(f"  single-object fraction median={med_frac:.2f} (the diff is ONE thing, not terrain scatter)")
# geometry, computed HERE from the scenario's own orbits - not copied from
# a comment: each craft carries its own circular speed, so the LOS lead is
# (v_d - v_c); the px scale follows from the TOML's FOV and resolution.
# The whole-flight baseline is deliberate: the residual of the line is the
# chief's sampled-data attitude limit cycle (~12 px, ~75 s period, identical
# in BOTH legs - the digests say the physics is), which averages out over
# two of its periods but bends any shorter window. The witness therefore
# demands the orbit slope over the full flight, a bounded breath, and that
# the moving thing be a single object at every frame.
import math
import tomllib

facts = tomllib.loads(SCENARIO_TOML.read_text())
pos_c = facts["spacecraft"][0]["orbit"]["rv"]["position"]
pos_d = facts["spacecraft"][1]["orbit"]["rv"]["position"]
vel_c = facts["spacecraft"][0]["orbit"]["rv"]["velocity"]
vel_d = facts["spacecraft"][1]["orbit"]["rv"]["velocity"]
rc = math.dist(pos_c, [0.0, 0.0, 0.0])
dr = rc - math.dist(pos_d, [0.0, 0.0, 0.0])
cam = facts["camera"][0]
px_per_m = cam["resolution"][0] / (2 * math.tan(cam["field_of_view"] / 2) * dr)
dv = math.dist(vel_d, [0.0, 0.0, 0.0]) - math.dist(vel_c, [0.0, 0.0, 0.0])
expected_slope = dv * px_per_m  # sign follows the frame's own orientation
print(f"  geometry says: |slope|={abs(expected_slope):.3f}px/s (|v_d|-|v_c|={dv:.4f}m/s * {px_per_m:.1f}px/m)")
ok = (
    abs(abs(slope) - abs(expected_slope)) <= 0.4 * abs(expected_slope)  # the crawl IS the orbit law
    and abs(intercept) <= 12.0      # the deputy starts ON the boresight
    and med_cx <= 15.0              # cross-track stays a bounded wobble, not a drift
    and resid <= 20.0               # along-track breath stays a cycle, not a wander
    and med_frac >= 0.9             # one object does the crawling
)

# evidence strip: first and last blob frames, fleet vs absent, 256 px crop
crops = []
for tag, frame in (("fleet-first", first), ("fleet-last", last)):
    name, ga, gb, (px, py) = frame
    crop = np.asarray(Image.fromarray(np.uint8(gb)).crop(
        (max(px - 128, 0), max(py - 128, 0), min(px + 128, 512), min(py + 128, 512))), dtype=np.int16)
    base = np.asarray(Image.fromarray(np.uint8(ga)).crop(
        (max(px - 128, 0), max(py - 128, 0), min(px + 128, 512), min(py + 128, 512))), dtype=np.int16)
    diff = (np.abs(crop - base) * 3).clip(0, 255)
    strip = np.concatenate([crop, base, diff], axis=1)
    crops.append(strip)
montage = np.vstack(crops).astype(np.uint8)
Image.fromarray(montage).save(out)
print(f"  evidence: {out} (per row: with-deputy | control | diff x3)")
if not ok:
    sys.exit("W2 FAILED: the blob does not obey the formation geometry")
EOF

# --- W3: both birds stamped an honest fit -------------------------------------
echo "=== W3: both vehicles' fits stayed honest ==="
for pair in "$CONSOLE_CHIEF:chief" "$CONSOLE_DEP:deputy"; do
    log="${pair%%:*}"; who="${pair##*:}"
    grep -q "FIT MATCHED" "$log" || {
        echo "FORMATION-EYE FAILED (W3): $who never stamped FIT MATCHED"; tail -6 "$log"; exit 1; }
    echo "  $who: $(grep -m1 'FIT MATCHED' "$log")"
done

# --- W4: the ground station counted BOTH craft during the flight --------------
echo "=== W4: COSMOS counted packets from both vehicles while the flight ran ==="
echo "  interfaces: GS_TCP_INT  before=$CHIEF_RX_BEFORE during=$CHIEF_RX_DURING bytes"
echo "              GS_TCP2_INT before=$DEP_RX_BEFORE during=$DEP_RX_DURING bytes"
((CHIEF_RX_DURING - CHIEF_RX_BEFORE > 10000)) || {
    echo "FORMATION-EYE FAILED (W4): chief interface crossed no traffic"
    echo "  is the station up? scripts/cosmos_stack.sh brings it up (idempotent)"
    exit 1; }
((DEP_RX_DURING - DEP_RX_BEFORE > 10000)) || {
    echo "FORMATION-EYE FAILED (W4): deputy interface crossed no traffic"
    echo "  is GS_TCP2_INT installed/enabled? scripts/cosmos_stack.sh re-installs the gem"
    exit 1; }

echo
echo "FORMATION-EYE CHECK OK"
echo "  the sister vehicle flew (digest-quiet), the eye painted it (pixel-verified),"
echo "  both fits stamped, and the ground station heard both craft the whole way."
