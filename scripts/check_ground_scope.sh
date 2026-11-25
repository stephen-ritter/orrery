#!/usr/bin/env bash
# The station is cut from the same TOML: no ghost buttons, flown twice.
#
#   strict    the orrery plugin is rebuilt with SIM_CHIEF's dictionary scoped
#             to chief.toml's wake list (no eye declared) and loaded. The
#             station then offers: no CAM command and no CAM_HK for SAT_CHIEF
#             - while SAT_DEPUTY, scoped by nothing, still offers the whole
#             catalog (the contrast lives inside one plugin, one station),
#             and the core's own commands are present everywhere: the core
#             always boots, so nobody can scope them away.
#   restored  scripts/cosmos_stack.sh brings the catalog plugin back and the
#             CAM commands return for chief - the yard always ends with the
#             station everyone's checks expect.
#
# The offline half of this rung (tests/test_ground_scope.py) proves the
# declared-eye scope reproduces the shipped bytes exactly; this script is
# the operator's half: the buttons themselves. Plugin versions are chosen
# BELOW the shipped 0.1.10, so a station restart can only ever wake the
# catalog, never the strict variant.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUNTIME="${RUNTIME:-docker}"
COSMOS_HOME="${COSMOS_HOME:-$HOME/Projects/cosmos}"
export OPENC3_API_PASSWORD="${OPENC3_API_PASSWORD:-orrery-dev}"
STRICT_VERSION=0.0.99
SCRATCH="$REPO/.scratch/ground_scope"
cd "$REPO"
mkdir -p /tmp/opencode/ground_scope

run_cli() {  # same fake-TTY dance cosmos_stack.sh uses
    ( cd "$REPO" && script -qc "$COSMOS_HOME/openc3.sh cli $*" /dev/null )
}

restore() {
    echo "=== restoring the catalog plugin (scripts/cosmos_stack.sh) ==="
    bash "$REPO/scripts/cosmos_stack.sh" >/tmp/opencode/ground_scope/restore.log 2>&1 || {
        echo "cosmos_stack restore FAILED - read /tmp/opencode/ground_scope/restore.log" >&2
        return 1
    }
}
trap 'mkdir -p /tmp/opencode/ground_scope; restore' EXIT

mkdir -p "$SCRATCH"
rm -rf "$SCRATCH/plugin" "$SCRATCH/scoped"

echo "=== [strict] the station dictionary, cut to what the wake list can honor ==="
uv run python -m codegen.eds2cosmos \
    --cfs cfs_mission/cFS --mission-defs cfs_mission/mission/orrery_defs/eds \
    --scope-scenario sim/scenarios/chief.toml --scope-vehicle chief \
    --out-cosmos "$SCRATCH/scoped" 2>/dev/null
grep -q "COMMAND SAT_CHIEF CAM_CMD " "$SCRATCH/scoped/sat_chief_cmd_def.txt" && {
    echo "[strict] the scoped dictionary still carries CAM commands offline?" >&2
    exit 1
}
cp -r cosmos_plugin "$SCRATCH/plugin"
rm -f "$SCRATCH/plugin"/*.gem  # a stale gem here bundles as DATA and re-extracts nested
cp "$SCRATCH/scoped/sat_chief_cmd_def.txt" "$SCRATCH/scoped/sat_chief_tlm_def.txt" \
    "$SCRATCH/plugin/targets/SAT_CHIEF/cmd_tlm/"
docker run --rm -v "$SCRATCH/plugin:/p" --user "$(id -u):$(id -g)" ruby:3.4-slim \
    sh -c "cd /p && VERSION=$STRICT_VERSION gem build openc3-cosmos-orrery.gemspec \
           --output=/p/openc3-cosmos-orrery-$STRICT_VERSION.gem" >/dev/null

LOAD_OUT="$(run_cli load "/openc3/local/.scratch/ground_scope/plugin/openc3-cosmos-orrery-$STRICT_VERSION.gem" DEFAULT 2>&1)"
if grep -qi "No version change detected\|backtrace" <<<"$LOAD_OUT"; then
    echo "[strict] the station did not really load the scoped plugin:" >&2
    tail -5 <<<"$LOAD_OUT" >&2
    exit 1
fi

echo "--- waiting for the station dictionary to speak the scope ---"
for _ in $(seq 1 90); do
    if uv run python - <<'PY' >/tmp/opencode/ground_scope/dict.json 2>/dev/null
import json
import sys

from sim.cosmos_api import login, rpc

tok = login()
chief_cmds = [c["packet_name"] for c in rpc(tok, "get_all_commands", "SAT_CHIEF")]
chief_tlms = [t["packet_name"] for t in rpc(tok, "get_all_telemetry", "SAT_CHIEF")]
dep_cmds = [c["packet_name"] for c in rpc(tok, "get_all_commands", "SAT_DEPUTY")]
report = {
    "chief_has_cam_cmd": "CAM_CMD" in chief_cmds,
    "chief_has_cam_hk": "CAM_HK_TLM" in chief_tlms,
    "chief_has_core_cmds": any(c.startswith("CFE_ES") for c in chief_cmds),
    "deputy_has_cam_cmd": "CAM_CMD" in dep_cmds,
}
print(json.dumps(report))
# ready = the scope is visible AND the core is still there (not a half-built dict)
raise SystemExit(0 if not report["chief_has_cam_cmd"] and report["chief_has_core_cmds"] else 1)
PY
    then
        break
    fi
    sleep 2
done
cat /tmp/opencode/ground_scope/dict.json
uv run python - <<'PY'
import json

r = json.load(open("/tmp/opencode/ground_scope/dict.json"))
assert not r["chief_has_cam_cmd"], "the station still offers CAM_CMD to a vehicle that never wakes cam"
assert not r["chief_has_cam_hk"], "the station still offers CAM_HK to a vehicle that never wakes cam"
assert r["chief_has_core_cmds"], "the core's commands scoped away - the core always boots"
assert r["deputy_has_cam_cmd"], "scoping chief must not reach into a different vehicle's dictionary"
PY

echo "=== [restored] the catalog returns; the buttons come back ==="
restore
trap - EXIT
for _ in $(seq 1 90); do
    if uv run python - <<'PY' >/tmp/opencode/ground_scope/restored.json 2>/dev/null
import json

from sim.cosmos_api import login, rpc

tok = login()
chief_cmds = [c["packet_name"] for c in rpc(tok, "get_all_commands", "SAT_CHIEF")]
ok = "CAM_CMD" in chief_cmds
print(json.dumps({"chief_cam_cmd_back": ok}))
raise SystemExit(0 if ok else 1)
PY
    then
        break
    fi
    sleep 2
done
cat /tmp/opencode/ground_scope/restored.json
grep -q '"chief_cam_cmd_back": true' /tmp/opencode/ground_scope/restored.json || {
    echo "[restored] the catalog plugin did not bring CAM_CMD back" >&2
    exit 1
}

echo "GROUND SCOPE CHECK OK: the station offers each vehicle exactly what its scenario wakes - and takes it back just as cleanly"
