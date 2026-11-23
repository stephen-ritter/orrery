#!/usr/bin/env bash
# P5 truth wrap: a scenario's fleet, truth on its own ports, the boundary
# measured.
#
# Prereq: scripts/cosmos_stack.sh has loaded gem >= 0.1.2 (plugin carries
# TRUTH_CHIEF/TRUTH_DEPUTY + the truth interfaces). Fleets up a scenario
# (fleet.sh starts each vehicle's truth gateway on its derived port), runs
# sim/truth_exhibits.py through JSON-RPC (fresh truth streams, truth-vs-
# knowledge gap at bias/noise scale, the GNSS opinion vs truth orbit pair
# [KN, 0x08CF], truth watching the commanded detumble, truth targets
# uncommandable), then proves the port-level half of the boundary: bytes
# written TO the truth port are refused and logged by the gateway (the D2
# beacon-refusal idiom, mirrored on the truth side).
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export OPENC3_API_PASSWORD="${OPENC3_API_PASSWORD:-orrery-dev}"
export DURATION="${DURATION:-300}"
cd "$REPO"

if ! docker ps --format '{{.Names}}' | grep -q '^cosmos-openc3-operator-1$'; then
    echo "COSMOS CE not running - run scripts/cosmos_stack.sh first" >&2
    exit 1
fi

SCENARIO="${SCENARIO:-sim/scenarios/twin_ops.toml}"
# the fleet the world runs AND the fleet the client checks: one knob, both
# sides (a script whose SCENARIO only reached the fleet would silently check
# a different spacecraft than it flew).
export ORRERY_SCENARIO="$SCENARIO"
cleanup() { bash scripts/fleet.sh down "$SCENARIO" >/dev/null 2>&1 || true; }
trap cleanup EXIT

cleanup  # never inherit a stale fleet
bash scripts/fleet.sh up "$SCENARIO"
sleep 5
bash scripts/fleet.sh status "$SCENARIO"

echo "--- COSMOS-side truth acceptance (JSON-RPC only) ---"
START_TS="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
rc=0
uv run python -m sim.truth_exhibits || rc=$?

echo "--- truth-port refusal (the boundary, at the wire) ---"
# Write junk at vehicle 0's truth port; the gateway must count, refuse,
# and say so loudly (mirrors gs_tcp's commands-on-the-beacon refusal).
# Derived, never hardcoded: the log belongs to the scenario's first block.
V0NAME="$(SCENARIO="$SCENARIO" uv run python -c 'import os; from sim.fleet import scenario_vehicles; print(scenario_vehicles(os.environ["SCENARIO"])[0].name)')"
uv run python - <<'EOF'
import socket

s = socket.create_connection(("127.0.0.1", 5150), timeout=5)
s.sendall(b"NOPE-ON-THE-TRUTH-PORT")
s.close()
print("wrote 22 stray bytes at :5150")
EOF
sleep 2
# The compose-born gateway logs to its container - read the door that holds
# the evidence (the old launcher's fleet_<name>_truth.log file is gone, and
# a leftover copy must never be grepped as if current). Capture THEN grep:
# grep -q under pipefail kills docker logs with SIGPIPE on first match and
# the pipeline reports failure on a log that clearly matches (the yard's own
# cosmos_stack.sh lesson, re-earned mid-fix - the trap is real).
truth_log="$(docker logs "orrery-truth-$V0NAME" 2>&1 || true)"
if grep -q "REFUSING.*inbound bytes on the truth port" <<<"$truth_log"; then
    echo "gateway refused and logged the inbound write (telemetry-only holds)"
else
    echo "truth gateway did NOT report the stray write:" >&2
    tail -3 <<<"$truth_log" >&2 || true
    rc=1
fi

echo "--- COSMOS log hygiene (since client start) ---"
unknown="$(docker logs --since "$START_TS" cosmos-openc3-operator-1 2>&1 \
    | grep -c "UNKNOWN packet" || true)"
echo "UNKNOWN packet lines (all interfaces): $unknown"
[ "$unknown" -gt 0 ] && rc=1

if [ "$rc" -eq 0 ]; then
    echo "P5 TRUTH RUN OK"
else
    echo "P5 TRUTH RUN FAILED"
fi
exit "$rc"
