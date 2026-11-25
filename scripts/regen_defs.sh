#!/usr/bin/env bash
# Regenerate COSMOS defs for a scenario's fleet.
#
#   scripts/regen_defs.sh [scenario.toml ...]   default: sim/scenarios/twin_ops.toml
#
# One EDS source, N targets: every vehicle derived from the scenario files
# (sim/fleet.py: block name -> SAT_<NAME>, TRUTH_<NAME>) gets its cmd/tlm
# defs under cosmos_plugin/targets/<TARGET>/cmd_tlm/ - the same generated
# packets, target column swapped (the --target rename proof pins exactly
# that). The C table (gs_tcp rs_mid_table.h) is flight software, not ground
# identity: it is written ONCE and every container compiles it in.
#
# The default is the OPERATIONAL fleet (the twin file is its superset:
# orrery + deputy). Pure-sim plants (the W3 witness) never appear here -
# they have no ground identity because nothing flies their target in COSMOS.
#
# After regenerating: bump GEM_VERSION in scripts/cosmos_stack.sh if the
# plugin's file set changed, rebuild + cli load (same-version loads are a
# silent no-op in CE).
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO"

SCENARIOS=("${@:-sim/scenarios/twin_ops.toml}")
targets="$(uv run python - "${SCENARIOS[@]}" <<'EOF'
import sys

from sim.fleet import scenario_vehicles

seen = []
for path in sys.argv[1:]:
    for v in scenario_vehicles(path):
        for target in (v.cosmos_target, v.truth_target):
            if target not in seen:
                seen.append(target)
print(" ".join(seen))
EOF
)"

for pair in $targets; do
    case "$pair" in
        TRUTH_*)
            # truth side: one generated tlm def, NO cmd def (absence-of-def
            # is the tripwire - the truth side has no actuators)
            echo "--- defs for COSMOS target $pair ---"
            uv run python -m codegen.eds2cosmos \
                --cfs cfs_mission/cFS \
                --mission-defs cfs_mission/mission/orrery_defs/eds \
                --emit-truth --target "$pair" \
                --out-cosmos "cosmos_plugin/targets/$pair/cmd_tlm"
            ;;
        *)
            echo "--- defs for COSMOS target $pair ---"
            uv run python -m codegen.eds2cosmos \
                --cfs cfs_mission/cFS \
                --mission-defs cfs_mission/mission/orrery_defs/eds \
                --target "$pair" \
                --out-cosmos "cosmos_plugin/targets/$pair/cmd_tlm"
            # C table once (SAT_CHIEF is the canonical flight target);
            # it must stay byte-identical, so only it writes the header.
            if [ "$pair" = "SAT_CHIEF" ]; then
                uv run python -m codegen.eds2cosmos \
                    --cfs cfs_mission/cFS \
                    --mission-defs cfs_mission/mission/orrery_defs/eds \
                    --out-c cfs_mission/mission/apps/gs_tcp/fsw/src/rs_mid_table.h \
                    --out-cmd-c cfs_mission/mission/apps/sps/fsw/src/sps_cmd_table.h
            fi
            ;;
    esac
done

echo "regen complete: $targets"
# The two C headers are flight software: if they changed, the binaries in
# mission/build are stale (a beacon that never streams a new packet, or a
# dictionary missing a new command, is a full acceptance run lost to a build
# order). Say it in the script's own voice.
if git -C "$REPO" status --short \
        cfs_mission/mission/apps/gs_tcp/fsw/src/rs_mid_table.h \
        cfs_mission/mission/apps/sps/fsw/src/sps_cmd_table.h 2>/dev/null | grep -q .; then
    echo "NOTE: C tables changed on disk - REBUILD THE MISSION before flying:"
    echo "      docker run --rm -v \"\$PWD:/work\" --user \"\$(id -u):\$(id -g)\" -e HOME=/tmp \\"
    echo "        orrery-cfs:dev bash -c 'cd /work && cfs_mission/build.sh all'"
fi
