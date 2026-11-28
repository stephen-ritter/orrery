#!/usr/bin/env bash
# Bring up COSMOS CE (the sanctioned ~/Projects/cosmos/openc3.sh flow) and
# install the orrery plugin gem. Idempotent: safe to re-run; re-run after
# regenerating cmd/tlm defs to pick up new packet layouts.
#
#   COSMOS_HOME=~/Projects/cosmos  openc3 checkout (has openc3.sh + .env)
#   OPENC3_API_PASSWORD            api password, default: orrery-dev
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COSMOS_HOME="${COSMOS_HOME:-$HOME/Projects/cosmos}"
export OPENC3_API_PASSWORD="${OPENC3_API_PASSWORD:-orrery-dev}"

# The station must BE the pinned station (cosmos.lock). This deployment ran a
# v6.4-era checkout against 7.5-era images for months - openc3.sh sources the
# checkout's .env, so an old checkout and `:latest` images drift apart silently,
# and every API the checks use answered correctly the whole time. The flight side
# has never been allowed that silence; the ground side is not now.
# Absent checkout -> this clones it (a fresh machine needs one command here too).
bash "$REPO/scripts/get_cosmos.sh" || exit 1

run_cli() {
    # openc3.sh cli wants a TTY; `script -qc` fakes one from non-tty contexts
    ( cd "$REPO" && script -qc "$COSMOS_HOME/openc3.sh cli $*" /dev/null )
}

echo "--- starting COSMOS CE stack ---"
( cd "$COSMOS_HOME" && ./openc3.sh run )

echo "--- waiting for bootstrap (init container) ---"
# NOTE: `openc3.sh run` RECREATES the init container, so bootstrap (~2 min of
# gem seeding) restarts on every script run; waiting is real work, not a no-op.
booted=0
for i in $(seq 1 90); do
    # capture-then-grep: `docker logs | grep -q` under pipefail can NEVER
    # succeed once the log grows past grep's read buffer - grep -q exits at
    # the match, docker logs takes SIGPIPE (141), pipefail poisons the pipe.
    # The hang this caused was indistinguishable from a slow bootstrap.
    ILOGS="$(docker logs cosmos-openc3-cosmos-init-1 2>&1 || true)"
    if grep -q "all done" <<<"$ILOGS"; then
        booted=1
        break
    fi
    [ $((i % 6)) -eq 0 ] && echo "  still bootstrapping ($((i * 5))s)..."
    sleep 5
done
if [ "$booted" -ne 1 ]; then
    echo "init never reported 'all done' — log tail:" >&2
    docker logs cosmos-openc3-cosmos-init-1 2>&1 | tail -10 >&2
    exit 1
fi

echo "--- API password + plugin gem ---"
# setpassword only SEEDS the initial password ("Password has already been
# set" thereafter - changing it is a web-interface act). Stations bootstrapped
# before 2026-11-23 were seeded the previous default; they keep it until re-keyed
# in the web UI or re-bootstrapped - export OPENC3_API_PASSWORD to match
# until then.
run_cli setpassword >/dev/null 2>&1 || true

# Every load here is a FORCED fresh load: unload first (a not-loaded plugin is
# not an error), then load. Same-version `cli load` otherwise prints "No version
# change detected" and skips plugin.txt reprocessing - and the 2026-11 switch to
# the pinned v7.5.1 proved that verdict can lie: v7 moved plugin storage (new
# bucket + gems volumes) while the old Redis model survived saying "0.1.3 is
# loaded". The station looked healthy - targets, interface, API answers - and
# telemetry froze when a microservice tried to read target files that were never
# extracted. A script that rebuilds the gem every run must not accept a cached
# load of a gem it just made; version bumps stay free to skip nothing now.
# Bump GEM_VERSION whenever plugin.txt changes anyway - it names what is flying.
# 0.1.5: the camera command surface (CAM_SET_CONFIG on its own interface).
# 0.1.8: EDS strings reach the ground as typed STRING - CFDP_DOWN paths can be
#        typed into the Command Installer; EOT filenames read as text.
# 0.1.9: CAM_CMD FC 1 ManifestCmd - the store inventory has a ground name.
#        (0.1.13 retired it: the store publishes its own inventory.)
# 0.1.10: target descriptions tell the mission, not the working plan; the
#         screens and defs a reader opens now read as the mission speaks.
# 0.1.14: the prebaked purge - thirteen silent modules left the dictionary
#         (63 -> 32 command packets, 298 -> 133 SPS rows). A station must not
#         offer commands for apps that no longer boot.
# 0.1.16: the truth frames grow the fuel ledger (TruthState 18 -> 22 f64:
#         remaining/spent kg, starved/throttled ticks). Only TRUTH_* targets
#         change: fuel is the world's book, never a beacon packet.
# 0.1.17: OBC_HK grows MISSION_CLOCK_S - the mission clock as CFE_TIME reads
#         it in a sensor frame's wake. The clock bench measures against it;
#         a station decoding SAT_* telemetry sees the same clock the craft
#         decides on.
# 0.1.18: the sc revival (sequence arc): SC_CMD/SC_SEND_HK/SC_WAKEUP enter
#         the command dictionary and SC_HK_TLM the frames - the ground now
#         commands and reads the stored-command service. SC's WAKEUP entry
#         is the nominal TC seat; the firmware wake is the sensor frame
#         (orrery_defs/config/sc_msgid_values.h), which the ground never
#         sends.
GEM_VERSION=0.1.18
rm -f "$REPO/cosmos_plugin"/openc3-cosmos-orrery-*.gem \
      # a stale gem in here gets bundled as DATA and re-extracts as a nested plugin on load
docker run --rm -v "$REPO/cosmos_plugin:/p" --user "$(id -u):$(id -g)" ruby:3.4-slim \
    sh -c "cd /p && VERSION=$GEM_VERSION gem build openc3-cosmos-orrery.gemspec \
           --output=/p/openc3-cosmos-orrery-$GEM_VERSION.gem" >/dev/null

# cwd ($REPO) is mounted as /openc3/local by `openc3.sh cli`
# Unload by the NAME THE REGISTRY ACTUALLY CARRIES, never by the version about
# to be installed: entries are versioned (openc3-cosmos-orrery-<ver>.gem__<n>),
# so unloading the NEW version's name before a bump is a silent no-op - the old
# entry survives, the load half-installs beside it, and the replace sweep then
# kills the freshly started microservices (bit the v0.1.13 bring-up: a Ruby
# backtrace, a listed entry, and no scope behind it). Anything else loaded
# (tools, demo) stays; first boot on a fresh station has nothing to unload.
for ENTRY in $(run_cli list DEFAULT 2>/dev/null \
        | grep -oE 'openc3-cosmos-orrery-[^ ]+\.gem__[0-9]+' || true); do
    echo "--- unloading $ENTRY ---"
    run_cli unload "$ENTRY" DEFAULT >/dev/null 2>&1 || true
done
LOAD_OUT="$(run_cli load "/openc3/local/cosmos_plugin/openc3-cosmos-orrery-$GEM_VERSION.gem" DEFAULT 2>&1)"
LOAD_OUT="${LOAD_OUT//$'\r'/}"
printf '%s\n' "$LOAD_OUT" | grep -vE '^ ?\{|Container' | tail -2
# After a forced unload there is nothing for the station to call "no change".
# If it says so anyway, its registry is lying (the v7.5.1 switch proved a lying
# registry freezes telemetry behind green screens) - refuse to call that up.
if grep -q "No version change detected" <<<"$LOAD_OUT"; then
    echo "cosmos_stack: load right after a forced unload said 'No version change detected' - the plugin registry does not match storage; refusing to call that healthy" >&2
    exit 1
fi
# And a load may die outright instead of lying: the 2026-11-24 v0.1.5 bring-up
# crashed mid-load (Ruby backtrace ending at openc3cli:1133), the registry kept
# serving the OLD gem, and this script still printed "loaded" - because
# `script -qc` masks the child's exit code, set -e never saw anything. A
# top-level backtrace IS the crash signature; trust it, not the echo.
if grep -qE "in '<main>'" <<<"$LOAD_OUT"; then
    echo "cosmos_stack: plugin load CRASHED (backtrace in output) - the station is still serving the previous gem; re-run this script, and if it repeats, read the trace" >&2
    exit 1
fi

echo "--- tripwire: the registry must SERVE the defs, not merely list them ---"
# A green word from a wrapper is a claim; the registry is the proof (2026-11-24).
# Extended 2026-11-27 to the version-bump race: a half-installed entry LISTED
# fine and SERVED nothing - targets exist only once the gem's microservices have
# extracted the target files. So the acceptance question is not "did the load
# print", it is "does get_target_names answer SAT_CHIEF", and it polls: target
# extraction after a load is the station's own asynchronous work.
SERVED=0
for i in $(seq 1 36); do
    if ( cd "$REPO" && uv run python -c \
            'import sys; from sim.cosmos_api import login, rpc; sys.exit(0 if "SAT_CHIEF" in rpc(login(), "get_target_names") else 1)' ) \
            >/dev/null 2>&1; then
        SERVED=1
        break
    fi
    [ $((i % 6)) -eq 0 ] && echo "  SAT_CHIEF not serving yet ($((i * 5))s)..."
    sleep 5
done
if [ "$SERVED" -ne 1 ]; then
    echo "cosmos_stack: the station lists the gem but serves no SAT_CHIEF - a half-installed registry." >&2
    echo "  It must not be trusted to fly: unload the entry 'cli list' actually names and load again." >&2
    exit 1
fi

echo "COSMOS stack up, orrery plugin loaded and SERVING"
