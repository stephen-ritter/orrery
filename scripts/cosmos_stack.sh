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
# and every API the gates use answered correctly the whole time. The flight side
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
# 0.1.5: CAM v2a - the command surface (CAM_SET_CONFIG on its own interface).
# 0.1.8: EDS strings reach the ground as typed STRING - CFDP_DOWN paths can be
#        typed into the Command Installer; EOT filenames read as text.
# 0.1.9: CAM_CMD FC 1 ManifestCmd - the store inventory has a ground name.
GEM_VERSION=0.1.9
rm -f "$REPO/cosmos_plugin"/openc3-cosmos-orrery-*.gem \
      # a stale gem in here gets bundled as DATA and re-extracts as a nested plugin on load
docker run --rm -v "$REPO/cosmos_plugin:/p" --user "$(id -u):$(id -g)" ruby:3.4-slim \
    sh -c "cd /p && VERSION=$GEM_VERSION gem build openc3-cosmos-orrery.gemspec \
           --output=/p/openc3-cosmos-orrery-$GEM_VERSION.gem" >/dev/null

# cwd ($REPO) is mounted as /openc3/local by `openc3.sh cli`
# Unload our exact registry entry first; anything else loaded (tools, demo)
# stays. First boot on a fresh station has nothing to unload, hence `|| true`.
run_cli unload "openc3-cosmos-orrery-$GEM_VERSION.gem__0" DEFAULT >/dev/null 2>&1 || true
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

echo "COSMOS stack up, orrery plugin loaded"
