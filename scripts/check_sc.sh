#!/usr/bin/env bash
# sc boot run: the revived sequencer boots, stores nothing, and answers ground.
#
# Same bones as check_clock.sh (mission up, ground client reads, trap cleans
# up) but NO plant: sc's wake is the sensor frame, so with no sim flying the
# sequencer simply waits - which is the honest half of the wake law. The
# probe watches the beacon for sc's own boot story (initialized; the RTS
# load pass reporting every slot empty per the factory-ships-no-sequences
# prune in cfs_mission/build.sh; the power-on autostart REFUSED), then
# commands SC_NOOP and holds SC_HK_TLM's counters to that single receipt.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# The yard must fly what this tree built: refuse a stale mission binary
# before the container starts (scripts/firmware.sh).
bash "$REPO/scripts/firmware.sh" check
RUNTIME="${RUNTIME:-docker}"
BOOT_TIMEOUT="${BOOT_TIMEOUT:-120}"

# One vehicle on the legacy port block; a live fleet squats on the same
# numbers, so start from a quiet sky.
bash "$REPO/scripts/fleet.sh" down sim/scenarios/twin_ops.toml >/dev/null 2>&1 || true

INSTANCE=0
BEACON=$((5050 + 2 * INSTANCE))
UPLINK=$((BEACON + 1))
SENSOR=$((5580 + 20 * INSTANCE))
LINK_CMD=$((SENSOR + 1))

cd "$REPO"

stale="$("$RUNTIME" ps -q --filter ancestor=orrery-cfs:dev || true)"
[ -n "$stale" ] && "$RUNTIME" stop $stale >/dev/null || true

CONTAINER=""
cleanup() {
    [ -n "$CONTAINER" ] && "$RUNTIME" stop "$CONTAINER" >/dev/null 2>&1 || true
}
trap cleanup EXIT

CONTAINER="$("$RUNTIME" run -d --network host --user "$(id -u):$(id -g)" -e HOME=/tmp \
    -e ORRERY_TLM_PORT="$BEACON" -e ORRERY_CMD_PORT="$UPLINK" \
    -e ORRERY_ZMQ_SENSOR_ENDPOINT="tcp://127.0.0.1:$SENSOR" \
    -e ORRERY_ZMQ_CMD_ENDPOINT="tcp://127.0.0.1:$LINK_CMD" \
    -v "$REPO:/work" orrery-cfs:dev \
    bash -c "cd /work/cfs_mission/mission/build/exe/cpu1 && ./container-start")"

echo "--- waiting for gs_tcp + fcs (max ${BOOT_TIMEOUT}s) ---"
SECONDS=0
while [ "$SECONDS" -lt "$BOOT_TIMEOUT" ]; do
    LOGS="$("$RUNTIME" logs "$CONTAINER" 2>&1 || true)"
    if grep -q "gs_tcp downlink :$BEACON uplink :$UPLINK" <<<"$LOGS" && grep -q "FCS App Initialized" <<<"$LOGS"; then
        break
    fi
    if grep -qE "(GS_TCP|FCS|BSK_LINK|SC).*(fail|DEAD)|Could not load file|ES Load Module Error" <<<"$LOGS"; then
        echo "SC BOOT FAILED (app init error)"
        grep -E "GS_TCP|FCS|BSK_LINK|SC|LoadModule|Could not load|EVS Port1.*66/1/ES" <<<"$LOGS" | tail -12
        exit 1
    fi
    sleep 1
done

LOGS="$("$RUNTIME" logs "$CONTAINER" 2>&1 || true)"
if ! grep -q "gs_tcp downlink :$BEACON uplink :$UPLINK" <<<"$LOGS" || ! grep -q "FCS App Initialized" <<<"$LOGS"; then
    echo "SC BOOT FAILED (boot markers not seen within ${BOOT_TIMEOUT}s)"
    grep -E "GS_TCP|FCS|BSK_LINK|SC|Could not load" <<<"$LOGS" | tail -12
    exit 1
fi

# The boot story, from the craft's own console: it happens before any ground
# socket exists (the beacon carries no replay), so the log is the record.
# Three beats: sc initializes; the RTS load pass reports EVERY slot empty
# (the factory-ships-no-sequences prune in cfs_mission/build.sh - compared
# numerically, so the law is "all unloaded" whatever the slot count is);
# and the power-on autostart is REFUSED - the gatekeeper refusing to run a
# sequence nobody wrote, on the record, once.
echo "--- boot story (console) ---"
LOGS="$("$RUNTIME" logs "$CONTAINER" 2>&1 || true)"
grep -q "66/1/SC 9: SC Initialized" <<<"$LOGS" || {
    echo "SC BOOT FAILED: sc never reported initialization"; exit 1; }
# the autostart beat rides the first sch SEND_HK poll (~6 s after boot)
SECONDS=0
while [ "$SECONDS" -lt 30 ]; do
    LOGS="$("$RUNTIME" logs "$CONTAINER" 2>&1 || true)"
    grep -q "66/1/SC 76: Start RTS 001 Rejected" <<<"$LOGS" && break
    sleep 1
done
grep -q "66/1/SC 76: Start RTS 001 Rejected" <<<"$LOGS" || {
    echo "SC BOOT FAILED: the power-on autostart never ran (or never refused)"; exit 1; }
loaded_line="$(grep -oE "SC 21: RTS table files not loaded at initialization = [0-9]+ of [0-9]+" <<<"$LOGS" | tail -1)"
[ -n "$loaded_line" ] || {
    echo "SC BOOT FAILED: no RTS load pass in the console"; exit 1; }
not_loaded="${loaded_line##*= }"; not_loaded="${not_loaded%% of *}"
slots="${loaded_line##* of }"
[ "$not_loaded" = "$slots" ] || {
    echo "SC BOOT FAILED: factory sequences survived the image prune: $loaded_line"; exit 1; }
echo "  $loaded_line"
grep -E "SC (9|76):" <<<"$LOGS" | tail -2

echo "--- sc revival acceptance client ---"
rc=0
uv run python -m sim.sc_probe --instance "$INSTANCE" || rc=$?

echo "--- mission console tail (the sequencer's own words) ---"
"$RUNTIME" logs "$CONTAINER" 2>&1 | grep -E "SC |TIME" | tail -6
if [ "$rc" -ne 0 ]; then exit "$rc"; fi

echo "SC BOOT RUN OK"
