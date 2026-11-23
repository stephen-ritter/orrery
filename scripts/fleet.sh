#!/usr/bin/env bash
# Fleet launcher: the scenario file IS the fleet, compose.yml is its engine.
#
#   scripts/fleet.sh up <scenario.toml> [--port-base N]   launch (detached)
#   scripts/fleet.sh down [scenario.toml]                 stop (default: all)
#   scripts/fleet.sh status [scenario.toml]               containers + world
#
# Ports come from sim/fleet.py (the single authority) and reach compose.yml
# through the environment. compose.yml ships services for chief and deputy;
# a third vehicle means editing that file - see docs/adding_a_spacecraft.md.
#
# DURATION=inf (env) runs the world perpetually: it turns until `fleet.sh
# down` (SIGTERM) ends it at a tick boundary and prints its digest-so-far.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUNTIME="${RUNTIME:-docker}"
COMPOSE="${COMPOSE:-docker compose -f $REPO/compose.yml -p orrery}"
STATE_DIR="${STATE_DIR:-/tmp/opencode}"
DURATION="${DURATION:-600}"
FLEET=()
STEM=""
cd "$REPO"

parse_args() {
    SCENARIO=""; PORT_BASE=0
    SCENARIO="${1:-}"; shift || true
    while [ $# -gt 0 ]; do
        case "$1" in
            --port-base) PORT_BASE="${2:?--port-base needs a number}"; shift 2 ;;
            *) echo "fleet.sh: unknown option $1" >&2; exit 2 ;;
        esac
    done
}

scenario_facts() {
    # FLEET lines: "name instance beacon uplink sensor link_cmd"
    mapfile -t FLEET < <(uv run python -c "
from sim.fleet import scenario_vehicles
for v in scenario_vehicles('$SCENARIO', port_base=$PORT_BASE):
    p = v.ports
    print(v.name, v.instance, p.beacon, p.uplink, p.sensor, p.link_cmd)
")
    [ "${#FLEET[@]}" -ge 1 ] || { echo "fleet.sh: $SCENARIO has no [[spacecraft]] blocks" >&2; exit 2; }
    STEM="$(basename "$SCENARIO" .toml)"

    local fact name instance beacon uplink sensor cmd var
    local -a want=()
    for fact in "${FLEET[@]}"; do
        read -r name instance beacon uplink sensor cmd <<<"$fact"
        var="$(tr '[:lower:]' '[:upper:]' <<<"$name")"
        export "${var}_BEACON=$beacon" "${var}_UPLINK=$uplink" \
            "${var}_SENSOR=$sensor" "${var}_CMD=$cmd"
        want+=("$name")
    done
    for name in "${want[@]}"; do
        case " chief deputy " in
            *" $name "*) ;;
            *) echo "fleet.sh: vehicle '$name' has no services in compose.yml;" \
                   "add them first (docs/adding_a_spacecraft.md)" >&2; exit 2 ;;
        esac
    done

    # the scenario must be visible inside the world container
    local abs; abs="$(realpath "$SCENARIO")"
    case "$abs" in
        "$REPO"/*) export ORRERY_SCENARIO="/work/${abs#"$REPO"/}" ;;
        "$STATE_DIR"/*) export ORRERY_SCENARIO="/state/${abs#"$STATE_DIR"/}" ;;
        *) echo "fleet.sh: $SCENARIO is neither under $REPO nor $STATE_DIR;" \
               "the world container cannot see it" >&2; exit 2 ;;
    esac
}

export_env() {
    export ORRERY_REPO="$REPO" ORRERY_STATE="$STATE_DIR" ORRERY_DURATION="$DURATION"
    export ORRERY_UID="$(id -u)" ORRERY_GID="$(id -g)"
    local _n _i _b _u sensor cmd extra=""
    if [ "${#FLEET[@]}" -eq 1 ]; then
        read -r _n _i _b _u sensor cmd <<<"${FLEET[0]}"
        [ "$_i" != 0 ] && extra="--instance $_i"
    elif [ "$PORT_BASE" != 0 ]; then
        extra="--port-base $PORT_BASE"
    fi
    export ORRERY_SIM_EXTRA="$extra"
}

services_of() {
    local fact name _x
    local -a s=("sync")
    for fact in "${FLEET[@]}"; do
        read -r name _x <<<"$fact"
        s+=("$name" "truth-$name")
    done
    s+=("world")
    printf '%s\n' "${s[@]}"
}

scenario_up() {
    scenario_facts
    export_env
    fleet_down

    # fit knobs: a world that lies about its fit prints the lie here and
    # forwards it to the world verbatim (see scripts/check_fit_handshake.sh)
    local fit_args="" part
    for part in ${FIT_DECLARE:-}; do fit_args+=" --fit-declare $part"; done
    for part in ${FIT_HIDE:-}; do fit_args+=" --fit-hide $part"; done
    [ -n "${FIT_DIGEST:-}" ] && fit_args+=" --fit-digest $FIT_DIGEST"
    [ -n "$fit_args" ] && echo "[${STEM}] FIT INJECTION declare='${FIT_DECLARE:-}' hide='${FIT_HIDE:-}' digest='${FIT_DIGEST:-}'"
    export ORRERY_FIT_ARGS="$fit_args"

    # launch-era host logs must not survive as evidence (containers log to
    # their containers now)
    rm -f "$STATE_DIR"/fleet_*_truth.log "$STATE_DIR/fleet_${STEM}_sim.log"

    # a cold machine builds both images and the mission inside `up` (the
    # compose init services); the boot wait must outlast a real build
    local wait_max=120
    if ! "$RUNTIME" image inspect orrery-cfs:dev >/dev/null 2>&1 ||
            [ ! -x "cfs_mission/mission/build/exe/cpu1/container-start" ]; then
        echo "[${STEM}] cold yard: compose is building the images and the mission (once; minutes)"
        wait_max=2400
    fi
    $COMPOSE up -d --remove-orphans $(services_of) >/dev/null

    local fact name instance beacon uplink _s _l
    for fact in "${FLEET[@]}"; do
        read -r name instance beacon uplink _s _l <<<"$fact"
        echo "--- [$name] instance $instance: beacon :$beacon uplink :$uplink ---"
        local booted=0 _
        for _ in $(seq 1 "$wait_max"); do
            # capture-then-grep: pipefail + grep -q + SIGPIPE (cosmos_stack.sh)
            if grep -q "gs_tcp downlink :$beacon uplink :$uplink" \
                    <<<"$($RUNTIME logs "orrery-$name" 2>&1 || true)"; then
                booted=1
                break
            fi
            sleep 1
        done
        if [ "$booted" -ne 1 ]; then
            echo "[$name] FAILED to boot on its own ports (see $RUNTIME logs orrery-$name)" >&2
            return 1
        fi
    done
    # the yard flew a binary the mission init built from THIS tree: verify the
    # stamp now, not before the build - compose is the build now. A refusal
    # here has already left nothing stale booted (init rebuilt what it found).
    bash "$REPO/scripts/firmware.sh" check
    echo "[${STEM}] world up (${#FLEET[@]} vehicles, one world, doc compose.yml)"
    echo "         world:  $RUNTIME logs orrery-world"
}

legacy_pids_down() {
    # pre-compose launchers ran the world and gateways as host processes;
    # their pidfiles still get taken down politely (world first, to its
    # completion: a perpetual run prints digests on SIGTERM)
    local pidfile _
    for pidfile in "$STATE_DIR"/fleet_*_sim.pid "$STATE_DIR"/fleet_*_truth.pid; do
        [ -f "$pidfile" ] || continue
        kill "$(cat "$pidfile")" >/dev/null 2>&1 || true
        for _ in $(seq 1 15); do
            kill -0 "$(cat "$pidfile")" 2>/dev/null || break
            sleep 1
        done
        kill "$(cat "$pidfile")" >/dev/null 2>&1 || true
        rm -f "$pidfile"
    done
}

fleet_down() {
    legacy_pids_down
    $COMPOSE down -t 35 --remove-orphans >/dev/null 2>&1 || true
    local name
    while read -r name; do
        [ -n "$name" ] || continue
        "$RUNTIME" rm -f "$name" >/dev/null 2>&1 || true
    done < <("$RUNTIME" ps -a --filter "name=orrery-" --format '{{.Names}}' 2>/dev/null || true)
    rm -f "$STATE_DIR"/fleet_*.compose.json 2>/dev/null || true
    [ -n "${SCENARIO:-}" ] && echo "[${STEM:-?}] down" || true
}

scenario_status() {
    scenario_facts
    local world="dead" fact name _x
    world="$($RUNTIME ps --filter "name=orrery-world$" --format '{{.Status}}' 2>/dev/null || true)"
    [ -n "$world" ] || world="dead"
    echo "== $SCENARIO: ${#FLEET[@]} vehicles, world: $world =="
    for fact in "${FLEET[@]}"; do
        read -r name _x <<<"$fact"
        printf '%-12s %-28s\n' "orrery-$name" \
            "$($RUNTIME ps --filter "name=orrery-$name$" --format '{{.Status}}' 2>/dev/null || true)"
    done
    $RUNTIME logs --tail 1 orrery-world 2>&1 | sed 's/^/    /' || true
}

command="${1:-}"; shift || true
case "$command" in
    up)
        [ $# -ge 1 ] || { echo "usage: fleet.sh up <scenario.toml> [--port-base N]" >&2; exit 2; }
        # no image or mission builds happen here any more: compose.yml
        # carries them (image `build:` + the mission init service)
        # the yard flies what this tree built (scripts/firmware.sh); `down`
        # and `status` stay available for stale fleets without that check
        bash "$REPO/scripts/firmware.sh" check
        parse_args "$@"
        scenario_up
        ;;
    down)
        parse_args "$@"
        fleet_down
        ;;
    status)
        if [ $# -ge 1 ] && [ -n "${1:-}" ]; then
            parse_args "$@"
            scenario_status
        else
            mapfile -t names < <("$RUNTIME" ps -a --filter "name=orrery-" --format '{{.Names}}' |
                sed 's|^orrery-||')
            [ "${#names[@]}" -ge 1 ] || { echo "no missions"; exit 0; }
            for name in "${names[@]}"; do
                printf '%-18s %-28s\n' "orrery-$name" \
                    "$($RUNTIME ps --filter "name=orrery-$name$" --format '{{.Status}}' 2>/dev/null || true)"
            done
        fi
        ;;
    *)
        echo "usage: fleet.sh {up <scenario.toml> [--port-base N] | down [scenario.toml] | status [scenario.toml]}" >&2
        exit 2
        ;;
esac
