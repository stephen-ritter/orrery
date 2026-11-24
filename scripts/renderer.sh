#!/usr/bin/env bash
# The rendered eye (CAM), staged: one law, every flight harness.
#
#   scripts/renderer.sh stage <scenario.toml> <container-name>
#
# When a scenario carries [[camera]], the vizard.lock-pinned Vizard joins the
# yard the way every other part of it does - a named container, started by the
# harness, swept by the harness's cleanup trap. The patience build in
# vizard.lock waits for the world's first message as long as the world needs,
# so the renderer simply starts with the yard: no sentinel, no probe, no
# sleep. (Stock Vizard gave up half a second after the bind - a window no
# service-started world can beat, since the first scene message physically
# needs a full scan; that window is what upstream is being asked to remove,
# and what the patience patch removes here.) Two measured laws still run the
# launch: NO port probe - a listener on 5556 can only be ourselves, and a
# probe CONNECTION while the world waits poisons its latch (measured), so
# squatters are refused at the yard door instead; and the player is exec'd
# DIRECTLY behind a hand-started Xvfb, because the xvfb-run wrapper corrupts
# Vizard's launch inside a container (measured - the child dies in the first
# seconds while the wrapper squats the socket). The vizard.lock unpack is
# verified HERE, before any container boots: a missing one otherwise kills the
# flight minutes later at its render wait, and so does an assembly that is not
# the patience build the lock hashes.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUNTIME="${RUNTIME:-docker}"

action="${1:-}"
case "$action" in
    stage)
        scenario="${2:?usage: renderer.sh stage <scenario.toml> <container-name>}"
        name="${3:?usage: renderer.sh stage <scenario.toml> <container-name>}"
        grep -qE '^[[:space:]]*\[\[camera\]\]' "$scenario" || {
            echo "renderer.sh: $(basename "$scenario") carries no [[camera]] - nothing to stage" >&2
            exit 2
        }
        viz_bin="$(sed -n 's/^binary = "\([^"]*\)".*/\1/p' "$REPO/vizard.lock")"
        if [ ! -x "$REPO/$viz_bin" ]; then
            echo "renderer.sh: $viz_bin is not here - run scripts/get_vizard.sh (vizard.lock pins the renderer)" >&2
            exit 2
        fi
        want_sha="$(sed -n 's/^patched_assembly_sha256 = "\([^"]*\)".*/\1/p' "$REPO/vizard.lock")"
        if [ -n "$want_sha" ]; then
            assembly="$(sed -n 's/^patched_assembly = "\([^"]*\)".*/\1/p' "$REPO/vizard.lock")"
            got_sha="$(sha256sum "$REPO/$assembly" 2>/dev/null | cut -d' ' -f1 || true)"
            if [ "$got_sha" != "$want_sha" ]; then
                echo "renderer.sh: the renderer is not the patience build vizard.lock pins" >&2
                echo "             (want ${want_sha:0:12}..., got ${got_sha:-nothing}...)" >&2
                echo "             rebuild it: scripts/build_vizard_patience.sh" >&2
                exit 2
            fi
        fi
        if ! "$RUNTIME" image inspect orrery-viz:dev >/dev/null 2>&1; then
            echo "--- building orrery-viz:dev (renderer runtime) ---"
            "$RUNTIME" build -q -t orrery-viz:dev -f "$REPO/docker/viz.Dockerfile" "$REPO/docker" >/dev/null
        fi
        # squat guard: the renderer EXISTS to open 5556. A listener already
        # here belongs to a dead world's Vizard - it accepts the sim's dial
        # with a deaf scene and stalls the flight in silence (measured).
        if ss -tln | grep -qE ':5556([[:space:]]|$)'; then
            echo "renderer.sh: tcp/5556 already has a listener - sweep the squatting renderer before flying cameras" >&2
            exit 2
        fi
        "$RUNTIME" rm -f "$name" >/dev/null 2>&1 || true
        "$RUNTIME" run -d --name "$name" --network host \
            --user "$(id -u):$(id -g)" -e HOME=/tmp \
            -w /viz/Vizard_Linux \
            -v "$REPO/.vizard/Vizard_Linux:/viz/Vizard_Linux:ro" \
            orrery-viz:dev bash -c '
                Xvfb :97 -screen 0 1280x1024x24 -nolisten tcp >/dev/null 2>&1 &
                export DISPLAY=:97
                for ((t = 0; t < 50; ++t)); do
                    [ -e /tmp/.X11-unix/X97 ] && break
                    sleep 0.2
                done
                if [ ! -e /tmp/.X11-unix/X97 ]; then
                    echo "Xvfb :97 never came up" >&2
                    exit 1
                fi
                exec ./Vizard.x86_64 -args -batchmode -noDisplay tcp://127.0.0.1:5556' >/dev/null
        echo "--- renderer container staged ($name, patient: it waits for the world) ---"
        ;;
    *)
        echo "usage: scripts/renderer.sh stage <scenario.toml> <container-name>" >&2
        exit 2
        ;;
esac
