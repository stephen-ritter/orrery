# orrery-viz — the renderer: Vizard paints the frames the plant's camera
# modules receive, headless, on the yard's own loopback. Like the other two
# Dockerfiles the yard ships, the repository is MOUNTED, never copied - and
# here so is the renderer itself: docker/viz.Dockerfile carries NO Vizard
# binary (license), only the X and Mesa surface that keeps a Unity player
# alive inside a container. scripts/get_vizard.sh produces the mounted
# unpack from vizard.lock's sha256.
#
#   build:  scripts/image.sh viz      (or let compose build it: cold start)
#   run:    the compose `camera` service, profile-gated; fleet.sh attaches
#           the profile when the scenario carries [[camera]].
#
# Software GL (llvmpipe) is the yard's default: the rendered-pixel probe
# measured 29/31 valid OpNav frames through it, so the knowledge channel
# does not need a GPU. A host with Vulkan can add -force-vulkan later; that
# is a runtime knob, not an image decision.

FROM ubuntu:24.04

# Xvfb for the display Unity insists on having, Mesa for the GL it insists on
# calling, and the handful of X extension libs a Unity IL2CPP player probes
# at init (cursor, randr, xinput, xtest). The wait-for-the-world-then-
# exec-DIRECTLY choreography lives in compose.yml, not here - this image
# only knows how to hold a headless screen. Note what this image does NOT
# need: xvfb-run. Measured, the wrapper corrupts Vizard's launch inside a
# container (child dies in the first seconds; the wrapper outlives it).
# Run Xvfb by hand and exec the player directly, as compose.yml does.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    libgl1 \
    libglu1-mesa \
    libxcursor1 \
    libxi6 \
    libxinerama1 \
    libxrandr2 \
    libxtst6 \
    mesa-libgallium \
    xvfb \
    && rm -rf /var/lib/apt/lists/*

LABEL org.opencontainers.image.title="orrery-viz" \
      org.opencontainers.image.description="Vizard, headless: the scene generator whose frames OpNav navigates from; the licensed binary is mounted from the vizard.lock-verified unpack, never baked in" \
      org.opencontainers.image.licenses="MIT"

WORKDIR /viz

CMD ["bash", "-c", "echo 'orrery-viz: the renderer (Vizard, headless; no binary in the image).'; echo '  mount the vizard.lock-verified unpack at /viz and run as your host UID,'; echo '  then: Xvfb :97 -screen 0 1280x1024x24 & DISPLAY=:97'; echo '    exec ./Vizard_Linux/Vizard.x86_64 -args -batchmode -noDisplay tcp://127.0.0.1:5556'; echo '    (directly - never via xvfb-run: that wrapper kills the child in a container)'"]
