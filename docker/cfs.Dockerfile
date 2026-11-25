# orrery-cfs — the build-and-run environment for the cFS mission layer.
#
# A stranger should be able to clone this repository on a machine with nothing
# but Docker and produce flight software. This image is that promise: everything
# the mission needs to compile that a stock Ubuntu does not have, and nothing it
# does not. Every package below names the step that needs it — the list used to
# carry leftovers from the protobuf-era bridge, and a build image full of
# "someone once needed this" is a list nobody dares prune and nobody can debug
# at 2am. The check is the fresh-clone build (README), not a comment.
#
# The repository is MOUNTED, not copied (`-v "$PWD":/work`), and that is a
# contract rather than a preference: the mission's CMake cache records /work as
# its source directory, so the build must always see the tree at that path.
# Keeping the image independent of the checkout is what lets one image serve the
# mission build, the fleet launch, and the defs regeneration.
#
#   build it:   scripts/image.sh cfs
#   use it:     scripts/mission_build.sh        (host side; calls this image)
#   inside it:  cd /work/cfs_mission && ./build.sh all
#
# Run containers as the host user (`--user $(id -u):$(id -g) -e HOME=/tmp`): the
# launchers boot the mission that way, so a root-owned build tree aborts PSP boot
# with `open: Permission denied`.

FROM ubuntu:24.04

LABEL org.opencontainers.image.title="orrery-cfs" \
      org.opencontainers.image.description="Build and flight-run environment for the orrery cFS mission: native PSP, libzmq, and the toolchain that compiles the mission's EdsLib runtime database" \
      org.opencontainers.image.licenses="MIT"

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    ca-certificates \
    cmake \
    git \
    libzmq3-dev \
    libexpat1-dev \
    liblua5.4-dev \
    pkg-config \
    && rm -rf /var/lib/apt/lists/*
# build-essential  cFE/OSAL/PSP and the EdsLib database are plain C
# ca-certificates  THE FETCH IS OVER TLS. scripts/fetch_cfs.sh runs inside this
#                  image (so a clone plus Docker is all a newcomer needs), and the
#                  Ubuntu 24.04 base ships no CA bundle: without this the clone
#                  dies with "server certificate verification failed. CAfile:
#                  none", which reads like a network or proxy fault and is not one.
#                  The previous image had this package only by accident, through
#                  python3-pip — a dependency chain nobody wrote down, which is how
#                  a pruning that looked safe broke the fresh-clone path.
# cmake          the mission's configure step (mission defs grafted in by -D flags)
# git            scripts/fetch_cfs.sh fetches the pinned core inside this image,
#                and `git ls-files` answers the provenance question the stamp asks
# libzmq3-dev    apps/bsk_link: the Basilisk link is ZeroMQ, in-mission
# libexpat1-dev  EdsLib parses the mission's EDS XML with expat
# liblua5.4-dev  EdsLib's code generator embeds Lua (>= 5.3)
# pkg-config     the CMake finders for the two libraries above
#
# NOT here, whatever older versions of this file claimed:
#   kconfig-frontends  an old comment swore cFS prep failed without it. cFS v7.0.1
#                      configures, compiles and installs from a wiped build tree
#                      without it — evidence, not memory: see the from-nothing
#                      build receipt recorded by scripts/image.sh --list.
#   python3            nothing in this build asks for an interpreter: the container
#                      compiles the EdsLib database with EDSLINK_NO_PYTHON=1 (the
#                      CPython binding is a host artifact, built from the project
#                      venv). It used to ride in via kconfig-frontends and pip.
#   libprotobuf-c-dev, protobuf-c-compiler, m4, python3-pip
#                      survivors of the protobuf-era bridge; nothing in the mission,
#                      in cFS's configure, or in edslink references them.

WORKDIR /work

# A bare `docker run` of this image answers with the two ways in, instead of
# starting a container that does nothing and waits to be noticed.
CMD ["bash", "-c", "echo 'orrery-cfs: build environment for the cFS mission layer.'; echo '  from the host:  scripts/mission_build.sh'; echo '  inside:         cd /work/cfs_mission && ./build.sh all'; echo '  (mount the repository at /work and run as your host UID)'"]
