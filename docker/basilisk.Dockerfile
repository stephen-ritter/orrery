# orrery-bsk — the Basilisk engine: the lockstep world that mission
# containers gate on, and where trajectory digests are born. It is the
# yard's determinism boundary: same tree, same digests, or the baselines
# are anecdotes.
#
# Like docker/cfs.Dockerfile, the repository is MOUNTED at /work, never
# copied; dependencies resolve at first run via `uv sync --frozen`.
#
#   build:  scripts/image.sh basilisk
#   run:    docker run --rm --network host --user $(id -u):$(id -g) \
#             -e HOME=/tmp -e OMP_NUM_THREADS=1 -v "$PWD":/work \
#             -v orrery-bsk-venv:/opt/engine -v orrery-bsk-eds:/opt/edslink \
#             orrery-bsk:dev bash -c 'uv sync --frozen && \
#               /opt/engine/venv/bin/python -m sim.forge_scenario --zmq --pace'
#
# OMP_NUM_THREADS=1 is the runner's job: BLAS thread count changes float
# summation order, and digests are float.

# Same base as docker/cfs.Dockerfile on purpose: gcc-14 (python:3.12-slim's)
# fails EdsLib's -Werror build; digests deserve one toolchain judgment.
FROM ubuntu:24.04

# EdsLib's build deps (see edslink/build.sh `check`) + python3-dev headers:
# the container builds its OWN EdsLib binding; CPython extensions are not
# portable between interpreters. git: uv resolves the forge from its git URL.
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    ca-certificates \
    cmake \
    curl \
    git \
    libexpat1-dev \
    liblua5.4-dev \
    pkg-config \
    python3 \
    python3-dev \
    && rm -rf /var/lib/apt/lists/*

LABEL org.opencontainers.image.title="orrery-bsk" \
      org.opencontainers.image.description="The Basilisk simulation engine: one lockstep world whose stepped ticks gate on acks from the flight containers; where trajectory digests are born" \
      org.opencontainers.image.licenses="MIT"

# uv at build time, venv at first run: the build context stays blind to the
# checkout. Curl-pipe because Ubuntu's python is PEP 668 externally managed.
RUN curl -LsSf https://astral.sh/uv/install.sh \
    | env UV_UNMANAGED_INSTALL=/usr/local/bin sh

# The venv volume mounts at /opt/engine, NOT at the venv itself: uv deletes
# and recreates a venv whose interpreter moved, and nothing can delete a
# mount point. The venv must be a directory inside the mount.
ENV UV_PROJECT_ENVIRONMENT=/opt/engine/venv \
    UV_LINK_MODE=copy

WORKDIR /work

CMD ["bash", "-c", "echo 'orrery-bsk: the Basilisk engine (the world that mission containers gate on).'; echo '  mount the repository at /work, the venv volume at /opt/engine and the'; echo '  edslink volume at /opt/edslink, run as your host UID, e.g.:'; echo '  uv sync --frozen && /opt/engine/venv/bin/python -m sim.forge_scenario --zmq --pace'"]
