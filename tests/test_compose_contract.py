"""The static compose.yml carries the yard's service contract.

The file is hand-editable (adding a vehicle is an accepted user action),
so these tests pin what must survive any edit: naming the gates sweep on,
the resolver's root-only chain, the sync dependency every engine service
owes, pinned volumes, host networking, and COSMOS staying out.
"""

from __future__ import annotations

import subprocess
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[1]
DOC = yaml.safe_load((REPO / "compose.yml").read_text())
SERVICES = DOC["services"]
ENGINE = {"sync", "truth-chief", "truth-deputy", "world"}


def test_the_shipped_services_are_the_twin_yard() -> None:
    """Services: two inits, chief/deputy pairs, one world, one gated renderer."""
    assert set(SERVICES) == {
        "mission",
        "sync",
        "chief",
        "deputy",
        "truth-chief",
        "truth-deputy",
        "world",
        "camera",
    }


def test_the_cold_start_lives_in_this_document() -> None:
    """Both images build from their Dockerfiles; both inits gate the fleet.

    A cold clone plus `docker compose up` must boot a fleet: the images
    arrive by `build:`, the mission by the mission service, the Python world
    by sync - and nothing that consumes them starts before they finish.
    """
    assert SERVICES["mission"]["build"]["dockerfile"] == "cfs.Dockerfile"
    assert SERVICES["sync"]["build"]["dockerfile"] == "basilisk.Dockerfile"
    mission_cmd = SERVICES["mission"]["command"]
    assert mission_cmd[:2] == ["bash", "-c"] and len(mission_cmd) == 3
    for step in ("scripts/fetch_cfs.sh", "./build.sh"):
        assert step in mission_cmd[2]
    done = {"condition": "service_completed_successfully"}
    for name in ("chief", "deputy"):
        assert SERVICES[name]["depends_on"] == {"mission": done}, name
    for name in ("truth-chief", "truth-deputy", "world"):
        assert SERVICES[name]["depends_on"] == {"sync": done}, name
    # the mission builds INTO this checkout, so it is the host's files it writes
    assert SERVICES["mission"]["user"] != "0:0"


def test_container_names_follow_the_convention_the_yard_sweeps_on() -> None:
    """Everything the yard owns is named orrery-*."""
    for svc in SERVICES.values():
        assert str(svc["container_name"]).startswith("orrery-")
    assert SERVICES["world"]["container_name"] == "orrery-world"


def test_the_resolver_is_the_only_root_and_its_chain_reaches_bash_whole() -> None:
    """Root builds, host UID owns: the chain must be one bash -c script.

    A string command at an ENTRYPOINT-less image is argv, not shell: an
    `a && b` chain would run only `a` and exit 0 regardless.
    """
    sync = SERVICES["sync"]
    assert sync["user"] == "0:0"
    assert "depends_on" not in sync
    cmd = sync["command"]
    assert isinstance(cmd, list) and cmd[:2] == ["bash", "-c"] and len(cmd) == 3
    for step in ("uv sync --frozen", "edslink/build.sh", "chown -R"):
        assert step in cmd[2]
    for name in ENGINE - {"sync"}:
        assert SERVICES[name]["user"] != "0:0", name
        assert SERVICES[name]["depends_on"] == {
            "sync": {"condition": "service_completed_successfully"}
        }, name


def test_ports_arrive_through_the_environment_with_stock_defaults() -> None:
    """fleet.sh exports the derived numbers; the file defaults to twin_ops."""
    env = SERVICES["chief"]["environment"]
    assert env["ORRERY_TLM_PORT"] == "${CHIEF_BEACON:-5050}"
    assert env["ORRERY_ZMQ_SENSOR_ENDPOINT"] == "tcp://127.0.0.1:${CHIEF_SENSOR:-5580}"
    env = SERVICES["deputy"]["environment"]
    assert env["ORRERY_TLM_PORT"] == "${DEPUTY_BEACON:-5052}"
    assert env["ORRERY_ZMQ_CMD_ENDPOINT"] == "tcp://127.0.0.1:${DEPUTY_CMD:-5601}"


def test_the_world_takes_scenario_duration_and_fit_knobs_from_the_environment() -> None:
    """The launcher passes them; the defaults fly the stock twin world."""
    cmd = SERVICES["world"]["command"]
    assert "${ORRERY_SCENARIO:-/work/sim/scenarios/twin_ops.toml}" in cmd
    assert "--duration ${ORRERY_DURATION:-600}" in cmd
    assert "--zmq --pace" in cmd
    assert "${ORRERY_FIT_ARGS:-}" in cmd


def test_the_camera_renderer_is_gated_probes_and_mounts() -> None:
    """The CAM renderer: profile-gated, socket-probed, binary mounted.

    The licensed Vizard binary joins the yard only by the camera profile,
    waits on the sim's REQ socket before dialing (the forge capture test's
    choreography law), and arrives from the vizard.lock unpack - it is
    never baked into an image.
    """
    cam = SERVICES["camera"]
    assert cam["profiles"] == ["camera"], "an ordinary yard must not depend on a renderer"
    done = {"condition": "service_completed_successfully"}
    assert cam["depends_on"] == {"sync": done}
    cmd = cam["command"]
    assert cmd[:2] == ["bash", "-c"] and len(cmd) == 3
    script = cmd[2]
    assert "/dev/tcp/127.0.0.1/" in script, "probe the sim's REQ socket before dialing it"
    assert "exit 1" in script, "the wait must end loudly, never as a silent doomed dial"
    assert "./Vizard.x86_64" in script and "xvfb-run" in script
    assert cam["volumes"][0].endswith(":/viz/Vizard_Linux:ro")
    assert cam["environment"]["ORRERY_VIZ_PORT"] == "${CHIEF_VIZ:-5556}"


def test_volumes_are_declared_and_name_pinned() -> None:
    """Undeclared volumes make compose reject the document.

    Unpinned names get the project prefix, so each project would rebuild
    its own engine venv.
    """
    for vol in ("orrery-bsk-venv", "orrery-bsk-eds"):
        assert DOC["volumes"][vol]["name"] == vol


def test_every_service_shares_the_host_network_like_the_launcher_did() -> None:
    """The fleet speaks on well-known ports; isolation would mute it."""
    for svc in SERVICES.values():
        assert svc["network_mode"] == "host"


def test_cosmos_is_absent_from_the_yard_document() -> None:
    """The station is its own pinned world (cosmos.lock), never a service."""
    text = (REPO / "compose.yml").read_text().lower()
    for svc in SERVICES.values():
        assert "cosmos" not in str(svc).lower()
        assert "openc3" not in str(svc).lower()
    assert "image:" in text  # sanity: services exist to inspect


def test_compose_itself_accepts_the_document() -> None:
    """Parse-and-interpolate with the real parser; skip if docker is absent."""
    subprocess.run(
        ["docker", "compose", "-f", str(REPO / "compose.yml"), "config", "--quiet"],
        check=True,
        capture_output=True,
        timeout=60,
    )
