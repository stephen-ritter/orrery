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


def test_the_camera_renderer_is_gated_starts_and_mounts() -> None:
    """The CAM renderer: profile-gated, patient by default, direct exec, mounted.

    The licensed Vizard binary joins the yard only by the camera profile and
    arrives from the vizard.lock unpack - never baked into an image. It
    starts WITH the yard and nothing else: the patience assembly the lock
    pins (tools/vizard-patience/) waits for the world's first message as
    long as the world needs, so the old sentinel handoff and its freshness
    gate are retired, and their ABSENCE is the contract - a renderer that
    waits for the world can only ever deadlock behind a marker that is an
    EFFECT of streaming. Two measured laws still run the launch: the
    service never touches the port before owning it - the world's REP dials
    THIS service, so a probe connection while it waits poisons the latch
    (measured), and squatters are refused at the yard door (scripts/fleet.sh
    refuses to raise a camera world onto an already-listening 5556) - and
    the player is exec'd DIRECTLY behind a hand-started Xvfb, because the
    xvfb-run wrapper corrupts Vizard's launch inside a container.
    """
    cam = SERVICES["camera"]
    assert cam["profiles"] == ["camera"], "an ordinary yard must not depend on a renderer"
    done = {"condition": "service_completed_successfully"}
    assert cam["depends_on"] == {"sync": done}
    cmd = cam["command"]
    assert cmd[:2] == ["bash", "-c"] and len(cmd) == 3
    script = cmd[2]
    assert "/dev/tcp" not in script, "a probe CONNECTION poisons the REP that dials us"
    assert "/proc/net/tcp" not in script, "the port is OURS to bind; squatters die at the door"
    assert "camera-wait" not in script and "renderer-started" not in script, (
        "the renderer starts with the yard: patience is in the assembly, not the choreography"
    )
    assert "exit 1" in script, "a screen that never comes up must end the service loudly"
    assert "xvfb-run" not in script, "the wrapper corrupts Vizard's launch in a container"
    assert "exec ./Vizard.x86_64" in script, "the player is the service itself"
    assert "tcp://127.0.0.1:" in script, "the zmq URI is colon-form; a slash mutes the renderer"
    assert "-batchmode" in script
    assert "Xvfb :97" in script and "DISPLAY=:97" in script, "the screen, held by hand"
    assert any(v.endswith(":/viz/Vizard_Linux:ro") for v in cam["volumes"])
    assert not any("/work" in v for v in cam["volumes"]), "the renderer reads no world state now"
    assert cam["environment"]["ORRERY_VIZ_PORT"] == "${CHIEF_VIZ:-5556}"
    world_env = SERVICES["world"]["environment"]
    assert "BSKFORGE_VIZ_WAIT_FILE" not in world_env, "the sentinel handoff is retired"
    assert "ORRERY_VIZ_READY_FILE" not in world_env


def test_the_staged_renderer_is_patient_and_lock_verified() -> None:
    """scripts/renderer.sh: the fleet-side staging obeys the same patience law.

    Staging refuses a scenario without [[camera]], refuses an unpack that
    is not there, and refuses an assembly that is not the patience build
    the lock hashes - the check that makes vizard.lock's
    2.4.0+orrery-patience1 label a fact, with the fixing command in the
    refusal. The container body mirrors the compose service: no sentinel
    wait, no probe, direct exec behind a hand-started Xvfb; the squatter
    refusal stays at the yard door.
    """
    script = (REPO / "scripts" / "renderer.sh").read_text()
    assert r"\[\[camera\]\]" in script, "a world without a camera stages no renderer"
    assert 'binary = "' in script and "get_vizard.sh" in script
    assert "patched_assembly_sha256" in script and "build_vizard_patience.sh" in script, (
        "a renderer that is not the patience build stops here, with the fixing command"
    )
    assert "camera-wait" not in script, "the staged renderer starts with the yard"
    body = script.split("bash -c '", 1)[1]
    assert "/dev/tcp" not in body, "no probe: the port is OURS to bind"
    assert "xvfb-run" not in body and "exec ./Vizard.x86_64" in body
    assert "DISPLAY=:97" in body, "the screen, held by hand"
    assert "5556 already has a listener" in script, "squatters are refused at the yard door"


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
