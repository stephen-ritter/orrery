"""Parts registry tests: the catalog answers for the circuit.

Two directions of trust:

* the registry may not outrun the CATALOG - every bskforge_type it names
  must exist in bskforge's sensor union (the [custom]-ban law, one level up);
* landed rows may not LIE - every surface the row claims (topicid XML,
  mirror headers, EVS header, ZMQ topic, EDS container size, coverage packets,
  bus subscribers) must agree, so while a surface is still hand-maintained
  the registry audits it, and once generators own it the same tests keep
  the generators honest. The `landed` flag is the phase line: unregistered
  reality never silently passes, and registered-but-unlanded rows
  (attitude) are skipped until they arrive - then they flip.
"""

from __future__ import annotations

import re
from pathlib import Path
from typing import get_args, get_origin

import pytest

from sim.parts import Part, load_registry, mount_law_violations, mount_limits

REPO = Path(__file__).resolve().parent.parent
MISSION_DEFS = REPO / "cfs_mission" / "mission" / "orrery_defs"
EDS_DIR = MISSION_DEFS / "eds"
APPS = REPO / "cfs_mission" / "mission" / "apps"

PARTS = load_registry()
LANDED = [p for p in PARTS if p.landed]


def test_registry_loads_the_catalog() -> None:
    """The rows load, and the exhibit promise is present where it belongs."""
    names = {p.name for p in PARTS}
    assert {"gnss", "onboard", "attitude"} <= names
    # every row carries its soul: opinion parts promise an exhibit
    gnss = next(p for p in PARTS if p.name == "gnss")
    assert gnss.exhibit is not None and gnss.exhibit.kind == "truth_pair"
    assert gnss.declared["pos_sigma"] == "pos_sigma_m"


def test_rig_topics_are_exactly_the_landed_rows() -> None:
    """The sim rig and the spacecraft consumer agree through the registry alone.

    lockstep imports the catalog itself: the sim refuses to wire an
    observability provider on a topic no landed row owns, and this pins that
    its notion of LANDED is the same set the spacecraft's generic table
    drives - one file, both sides of the airgap.
    """
    from sim.lockstep import OBSERVER_TOPICS

    assert OBSERVER_TOPICS == {p.zmq_topic for p in LANDED if p.consumer == "bsk_link"}


def test_every_landed_part_ships_its_exhibit() -> None:
    """Every landed row that promises an exhibit runs in the coverage check.

    The check's row list and its sigma map read the same registry and the
    same manifest the plants are built from; a landed row the check would
    silently skip fails HERE, long before a live run has to notice.
    """
    from sim.fleet import scenario_vehicles
    from sim.truth_exhibits import SIGMAS, TRUTH_PAIR_ROWS

    promised = {p.name for p in PARTS if p.landed and p.exhibit is not None}
    assert {row.name for row in TRUTH_PAIR_ROWS} == promised
    twin_fleet = scenario_vehicles(REPO / "sim/scenarios/twin_ops.toml")
    assert set(SIGMAS) == {v.name for v in twin_fleet}
    for per_part in SIGMAS.values():
        assert per_part["gnss"] is not None  # the twin fleet fits receivers


def test_identity_surfaces_are_the_registry_rendered() -> None:
    """The generated identity surfaces == what emit_frames.py renders.

    The golden pin other generated artifacts enjoy: the C world-frame table,
    the part topic/MsgId mirrors, and the topicids XML region are only
    trusted because they are the registry's own text - regenerate with
    `uv run python -m codegen.emit_frames` when a row lands or moves.
    """
    from codegen.emit_frames import (
        OUT,
        OUT_MSGIDS,
        OUT_TOPICIDS,
        TOPICIDS_XML,
        emit_frames_header,
        emit_part_msgids,
        emit_part_topicids,
        render_topicids_xml,
    )

    parts = load_registry()
    assert OUT.read_text() == emit_frames_header(parts)
    assert OUT_TOPICIDS.read_text() == emit_part_topicids(parts)
    assert OUT_MSGIDS.read_text() == emit_part_msgids(parts)
    xml = TOPICIDS_XML.read_text()
    assert xml == render_topicids_xml(xml, parts)


def test_bskforge_types_exist_in_the_catalog() -> None:
    """No phantom parts.

    A row naming a type bskforge does not have would be wiring dressed as
    a part - the same law that bans [custom].

    The catalog is the schema's SENSOR union *and* its EFFECTOR union: a wheel
    set is a part as much as a star tracker is (the onboard bundle's frame carries
    wheel bytes), and a registry that could only name sensors would push
    actuator meaning back into hand-written code. It is also the ROOT's module
    sections (CAM): [[camera]] mounts instrument hardware on a craft exactly
    as a union device does, without belonging to either union - the opnav row
    claims it, so the catalog must see it.
    """
    from bskforge.schema.effectors import EffectorConfig
    from bskforge.schema.root import ScenarioConfig
    from bskforge.schema.sensors import SensorConfig

    catalog: set[str] = set()
    for union in (SensorConfig, EffectorConfig):
        for cls in get_args(get_args(union)[0]):  # Annotated[Union[...], discriminator]
            default = cls.model_fields["type"].default
            if isinstance(default, str):
                catalog.add(default)
    for name, field in ScenarioConfig.model_fields.items():
        if name != "spacecraft" and get_origin(field.annotation) is list:
            catalog.add(name)  # list-Sections are module sections; spacecraft is not hardware
    claimed = {t for p in PARTS for t in p.bskforge_types}
    assert claimed <= catalog, f"registry names types the schema lacks: {claimed - catalog}"


def test_topic_offsets_and_mids_are_unique_across_the_registry() -> None:
    """Two parts may never share an offset, a MsgId name, a wire topic, an EVS voice."""
    assert len({p.topic_offset for p in PARTS}) == len(PARTS), (
        "two rows at one topic_offset would render two names onto ONE TopicId: "
        "identical MsgIds on the bus, and nothing downstream would notice"
    )
    assert len({p.mid_name for p in PARTS}) == len(PARTS)
    assert len({p.zmq_topic for p in PARTS}) == len(PARTS)
    voices = [p.evs_info_id for p in PARTS if p.evs_info]
    assert len(set(voices)) == len(voices), (
        "an EVS id identifies WHICH part spoke: sharing one makes the operator's "
        "event log ambiguous, which is the one thing a boot-time audit must not be"
    )


@pytest.mark.parametrize("part", LANDED, ids=lambda p: p.name)
def test_identity_surfaces_agree(part: Part) -> None:
    """Topic id (XML), MsgId mirror (C), EVS event - all three agree or CI stops."""
    topicids = (MISSION_DEFS / "eds" / "cfe-topicids.xml").read_text()
    m = re.search(rf'<Define name="{re.escape(part.topic_name)}"[^>]*value="([^"]+)"', topicids)
    assert m and m.group(1).endswith(f"+ {part.topic_offset}"), (
        f"{part.name}: {part.topic_name} not at base+{part.topic_offset} in cfe-topicids.xml"
    )

    msgids = (MISSION_DEFS / "inc" / "orrery_msgids.h").read_text()
    # part MIDs are generated: the compiler reads both files as one
    msgids += (MISSION_DEFS / "inc" / "orrery_part_msgids.h").read_text()
    assert f"#define {part.mid_name}_MID" in msgids
    assert f"CFE_MISSION_{part.topic_name}" in msgids, (
        f"{part.name}: {part.mid_name}_MID does not derive from {part.topic_name}"
    )

    if part.evs_info:
        events = (APPS / "bsk_link" / "fsw" / "src" / "bsk_link_events.h").read_text()
        m = re.search(rf"#define {part.evs_info}\s+(\d+)", events)
        assert m and int(m.group(1)) == part.evs_info_id, f"{part.name}: EVS id drifted"


@pytest.mark.parametrize("part", LANDED, ids=lambda p: p.name)
def test_zmq_topic_lives_on_the_sensor_socket(part: Part) -> None:
    """The wire topic exists on the sim's sensor socket.

    This once meant a hand-written TOPIC_* constant in lockstep.py; now
    the rig DERIVES its observer-topic set from this registry and refuses
    to wire anything else (RigChannel.__post_init__), so the landed row IS
    the declaration - the mirror check imports rather than greps.
    """
    from sim.lockstep import OBSERVER_TOPICS

    assert part.zmq_topic in OBSERVER_TOPICS


@pytest.mark.parametrize("part", LANDED, ids=lambda p: p.name)
def test_container_exists_and_has_the_declared_size(part: Part) -> None:
    """The container byte count is shared by three sources.

    The EDS container the row names must be real and its payload exactly
    container_bytes - defs, the C assert, and the registry read one number.
    """
    from codegen.eds2cosmos import build_eds

    core = REPO / "cfs_mission/cFS"
    if not core.is_dir():
        pytest.skip("pinned cFS core not fetched here: scripts/fetch_cfs.sh")
    eds = build_eds(core, EDS_DIR)
    pkg, _, _ = part.eds_container.partition("/")
    items = eds.leaf(part.eds_container, pkg)
    bits = sum(i.bits for i in items)
    assert bits == part.container_bytes * 8, (
        f"{part.name}: {part.eds_container} is {bits // 8} B on the wire, registry says "
        f"{part.container_bytes} B"
    )


@pytest.mark.parametrize("part", [p for p in LANDED if p.exhibit], ids=lambda p: p.name)
def test_exhibit_packets_are_what_the_ground_reads(part: Part) -> None:
    """The exhibit names packets that provably exist on the ground.

    Generality made the check read whatever the row names - so the mirror
    moved DOWN to the source of packets: the flight side must be a real
    EDS SB interface (that is what generates the COSMOS def and the beacon
    slot), and the truth side must be the truth stream the check samples directly.
    """
    assert part.exhibit is not None
    interfaces = (EDS_DIR / "bsk_link.xml").read_text()
    iface = part.exhibit.flight_packet.removeprefix("BSK_LINK_")
    assert f'Interface name="{iface}"' in interfaces, (
        f"{part.name}: {part.exhibit.flight_packet} has no EDS interface - "
        "no COSMOS def, no beacon slot, coverage would sample a packet that cannot exist"
    )
    assert part.exhibit.truth_packet == "TRUTH_STATE_TLM"


@pytest.mark.parametrize("part", [p for p in LANDED if p.subscribers], ids=lambda p: p.name)
def test_bus_subscribers_actually_subscribe(part: Part) -> None:
    """A row claiming a bus consumer is checked against the app's source."""
    for app in part.subscribers:
        src = APPS / app / "fsw" / "src"
        files = list(src.glob("*.c"))
        assert files, f"{part.name}: subscriber app {app} has no sources"
        assert any(part.mid_name in f.read_text() for f in files), (
            f"{part.name}: {app} never mentions {part.mid_name} - not a subscriber"
        )


# --- the mounts law ---------------------------------------------------------

_MINIMAL_ROW = """\
schema_version = 1

[[part]]
name = "altimeter"
display = "radar altimeter"
bskforge_type = "radarAltimeter"
zmq_topic = "ALTSTATE"
eds_container = "BSK_LINK/AltitudeState"
container_bytes = 32
topic_offset = 209
topic_name = "BSK_LINK_ALTITUDE_STATE_TLM_TOPICID"
mid_name = "BSK_LINK_ALTITUDE_STATE_TLM"
consumer = "bsk_link"
"""


def _write_registry(tmp_path: Path, body: str) -> Path:
    path = tmp_path / "parts.toml"
    path.write_text(body, encoding="utf-8")
    return path


def test_a_mounts_law_may_only_legislate_for_hardware_the_row_names(tmp_path: Path) -> None:
    """A typo must not install a law for some other part's hardware."""
    path = _write_registry(tmp_path, _MINIMAL_ROW + "[part.instances]\nmagnetometer = 1\n")
    with pytest.raises(ValueError, match="does not name"):
        load_registry(path)


def test_two_rows_naming_one_type_must_agree_on_its_law(tmp_path: Path) -> None:
    """Several rows naming one type are several witnesses to one fact."""
    path = _write_registry(
        tmp_path,
        _MINIMAL_ROW.replace("radarAltimeter", "coarseSunSensor")
        + '[part.instances]\ncoarseSunSensor = 4\n\n[[part]]\nname = "b"\ndisplay = "b"\n'
        'bskforge_type = "coarseSunSensor"\nzmq_topic = "B"\neds_container = "B"\n'
        'container_bytes = 8\ntopic_offset = 210\ntopic_name = "TB"\nmid_name = "MB"\n'
        'consumer = "bsk_link"\n[part.instances]\ncoarseSunSensor = 2\n',
    )
    with pytest.raises(ValueError, match="disagree"):
        load_registry(path)


def test_the_law_defaults_to_single_and_the_registry_raises_it() -> None:
    """Every registered type is under law; the unlegislated stay absent."""
    limits = mount_limits()
    assert limits["coarseSunSensor"] == 4  # raised by the onboard row's table
    for t in ("starTracker", "magnetometer", "gpsReceiver", "reactionWheels", "camera"):
        assert limits[t] == 1, f"{t}: single is the law until a row says otherwise"
    assert "imuSensor" not in limits, "the barrier's IMU is physics, not a wire part"


def test_mount_law_violations_name_the_type_the_count_and_the_law() -> None:
    """Sentences the forge speaks with the vehicle's name in front."""
    assert mount_law_violations(["starTracker", "starTracker"]) == [
        "2 starTracker mounts; the registry law is 1 per vehicle"
    ]
    assert mount_law_violations(["coarseSunSensor"] * 4) == []
    assert mount_law_violations(["coarseSunSensor"] * 5) == [
        "5 coarseSunSensor mounts; the registry law is 4 per vehicle"
    ]
    assert mount_law_violations(["imuSensor", "imuSensor", "magnetometer"]) == []
