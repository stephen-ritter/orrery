"""The boot handshake: three views of one vehicle's fit.

The scenario's TOML CLAIMS a fit, the flight binary was BUILT from a registry
digest, and the bus STREAMS whatever actually decoded. These tests pin the
vocabulary both sides share - the bit order, the digest's rules, and the
injection knobs the acceptance flight uses - so a verdict on the event bus means
exactly what it says. The verdict itself is flown live by
``scripts/check_fit_handshake.sh``; nothing here can substitute for that flight.
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest

from sim.eds_link import available
from sim.forge_scenario import resolve_fit_knobs
from sim.lockstep import OBSERVER_TOPICS, TOPIC_FIT
from sim.parts import (
    REPO,
    frame_rows,
    load_registry,
    registry_digest,
)

FRAMES_H = REPO / "cfs_mission" / "mission" / "orrery_defs" / "inc" / "orrery_frames.h"
BSK_LINK_EDS = REPO / "cfs_mission" / "mission" / "orrery_defs" / "eds" / "bsk_link.xml"
TOPICIDS_XML = REPO / "cfs_mission" / "mission" / "orrery_defs" / "eds" / "cfe-topicids.xml"
PLUGIN = REPO / "cosmos_plugin"

ROWS = frame_rows(load_registry())
NAMES = [row.name for row in ROWS]


def _write_registry(tmp_path: Path, text: str) -> list:
    path = tmp_path / "parts.toml"
    path.write_text(text)
    return load_registry(path)


def _registry_text(changed: str, value: str) -> str:
    """The real registry with one field rewritten (semantics, not prose)."""
    text = (REPO / "registry" / "parts.toml").read_text()
    assert text.count(changed) >= 1, changed
    return text.replace(changed, value, 1)


def test_bit_order_is_the_generated_table_order() -> None:
    """A bit index means the same part in Python, in C, and on the console.

    The mask crosses the airgap as a number, so the ORDER is the contract. Both
    sides claim to render registry order; this is where that is checked instead
    of assumed.
    """
    table = FRAMES_H.read_text()
    c_topics = re.findall(r'^\s+\{ "([A-Z]+)", "BSK_LINK/', table, flags=re.MULTILINE)
    assert c_topics == [row.zmq_topic for row in ROWS]


def test_baked_digest_matches_the_catalog_the_world_loads() -> None:
    """The firmware's baked digest is the live registry's digest.

    If this test fails, either the header is stale (re-run ``emit_frames``) or
    the registry changed and every flight binary on the bench is about to abort
    its handshake - which is the correct verdict, but the offline check must say
    so before a fleet does.
    """
    baked = re.search(r"#define BSK_LINK_FIT_REGISTRY_HASH 0x([0-9a-f]{8})u", FRAMES_H.read_text())
    assert baked is not None, "no baked registry digest in orrery_frames.h"
    assert baked.group(1) == registry_digest(load_registry())


def test_digest_moves_with_wire_semantics_only(tmp_path: Path) -> None:
    """A reworded comment must not demand a rebuild; a new field name must.

    The digest is the build's identity, so its sensitivity profile is the
    project's: prose is free, the wire is not. Both directions are pinned -
    a digest blind to a container change would let stale firmware decode another
    field's bytes, and a digest that moved for a typo would have the operator
    rebuild a mission that cannot tell the difference.
    """
    base = load_registry()
    base_text = (REPO / "registry" / "parts.toml").read_text()
    assert registry_digest(base) == registry_digest(_write_registry(tmp_path, base_text))

    display_only = base_text.replace(
        'display = "GNSS receiver (orbital knowledge)"',
        'display = "GNSS, renamed by a poet"',
        1,
    )
    assert display_only != base_text
    assert registry_digest(_write_registry(tmp_path, display_only)) == registry_digest(base)

    # instance attribution counts at boot and decodes nothing: renaming the
    # id field is a law change, not a layout change, and must not demand a
    # rebuild - the frame's own bytes are untouched (layout is pinned by the
    # container audit and the manifest codec, not by this digest)
    id_field = base_text.replace('instance_field = "CameraId"', 'instance_field = "CameraIdV2"', 1)
    assert id_field != base_text
    assert registry_digest(_write_registry(tmp_path, id_field)) == registry_digest(base)

    for label, changed in (
        ("topic", _registry_text('zmq_topic = "NAVSTATE"', 'zmq_topic = "NAVSTATE2"')),
        (
            "container",
            _registry_text(
                'eds_container = "BSK_LINK/NavState"', 'eds_container = "BSK_LINK/NavStateV2"'
            ),
        ),
        ("size", _registry_text("container_bytes = 64", "container_bytes = 72")),
        ("evs voice", _registry_text("evs_info_id = 15", "evs_info_id = 19")),
    ):
        assert registry_digest(_write_registry(tmp_path, changed)) != registry_digest(base), label


def test_declared_mask_reads_the_manifest_not_the_wire() -> None:
    """A row is claimed when the TOML declares any type it covers.

    The bundle row covers four instruments, so one CSS claims it; a vehicle with
    no GNSS cannot claim the nav frame no matter what its bus later streams -
    that disagreement is precisely what the handshake reports.
    """
    from sim.parts import declared_mask

    rows = ROWS
    by_name = {row.name: row for row in rows}

    def bit(name: str) -> int:
        return 1 << NAMES.index(name)

    assert declared_mask(["gpsReceiver"], None, rows) == bit("gnss")
    # The star tracker is the fan-out in one line: its type feeds the onboard
    # bundle AND the registry's own attitude frame, so claiming the device
    # claims both wires. A manifest that names one sensor is not naming one
    # part, and the handshake report says so in two bits.
    assert declared_mask(["starTracker"], None, rows) == bit("onboard") | bit("attitude")
    assert declared_mask(["coarseSunSensor"], None, rows) == bit("onboard")
    wheels_and_nav = declared_mask(["reactionWheels", "gpsReceiver"], None, rows)
    assert wheels_and_nav == bit("onboard") | bit("gnss")
    assert declared_mask([], None, rows) == 0
    # a type no row covers claims nothing (and is refused elsewhere, by the
    # registry's first-class-type audit)
    assert declared_mask(["someDeviceNobodyLanded"], None, rows) == 0

    # The bundle's coverage is READINGS, not nearby hardware. Its frame carries
    # wheel speeds, ST quaternion, CSS cosines and the magnetic field, so the
    # effector is in the list; the IMU is not, because its bytes ride the
    # barrier's SENSOR frame and a vehicle with only an IMU streams no bundle.
    # A row that claimed otherwise would cry SILENT at every boot of a vehicle
    # that is honestly fitted - and an alarm nobody trusts is worse than none.
    assert by_name["onboard"].bskforge_types == (
        "reactionWheels",
        "starTracker",
        "coarseSunSensor",
        "magnetometer",
    )
    assert declared_mask(["imuSensor"], None, rows) == 0


def test_injections_lie_about_bits_and_never_the_digest() -> None:
    """The knobs a check uses are validated before a plant is built.

    An override flips a claim. A digest override simulates STALE FIRMWARE, and
    only that: a test about a missing sensor must not also pretend the catalog
    changed, or a passing verdict proves nothing about either.
    """
    override, id_inject, digest = resolve_fit_knobs(["attitude"], ["gnss"], "deadbeef", [])
    assert override == {"attitude": True, "gnss": False}
    assert id_inject == {}
    assert digest == "deadbeef"

    # the id knob aims only at id rows, only at ids the mask can hold
    assert resolve_fit_knobs([], [], None, ["camstate=2"])[1] == {"camstate": {2}}
    with pytest.raises(ValueError, match="pick one per part"):
        resolve_fit_knobs(["attitude"], ["attitude"], None, [])
    with pytest.raises(ValueError, match="no such registered part"):
        resolve_fit_knobs(["fluxCapacitor"], [], None, [])
    with pytest.raises(ValueError, match="wants ROW=ID"):
        resolve_fit_knobs([], [], None, ["attitude=2"])
    with pytest.raises(ValueError, match="does not fit"):
        resolve_fit_knobs([], [], None, ["camstate=33"])
    with pytest.raises(ValueError, match="8 hex digits"):
        resolve_fit_knobs([], [], "not-hex", [])
    assert resolve_fit_knobs([], [], "0x0", [])[2] == "00000000"


def test_handshake_never_reaches_the_ground() -> None:
    """No TopicId, so no MsgId, no ground definition, no beacon line.

    The fit is the spacecraft's own claim about itself. Giving it a telemetry
    identity would put the ground's copy of the verdict on the wire beside the
    flight one, and the next reader would grade the wrong number. The absence is
    the tripwire: a leak here turns into an UNKNOWN packet on the interface
    service, which the beacon-defs check already refuses.
    """
    eds = BSK_LINK_EDS.read_text()
    assert 'ContainerDataType name="FitManifest"' in eds
    block = eds.split('ContainerDataType name="FitManifest"', 1)[1]
    block = block.split("</ContainerDataType>", 1)[0]
    for field in ("SimTimeS", "Frame", "DeclaredBits", "RegistryHash"):
        assert f'name="{field}"' in block
    for slot in range(4):
        assert f'name="DeclaredIds{slot}"' in block

    assert "FitManifest" not in TOPICIDS_XML.read_text()
    leaked = [
        path
        for path in PLUGIN.rglob("*")
        if path.is_file() and "FitManifest" in path.read_text(errors="ignore")
    ]
    assert leaked == [], f"handshake leaked into the ground defs: {leaked}"


@pytest.mark.skipif(not available(), reason="EdsLib database not built")
def test_the_handshake_round_trips_through_the_mission_schema() -> None:
    """The handshake numbers survive the mission's own codec, id slots included.

    The bits, the digest, and the four instance masks are the whole message;
    if any were silently truncated by the container's width, the spacecraft
    would compare two numbers that were never meant to be equal - the
    quietest possible way to be wrong.
    """
    from sim.eds_link import decode_fit_manifest, encode_fit_manifest

    packed = bytes(encode_fit_manifest(12.5, 7, 0b101, "f0b0bfb6", [0b1, 0, 0, 0]))
    assert len(packed) == 40
    frame = decode_fit_manifest(packed)
    assert frame.sim_time_s == 12.5
    assert frame.frame == 7
    assert frame.declared_bits == 0b101
    assert frame.registry_hash == "f0b0bfb6"
    assert frame.declared_ids == (0b1, 0, 0, 0)

    # an honest one-eye claim: slot 0 (camstate) carries id 1 and nothing else
    honest = decode_fit_manifest(bytes(encode_fit_manifest(1.0, 2, 0b1, "f0b0bfb6", [0b1])))
    assert honest.declared_ids == (0b1, 0, 0, 0)
    with pytest.raises(ValueError, match="4 slots"):
        encode_fit_manifest(1.0, 2, 0, "f0b0bfb6", [1, 2, 3, 4, 5])


def test_the_handshake_is_not_a_part() -> None:
    """FIT is control chatter, so it may not register as an observability part.

    ``OBSERVER_TOPICS`` is the registry's landed set; a topic in it is a part with
    a MsgId, a container, and a ground definition. The handshake has none of
    those, and the C side reads it on the sensor socket beside the barrier's own
    frames. Keeping it out of that set is what keeps the previous test true.
    """
    assert TOPIC_FIT == b"FIT"
    assert TOPIC_FIT.decode() not in OBSERVER_TOPICS
    assert all(row.zmq_topic != TOPIC_FIT.decode() for row in ROWS)
