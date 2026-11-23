"""The Basilisk <-> cFS link contract: one schema, target-native payload bytes.

Two guarantees, both cheap and both easy to break by accident:

1. ``cfs_mission/mission/orrery_defs/eds/bsk_link.xml`` is the only description of
   the link frames, it declares its data types little-endian, and EdsLib therefore
   produces exactly the bytes a plain C struct in an FSW app reads. If a type is
   "tidied" to inherit ``${CFE_MISSION/DATA_BYTE_ORDER}`` (bigEndian), the simulator
   and the flight code start disagreeing about every float on the link, silently.
2. The EdsLib database build (``edslink/``) and ``codegen/eds2cosmos`` consume the
   same XML files, so ground definitions and the runtime schema cannot drift.
"""

from __future__ import annotations

import os
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from codegen.eds_model import load_eds
from codegen.eds_sources import (
    CFS,
    CMAKE_MANIFEST,
    MISSION_DEFS_EDS,
    REPO,
    all_xml_files,
    cmake_manifest,
    db_source_dirs,
    mission_define_files,
)

pytestmark = pytest.mark.skipif(
    not CFS.is_dir(), reason="cFS is not fetched: run scripts/fetch_cfs.sh"
)

# The link package lives in the mission EDS dir since P3c: its two frames
# graduated to the Software Bus, so they need TopicIds next to the other
# mission topics. Same file, same guarantees, fewer directories.
LINK_XML = MISSION_DEFS_EDS / "bsk_link.xml"
EDSLINK = REPO / "edslink"
DIST = EDSLINK / "dist"
ENCODINGS = ("IntegerDataEncoding", "FloatDataEncoding", "RealDataEncoding")


def test_link_package_declares_target_native_order() -> None:
    """Every link data type pins littleEndian; nothing inherits the bigEndian default."""
    root = ET.parse(LINK_XML).getroot()
    encodings = [el for el in root.iter() if el.tag.rsplit("}")[-1] in ENCODINGS]
    assert len(encodings) >= 3, "link package lost a data type"
    for enc in encodings:
        assert enc.get("byteOrder") == "littleEndian", (
            f"a {enc.get('sizeInBits')}-bit link field is {enc.get('byteOrder')!r}; the "
            "non-EDS cFE runtime puts payload bytes in target-native order"
        )
    assert 'byteOrder="bigEndian"' not in LINK_XML.read_text()


def test_codegen_parses_the_link_package_too() -> None:
    """Our codegen and NASA's toolchain must accept the same link XML."""
    eds = load_eds(CFS, mission_define_files(), extra_files=[])
    assert "BSK_LINK/SensorFrame" in eds.types
    assert "BSK_LINK/TorqueCommand" in eds.types
    assert "BSK_LINK/OnboardState" in eds.types, "P4b frame missing from the shared schema"


def test_cmake_mirror_matches_the_python_enumeration() -> None:
    """edslink/eds_dirs.cmake is a current mirror of codegen.eds_sources."""
    assert CMAKE_MANIFEST.read_text() == cmake_manifest(db_source_dirs()), (
        "run: uv run python -m codegen.eds_sources --write"
    )
    assert LINK_XML in all_xml_files(), "link package missing from the shared EDS file list"


def test_step_ack_decode_matches_packed_bytes() -> None:
    """The sim's gate decodes the 24 B barrier ack from raw struct bytes.

    The bytes are what bsk_link forwards verbatim from the bus: struct.pack
    here is the same contract verify.py pins on the encode side, seen from the
    consumer that actually blocks on it (``DetumbleScenario._await_barrier``).
    """
    if not (DIST / "EdsLib.so").is_file():
        pytest.skip("run edslink/build.sh to build the EdsLib database and binding")
    import struct

    from sim.eds_link import decode_step_ack

    payload = struct.pack("<dQb3xI", 4.5, 9, 1, 7)
    ack = decode_step_ack(payload)
    assert (ack.sim_time_s, ack.frame, ack.mode, ack.torques_issued) == (4.5, 9, 1, 7)


def test_onboard_state_codec_round_trip() -> None:
    """P4b observability frame: exact values out, plain C bytes on the wire.

    R4: the sim packs observability containers through ONE registry-checked
    codec - the dict keys are the schema's field names, nothing per-frame.
    """
    if not (DIST / "EdsLib.so").is_file():
        pytest.skip("run edslink/build.sh to build the EdsLib database and binding")
    import struct

    from sim.eds_link import decode_onboard_state, encode_container

    wheels = (123.5, -60.25, 0.5)
    quat = (0.99, 0.01, -0.02, 0.03)
    css = (0.8, 0.1, 0.0, 0.5)
    mag = (-2.6e-5, 5e-6, 1.1e-5)

    payload = encode_container(
        "BSK_LINK/OnboardState",
        {
            "SimTimeS": 5.5,
            "Frame": 11,
            "WheelSpeeds": wheels,
            "StQuatBn": quat,
            "CssCos": css,
            "MagB": mag,
        },
    )
    assert len(payload) == 128, "the registry row and the C twin pin 128 B"
    state = decode_onboard_state(payload)
    assert (state.sim_time_s, state.frame) == (5.5, 11)
    assert (state.wheel_speeds, state.st_quat_bn, state.css_cos, state.mag_b) == (
        wheels,
        quat,
        css,
        mag,
    )
    assert payload == struct.pack("<dQ14d", 5.5, 11, *wheels, *quat, *css, *mag)


def test_nav_state_codec_round_trip() -> None:
    """KN observability frame: the GNSS opinion out exactly, plain C bytes in."""
    if not (DIST / "EdsLib.so").is_file():
        pytest.skip("run edslink/build.sh to build the EdsLib database and binding")
    import struct

    from sim.eds_link import decode_nav_state, encode_container

    pos = (7.0e6, -2.0e5, 1.0e3)
    vel = (-0.3, 7.1, 0.2)

    payload = encode_container(
        "BSK_LINK/NavState",
        {"SimTimeS": 6.5, "Frame": 13, "RBnN": pos, "VBnN": vel},
    )
    assert len(payload) == 64, "the registry row and the C twin pin 64 B"
    fix = decode_nav_state(payload)
    assert (fix.sim_time_s, fix.frame) == (6.5, 13)
    assert (fix.r_bn_n, fix.v_bn_n) == (pos, vel)
    assert payload == struct.pack("<dQ6d", 6.5, 13, *pos, *vel)


def test_opnav_container_bytes_pinned_before_landing() -> None:
    """CAM knowledge frame: the byte layout is pinned today, the encode door opens with the row.

    The container ships before the part lands (the registry's phase line), so
    what is testable today is the WIRE: 72 bytes packing exactly as the
    rendered-pixel probe measured, and decoding to the fields it promises.
    The registry's encode door must stay shut - a provider packing a frame
    the spacecraft never agreed to decode is a bug, raised at the edge of
    the world rather than a fiction inside it.
    """
    if not (DIST / "EdsLib.so").is_file():
        pytest.skip("run edslink/build.sh to build the EdsLib database and binding")
    import struct

    from sim.eds_link import (
        LinkUnavailable,
        _lib,
        decode_opnav_state,
        encode_container,
        entry,
    )

    los = (1.2e7, -3.4e5, 5.6e4)
    lib = _lib()
    obj = entry("BSK_LINK/OpNavState")()
    obj["SimTimeS"] = 3.5
    obj["Frame"] = 7
    for index, value in enumerate(los):
        obj["RBnB"][index] = value
    obj["PlanetId"] = 1
    obj["CameraId"] = 2
    obj["Valid"] = 1
    obj["FaultDetected"] = 0
    payload = bytes(lib.PackedObject(obj))

    assert payload == struct.pack("<dQ3d4Q", 3.5, 7, *los, 1, 2, 1, 0)
    assert len(payload) == 72, "the registry row and the XML container must agree on 72 B"
    fix = decode_opnav_state(payload)
    assert (fix.sim_time_s, fix.frame) == (3.5, 7)
    assert fix.r_bn_b == pytest.approx(los)
    assert (fix.planet_id, fix.camera_id) == (1, 2)
    assert fix.valid and not fix.fault_detected

    with pytest.raises(LinkUnavailable, match="not a landed registry container"):
        encode_container("BSK_LINK/OpNavState", {"SimTimeS": 3.5})


def test_container_codec_refuses_unregistered_containers() -> None:
    """Observability codecs come from the registry (REG): no row, no codec.

    The control-plane frames (SensorFrame and friends) are not parts and
    never pack through this door - a provider asking for an unregistered or
    unlanded container is a bug at the edge of the world, raised there.
    """
    if not (DIST / "EdsLib.so").is_file():
        pytest.skip("run edslink/build.sh to build the EdsLib database and binding")
    from sim.eds_link import LinkUnavailable, encode_container

    with pytest.raises(LinkUnavailable, match="not a landed registry container"):
        encode_container("BSK_LINK/SensorFrame", {"SimTimeS": 1.0, "Frame": 2})
    with pytest.raises(LinkUnavailable, match="not a landed registry container"):
        encode_container("BSK_LINK/CameraState", {"SimTimeS": 1.0})  # CAM-era container, unwritten


def test_edslink_dist_env_selects_the_worlds_own_binding(tmp_path: Path) -> None:
    """A CPython extension is not portable between interpreters.

    So the importer must be told which world it is in - EDSLINK_DIST is that
    saying. Out of process, because that is how the worlds consume it: every
    container, and every honest check, is a fresh interpreter deciding its
    DIST at import. (Reloading the module in-process would rewrite the very
    module object the other tests' worlds are running - reload mutates in
    place - which is the same "one module, two worlds" mistake this knob
    exists to prevent.)
    """
    probe = "import sim.eds_link as m; print(m.DIST); print(m.available())"
    other = tmp_path / "cp312-dist"
    other.mkdir()
    env = {**os.environ, "EDSLINK_DIST": str(other)}
    pointed = subprocess.run(
        [sys.executable, "-c", probe],
        capture_output=True,
        text=True,
        check=False,
        env=env,
    )
    assert pointed.returncode == 0, pointed.stdout + pointed.stderr
    lines = pointed.stdout.splitlines()
    assert lines[0] == str(other)
    # nothing was built there: the module must report the link absent, never
    # reach back into the tree's (foreign) artifacts
    assert lines[1] == "False"

    host = subprocess.run(
        [sys.executable, "-c", probe],
        capture_output=True,
        text=True,
        check=False,
    )
    assert host.returncode == 0, host.stdout + host.stderr
    assert host.stdout.splitlines()[0] == str(EDSLINK / "dist")


def test_built_database_encodes_frames_as_c_structs() -> None:
    """edslink/verify.py: EdsLib bytes == struct.pack bytes, for every link frame.

    Run out of process so the check exercises the same import path a fresh
    interpreter takes (the binding dlopen()s ``orrery_eds_db.so`` by name).
    """
    if not (DIST / "EdsLib.so").is_file():
        pytest.skip("run edslink/build.sh to build the EdsLib database and binding")
    result = subprocess.run(
        [sys.executable, str(EDSLINK / "verify.py")],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert "EDSLINK VERIFY OK" in result.stdout
