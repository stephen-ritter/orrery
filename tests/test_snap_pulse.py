"""The shutter pulse: a snapshot is the frame AFTER the ask, or nothing.

CAM_CMD FC 2 says "look now". The laws that make those two words honest,
pinned without a flight because they are laws of text, order, and arithmetic
that a bench can only sample once:

* the edge is single: the FSW forwards Pulse=1 on exactly one call site
  (the SNAP handler); SET_CONFIG - and any re-sent config - carries 0,
  so no duplicate frame can buy a second snapshot;
* the pulse pays with the NEXT capture: an ask is satisfied only by a
  capture whose ordinal is strictly past the baseline recorded at the ask,
  so a snapshot can never be an old file handed over early;
* patience, not lies: a file not on disk yet leaves the ask pending
  (shutter held open for it); a deadline passes and drops the ask
  UNCOUNTED - CamState.Snapshots below CAM_HK.SnapshotCmds is then the
  true story on the wire, not a swallowed pulse;
* the shutter is borrowed, not re-decided: while an ask is open the pulse
  owns it, and the moment the queue drains the ground's last steady
  command comes back.

The schema half rides the generated defs (test_parts pins the registry's
container_bytes against them); here the containers are pinned to speak the
pulse at all: CamConfig carries Pulse, CamState answers with Snapshots,
CAM_HK counts the asks.
"""

from __future__ import annotations

import re
import struct
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import pytest

REPO = Path(__file__).resolve().parents[1]
BXML = REPO / "cfs_mission" / "mission" / "orrery_defs" / "eds" / "bsk_link.xml"
CXML = REPO / "cfs_mission" / "mission" / "orrery_defs" / "eds" / "cam.xml"
CAM_C = REPO / "cfs_mission" / "mission" / "apps" / "cam" / "fsw" / "src" / "cam_app.c"


def _container_entries(xml: str, name: str) -> list[str]:
    """Entry names of one ContainerDataType, in declaration order."""
    block = re.search(rf'<ContainerDataType name="{name}".*?</ContainerDataType>', xml, re.DOTALL)
    assert block, f"{name} is no longer in the EDS tree"
    return re.findall(r'<Entry name="([^"]+)"', block.group(0))


def test_the_wire_speaks_the_pulse_on_both_sides() -> None:
    """Pulse goes down, Snapshots comes back up - last fields, both twins."""
    bxml = BXML.read_text()
    assert _container_entries(bxml, "CamConfig")[-1] == "Pulse", (
        "the ask edge must ride the config frame"
    )
    assert _container_entries(bxml, "CamState")[-1] == "Snapshots", (
        "the delivery witness must ride the device view"
    )
    hk = _container_entries(CXML.read_text(), "HkTlm_Payload")
    assert "SnapshotCmds" in hk, "the asks have no counter: divergence would be invisible"


def test_the_pulse_is_an_edge_with_exactly_one_source() -> None:
    """cam_app forwards Pulse=1 from ONE call site and Pulse=0 from the other.

    The pending-flag trap: a steady config re-forwarded for any reason must
    not carry a stale pulse, so the edge is a parameter, not state - the
    call-site census is the proof. FC 2 is header-only (the same CAM_CMD
    contract as NOOP and MANIFEST), and SNAP before the first SET_CONFIG is
    refused: pulsing eye 0 would be the app guessing at TOML it never read.
    """
    c = CAM_C.read_text()
    calls = re.findall(r"CAM_ForwardConfig\((\w+)\)", c)
    assert sorted(calls) == ["0", "1"], (
        f"config forwards must be exactly the steady (0) and pulse (1) edges: {calls}"
    )
    assert "case 2:" in c and "CAM_HandleSnap" in c, (
        "FC 2 is in the schema but not in the dispatcher"
    )
    assert "CameraId == 0" in c[c.index("static void CAM_HandleSnap") :], (
        "SNAP does not refuse an uncommanded eye"
    )
    xml = CXML.read_text()
    snap = re.search(r'<ContainerDataType name="SnapCmd".*?</ContainerDataType>', xml, re.DOTALL)
    assert snap and 'entry="Sec.FunctionCode" value="2"' in snap.group(0)


# -- the sim door, unit-tested without a world ---------------------------------


def _body(tmp: Path, rate: int = 100, mode: str = "cadence") -> Any:  # noqa: ANN401 - a bundle of test doubles
    """A ForgeBody skeleton wired only to a store outlet, plus its captures.

    Returns (body, store, emitted, eye, captures). Captures 1..4 exist on
    disk (t = 1.0 .. 4.0 s); the cadence door is pinned shut (rate=100) so
    anything sent in these tests is a pulse's doing, and nothing else can
    move the counters. ``mode`` sets the outlet door: "snap" closes the
    cadence door by declaration instead of by distance.
    """
    from sim import forge_scenario as fz

    captures = [1.0, 2.0, 3.0, 4.0]
    for t in captures:
        (tmp / f"{t:.6f}.png").write_bytes(b"png-" + bytes([int(t * 7)]) * 8)

    emitted: list[bytes] = []
    body = object.__new__(fz.ForgeBody)
    body.name = "unit"
    body._store_emit = emitted.append
    store = {
        "mode": mode,
        "rate": rate,
        "dir": tmp,
        "sent": [],
        "count": 0,
        "snap": [],
        "snaps": 0,
        "steady_shutter": 0,
    }
    module = SimpleNamespace(cameraIsOn=1, renderRate=1_000_000_000)
    eye = (1, module, None, None, None, None, store)
    return body, store, emitted, eye, captures


def _sent_ordinals(emitted: list[bytes]) -> list[int]:
    """Which captures left the outlet, read from the IMG chunk headers."""
    return [struct.unpack_from("<QQII", payload)[1] for payload in emitted]


def test_a_pulse_pays_with_the_next_capture_then_restores(tmp_path: Path) -> None:
    """The whole law in one tick.

    Ask at baseline 2 -> capture 3 leaves, the tally moves, the queue drains,
    and the closed shutter closes again.
    """
    body, store, emitted, eye, captures = _body(tmp_path)
    store["snap"].append({"baseline": 2, "deadline_s": 999.0})
    assert body._pump_snap(eye, captures, sim_time_s=10.0) == 1
    assert _sent_ordinals(emitted) == [3], "the pulse took a frame other than the next capture"
    assert store["snap"] == [] and store["snaps"] == 1
    assert eye[1].cameraIsOn == 0, "the pulse left the shutter it borrowed open"


def test_a_pulse_waits_patiently_and_holds_the_shutter(tmp_path: Path) -> None:
    """The next file is not on disk yet: no send, no count, ask kept.

    The shutter stays open too, because the pulse still owns its promise.
    """
    body, store, emitted, eye, captures = _body(tmp_path)
    (tmp_path / "3.000000.png").unlink()
    store["steady_shutter"] = 0
    eye[1].cameraIsOn = 0
    store["snap"].append({"baseline": 2, "deadline_s": 999.0})
    assert body._pump_snap(eye, captures, sim_time_s=10.0) == 0
    assert emitted == [] and store["snaps"] == 0
    assert len(store["snap"]) == 1, "the ask gave up before its deadline"
    assert eye[1].cameraIsOn == 1, "the pulse closed the shutter before its frame could render"
    # the file appears: the ask cashes it on the next tick, no new ask needed
    (tmp_path / "3.000000.png").write_bytes(b"late")
    assert body._pump_snap(eye, captures, sim_time_s=11.0) == 1
    assert _sent_ordinals(emitted) == [3] and eye[1].cameraIsOn == 0


def test_an_ask_never_buys_two_frames(tmp_path: Path) -> None:
    """A standing send satisfies the ask without a second frame leaving."""
    body, store, emitted, eye, captures = _body(tmp_path)
    store["sent"].append("3.000000.png")  # already out the door
    store["snap"].append({"baseline": 2, "deadline_s": 999.0})
    assert body._pump_snap(eye, captures, sim_time_s=10.0) == 1
    assert emitted == [], "a standing send was re-sent for the same ask"
    assert store["snaps"] == 1 and store["snap"] == []


def test_a_missed_deadline_drops_the_ask_uncounted(tmp_path: Path) -> None:
    """A pulse the world never answered is dropped with a line, not a lie.

    No count, no frame, and the shutter is given back to the ground's word.
    """
    body, store, emitted, eye, captures = _body(tmp_path)
    (tmp_path / "3.000000.png").unlink()
    store["steady_shutter"] = 0
    eye[1].cameraIsOn = 0
    store["snap"].append({"baseline": 2, "deadline_s": 9.0})
    assert body._pump_snap(eye, captures, sim_time_s=10.0) == 0
    assert store["snap"] == [] and store["snaps"] == 0 and emitted == []
    assert eye[1].cameraIsOn == 0, "a dropped ask left the shutter commandeered"


def test_the_config_frame_round_trips_with_its_edge() -> None:
    """Pulse survives the schema both ways, and steady frames default to 0."""
    from sim.eds_link import decode_cam_config, encode_cam_config

    steady = bytes(encode_cam_config(3.5, 42, 1, 0, 4.25))
    assert len(steady) == 48, "the CamConfig row declares 48 B (Pulse included)"
    assert decode_cam_config(steady).pulse == 0

    pulse = bytes(encode_cam_config(4.0, 43, 1, 0, 4.25, pulse=1))
    cfg = decode_cam_config(pulse)
    assert cfg.pulse == 1 and cfg.shutter == 0, (
        "the edge must not masquerade as a steady shutter command"
    )


def test_snap_mode_closes_the_cadence_door_by_declaration(tmp_path: Path) -> None:
    """Snap mode is the door DECLARED, not a cadence faked by distance.

    The old bench got this from record_rate 1000 - arithmetic pretending to
    be a law. Here the cadence door does not exist: pumping a streaming
    eye's captures ships nothing, rate 0 never reaches the modulo (the
    rung-4 divide-by-zero landmine), and one ask ships exactly one capture.
    A delivered pulse IS a recorded frame: the outlet owns the tally.
    """
    body, store, emitted, eye, captures = _body(tmp_path, rate=0, mode="snap")
    assert body._pump_store(eye, captures) == 0, "snap mode shipped a steady capture"
    assert emitted == []
    store["snap"].append({"baseline": 2, "deadline_s": 999.0})
    assert body._pump_snap(eye, captures, sim_time_s=10.0) == 1
    assert _sent_ordinals(emitted) == [3]
    assert body._pump_store(eye, captures) == 1, "a delivered pulse is not counted as recorded"
    assert body._pump_store(eye, captures) == 1, "the steady door stays shut forever"


def test_the_snap_scenario_declares_its_own_door() -> None:
    """camera_snap.toml: the record bird with the declaration, no arithmetic.

    The schema refuses record_rate beside record_mode upstream; this pins
    the yard's own scenario to keep the cadence door out of the file
    entirely and the recorder sized for the bytes pulses will write.
    """
    import tomllib

    facts = tomllib.loads((REPO / "sim" / "scenarios" / "camera_snap.toml").read_text())
    cams = facts["camera"]
    assert len(cams) == 1 and cams[0]["record_mode"] == "snap"
    assert "record_rate" not in cams[0], (
        "cadence smuggled beside a declared snap door: the schema would refuse this file"
    )
    assert cams[0].get("downlink", False) is False
    assert any(row["parent"] == cams[0]["parent"] for row in facts["store"]), (
        "an open outlet with no sized recorder"
    )


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-q"]))
