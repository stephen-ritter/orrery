"""The recorder sizes fly: the scenario's [[store]] rows become a compiled header.

A capacity nobody compiles in is a comment. ``codegen/emit_store`` lands the
flying scenario's recorder rows as a header the mission build consumes, so the
binary that flew carries the sizes the scenario declared and the provenance
stamp moves when they change - the same discipline that already governs the
build's app list and the per-vehicle startup table. What these tests pin:

* a recording scenario lands a row naming its vehicle and both bounds;
* an outlet that writes bytes with no sized recorder is refused before a
  single object file is compiled - the yard's copy of a law that also lives in
  bskforge's schema, because a dependency bump must not be able to soften it;
* the union default claims nothing: a quiet yard does not invent a recorder;
* the yard door lands the header on the way up and sweeps it on the way down,
  so a dead scenario's numbers cannot bias the next build.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from codegen.emit_store import StoreDoorError, render, store_rows, write_header

REPO = Path(__file__).resolve().parents[1]
SCENARIOS = REPO / "sim" / "scenarios"
FLEET = REPO / "scripts" / "fleet.sh"

FORMATION_EYE = SCENARIOS / "formation_eye.toml"
TWO_EYES = SCENARIOS / "two_eyes.toml"


def without_store(scenario: Path, tmp_path: Path) -> Path:
    """The same document with its recorder rows deleted."""
    text = scenario.read_text().split("\n[[store]]")[0]
    path = tmp_path / scenario.name
    path.write_text(text + "\n")
    assert "[[store]]" not in path.read_text(), "the helper kept the recorder"
    return path


def test_a_recording_vehicle_lands_its_recorder(tmp_path: Path) -> None:
    """The scenario's numbers, in the form the compiler reads them."""
    header = write_header(FORMATION_EYE, tmp_path / "orrery_store.h").read_text()
    assert '{"chief", "cam", 67108864ULL, 512U}' in header
    assert "Landed for formation_eye.toml: 1 recorder row(s)" in header


def test_two_eyes_share_the_one_row_they_declare(tmp_path: Path) -> None:
    """Two instruments, one store, one budget - and one row to prove it."""
    rows = store_rows(TWO_EYES)
    assert [(vehicle, payload) for vehicle, payload, _, _ in rows] == [("chief", "cam")]
    recording = 2  # two [[camera]] blocks with record_rate > 0
    assert len(rows) < recording, "two eyes declared two budgets: the law regressed"


def test_an_outlet_with_no_recorder_is_refused(tmp_path: Path) -> None:
    """The yard refuses to build what the craft could not obey."""
    with pytest.raises(StoreDoorError, match="no sized recorder"):
        store_rows(without_store(FORMATION_EYE, tmp_path))


def test_an_undersized_recorder_is_refused(tmp_path: Path) -> None:
    """A recorder sized zero is a store that can never say it is full."""
    broken = tmp_path / "zero.toml"
    broken.write_text(
        FORMATION_EYE.read_text().replace("capacity_bytes = 67108864", "capacity_bytes = 0")
    )
    with pytest.raises(StoreDoorError, match="not sized"):
        store_rows(broken)


def test_one_recorder_per_vehicle(tmp_path: Path) -> None:
    """A second row for the same store is a second truth about it."""
    text = FORMATION_EYE.read_text()
    broken = tmp_path / "two_rows.toml"
    broken.write_text(text + "\n" + text[text.index("[[store]]") :])
    with pytest.raises(StoreDoorError, match="two truths"):
        store_rows(broken)


def test_the_union_default_claims_no_recorder() -> None:
    """A quiet yard invents no hardware it did not declare."""
    header = render([], None)
    assert '{"", "", 0ULL, 0U}' in header
    assert "no vehicle claims a recorder" in header


def test_the_yard_door_lands_and_sweeps_the_sizes() -> None:
    """The sizes arrive with the scenario and leave when it does."""
    door = FLEET.read_text()
    assert "codegen.emit_store --scenario" in door, "the door builds without the sizes"
    assert "codegen.emit_store --sweep" in door, "a dead scenario's sizes survive"
