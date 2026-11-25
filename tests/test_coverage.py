"""The coverage ledger: every asset upstream exposes, classified by us.

"Have we supported X?" stopped being an archaeology dig the day it became a
file. These tests are the reason the file can be trusted: completeness is checked
against bskforge's *live* schema unions, ``landed`` is checked both ways against
the parts registry, every gap is required to carry a first missing hop and an
owner, and the rendered document must equal its own generator. A ledger nobody
audits is a README with better typography.
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest

from sim.coverage import (
    DOC,
    FIRST_HOPS,
    Coverage,
    TypeEntry,
    load_coverage,
    render_doc,
    schema_area_names,
    schema_disturbance_types,
    schema_effector_types,
    schema_sensor_types,
)
from sim.parts import load_registry

COVERAGE = load_coverage()
LANDED_ROWS = [p for p in load_registry() if p.landed]
UNLANDED = {"catalog-unlanded", "command-path", "refused"}


def _types(cov: Coverage) -> dict[str, TypeEntry]:
    return {t.name: t for t in cov.types}


def test_every_schema_type_is_classified() -> None:
    """Upstream added a part? This project now has to say what it does with it.

    Both directions, because a ledger that only checks "did we forget one" goes
    quietly stale the day a type is renamed upstream: a classification for a part
    that no longer exists reads exactly like a gap we chose.
    """
    schema = schema_sensor_types() | schema_effector_types() | schema_disturbance_types()
    ledger = {t.name for t in COVERAGE.types}

    missing = schema - ledger
    assert not missing, (
        f"unclassified bskforge type(s): {sorted(missing)} - "
        "add them to registry/coverage.toml, or delete the row that references them"
    )

    ghosts = ledger - schema
    assert not ghosts, (
        f"coverage classifies types upstream no longer has: {sorted(ghosts)} - "
        "a retired part should leave the ledger, not haunt it"
    )


def test_every_scenario_section_is_classified() -> None:
    """Whole scenario sections are assets too - formations, comm, Monte Carlo.

    A section upstream adds is a capability this project has not decided about.
    Silence about it is how "we support that" becomes true by accident.
    """
    schema = schema_area_names()
    ledger = {a.name for a in COVERAGE.areas}
    assert schema - ledger == set(), f"unclassified scenario section(s): {sorted(schema - ledger)}"
    assert ledger - schema == set(), (
        f"coverage classifies sections upstream dropped: {sorted(ledger - schema)}"
    )


def test_landed_agrees_with_the_parts_registry_both_ways() -> None:
    """A part is `landed` here iff a landed row covers it in `parts.toml`.

    The forward direction catches a landing that forgot the ledger (the doc then
    understates the project - harmless-looking, and it makes every other number
    in the file suspect). The reverse catches a claim whose row never landed or
    has since gone, which is the one this project has real history with.
    """
    row_types = {t for row in LANDED_ROWS for t in row.bskforge_types}
    claimed = COVERAGE.names_by_status("landed")

    unclassified = row_types - claimed
    assert not unclassified, (
        f"landed registry rows with no `landed` classification: {sorted(unclassified)}"
    )

    unsupported = claimed - row_types
    assert not unsupported, (
        f"`landed` in coverage but no landed row covers it: {sorted(unsupported)}"
    )


def test_an_actuator_earning_a_msgid_is_a_bug_not_a_feature() -> None:
    """Command-path parts must claim no wire; knowledge on the bus must claim one.

    This is the "Not every byte is a part" rule promoted from prose to a check:
    drag does not need a MsgId, and a ledger that lets it look like a missing
    sensor invites someone to go build one.
    """
    for entry in COVERAGE.types:
        if entry.status == "command-path":
            assert entry.wire == "", f"{entry.name}: a command-path part claims a wire"
            assert entry.command_path, f"{entry.name}: command-path with no command path named"
        if entry.status == "landed":
            assert entry.wire, f"{entry.name}: landed with no wire, so what landed?"


@pytest.mark.parametrize("name", sorted(t.name for t in COVERAGE.types if t.status in UNLANDED))
def test_every_gap_names_its_first_missing_hop_and_its_owner(name: str) -> None:
    """An ownerless gap is a rumor, and "eventually" is not a cost.

    The hop vocabulary is the same one docs/adding_a_sensor.md uses for a part's
    circuit, so the ledger and the recipe agree about what is missing - and the
    owner column turns this file into the queue it also happens to be.
    """
    entry = _types(COVERAGE)[name]
    assert entry.first_hop and entry.first_hop in FIRST_HOPS, f"{name}: no first missing hop"
    assert entry.owner, f"{name}: a gap with no owner"
    assert len(entry.why) > 40, f"{name}: a reason short enough to be a shrug"


def test_the_document_is_the_ledger_rendered() -> None:
    """docs/coverage.md is generated, and says so inside its own markers.

    Equality with the renderer is the only way the summary sentence can be
    trusted; a hand-tuned number inside the region is the exact failure mode the
    mirror tests were written against.
    """
    text = DOC.read_text(encoding="utf-8")
    assert text == render_doc(text, COVERAGE)


def test_the_summary_sentence_counts_the_rows() -> None:
    """The headline number, re-derived from the rows rather than re-read.

    A parser here instead of a string compare so the sentence can be reworded
    without breaking CI - but not renumbered.
    """
    text = DOC.read_text(encoding="utf-8")
    sentence = re.search(
        r"\*\*(\d+) of (\d+) part types are supported on the bus\*\*"
        r" \((\d+) landed, (\d+) as the barrier",
        text,
    )
    assert sentence is not None, "the doc's summary sentence was rewritten out of recognition"
    supported, total, landed, barrier = (int(g) for g in sentence.groups())

    assert total == len(COVERAGE.types)
    landed_count, barrier_count = COVERAGE.count("landed"), COVERAGE.count("barrier")
    assert (landed, barrier) == (landed_count, barrier_count)
    assert supported == landed_count + barrier_count

    areas = re.search(r"\*\*(\d+) of (\d+) scenario sections are used\*\*", text)
    assert areas is not None, "the doc's scenario-section count vanished"
    assert (int(areas.group(1)), int(areas.group(2))) == (
        COVERAGE.area_count("used"),
        len(COVERAGE.areas),
    )


def test_the_ledger_refuses_its_own_kinds_of_silence(tmp_path: Path) -> None:
    """A coverage file that misclassifies must fail at load, not at read time.

    The two failures chosen are the quiet ones: an invented status (a type
    quietly excluded from every check) and an invented hop (the cost column
    becoming prose).
    """
    base = """
[[type]]
name = "gpsReceiver"
kind = "sensor"
status = "landed"
bskforge = "schema+builder"
wire = "BSK_LINK/NavState"

[[area]]
name = "spacecraft"
status = "used"
"""
    path = tmp_path / "coverage.toml"
    path.write_text(base, encoding="utf-8")
    assert load_coverage(path).count("landed") == 1

    path.write_text(
        base.replace('status = "landed"', 'status = "sort-of-landed"'), encoding="utf-8"
    )
    with pytest.raises(ValueError, match="unknown status"):
        load_coverage(path)

    path.write_text(
        base.replace('status = "landed"', 'status = "catalog-unlanded"\nfirst_hop = "whenever"'),
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="unknown first_hop"):
        load_coverage(path)
