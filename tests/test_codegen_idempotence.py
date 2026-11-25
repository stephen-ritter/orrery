"""Generation is a function of its inputs, so regeneration changes nothing.

The claim is worth stating out loud, because its absence cost an acceptance check:
the emitters used to stamp the wall-clock second into every generated file. So
"did the wire schema move?" could not be read off a diff, regenerating an
unchanged fleet dirtied six tracked files, and a test that compared two emissions
failed whenever the calls happened to straddle a second boundary. Headers now
carry a digest of the content they describe, and these tests keep it that way:

* the committed bytes are what the generators produce **right now** — a definition
  drifted from its schema fails here rather than as a decode error on the ground;
* no generated artifact records a clock, anywhere;
* two emissions in one process are byte-identical, the cheapest possible
  regression against a timestamp creeping back in.
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest

from codegen.eds2cosmos import (
    TRUTH_CONTAINER,
    TRUTH_PKG,
    _cfs_sha,
    build_eds,
    emit_c_cmd_table,
    emit_c_table,
    emit_cosmos,
    emit_cosmos_truth,
    mission_packets,
)
from codegen.eds_model import Eds, Packet

REPO = Path(__file__).resolve().parent.parent
CFS = REPO / "cfs_mission" / "cFS"
MISSION_DEFS = REPO / "cfs_mission" / "mission" / "orrery_defs" / "eds"
TARGETS = REPO / "cosmos_plugin" / "targets"
C_TABLE = REPO / "cfs_mission/mission/apps/gs_tcp/fsw/src/rs_mid_table.h"
CMD_TABLE = REPO / "cfs_mission/mission/apps/sps/fsw/src/sps_cmd_table.h"
INC = REPO / "cfs_mission/mission/orrery_defs/inc"

FLIGHT_TARGETS = ("SAT_CHIEF", "SAT_DEPUTY")
TRUTH_TARGETS = ("TRUTH_CHIEF", "TRUTH_DEPUTY")

# A generated file must not record WHEN it was made. This one pattern is the
# difference between a diff that means something and a diff that lies.
CLOCK = re.compile(r"\b20\d{2}-\d{2}-\d{2}[T ]\d{2}:\d{2}")


@pytest.fixture(scope="module")
def eds() -> Eds:
    """The loaded EDS model of the pinned checkout (core must be fetched)."""
    if not CFS.is_dir():
        pytest.skip("pinned cFS core not fetched here: scripts/fetch_cfs.sh")
    return build_eds(CFS, MISSION_DEFS)


@pytest.fixture(scope="module")
def packets(eds: Eds) -> list[Packet]:
    """The packets the ground actually defines (registry- and beacon-filtered)."""
    return mission_packets(eds.packets())


def _defs(target: str) -> list[Path]:
    return sorted((TARGETS / target / "cmd_tlm").glob("*.txt"))


def test_flight_defs_are_their_own_golden(eds: Eds, packets: list[Packet]) -> None:
    """Each committed def file equals what the generator emits right now, byte for byte.

    The file *is* the golden, so there is nothing to re-anchor and no tolerance to
    argue about: a definition hand-edited on the ground, or regenerated against a
    different checkout, fails here instead of decoding silently wrong in COSMOS.
    """
    for target in FLIGHT_TARGETS:
        cmd_text, tlm_text = emit_cosmos(packets, target=target)
        on_disk = {path.name: path.read_text(encoding="utf-8") for path in _defs(target)}
        assert sorted(on_disk) == [
            f"{target.lower()}_cmd_def.txt",
            f"{target.lower()}_tlm_def.txt",
        ], f"{target}: unexpected files in cmd_tlm"
        assert on_disk[f"{target.lower()}_cmd_def.txt"] == cmd_text
        assert on_disk[f"{target.lower()}_tlm_def.txt"] == tlm_text


def test_truth_defs_are_their_own_golden(eds: Eds) -> None:
    """The truth target's single def, same rule - and still exactly one file.

    The missing command definition is the tripwire (the truth side has no
    actuators), so "no cmd def here" is part of the assertion, not an omission.
    """
    for target in TRUTH_TARGETS:
        text = emit_cosmos_truth(eds.leaf(TRUTH_CONTAINER, TRUTH_PKG), target=target)
        on_disk = {path.name: path.read_text(encoding="utf-8") for path in _defs(target)}
        assert list(on_disk) == [f"{target.lower()}_tlm_def.txt"], "the truth side has no cmd def"
        assert next(iter(on_disk.values())) == text


def test_the_c_tables_are_theirs_too(eds: Eds) -> None:
    """The beacon table and the command dictionary are flight software: same rule.

    A stale table means a packet the ground never sees, or a command the
    dictionary refuses at upload - each of which costs a whole acceptance run to
    discover the slow way.
    """
    emitted = mission_packets(eds.packets())
    sha = _cfs_sha(CFS)
    assert C_TABLE.read_text(encoding="utf-8") == emit_c_table(emitted, sha)
    assert CMD_TABLE.read_text(encoding="utf-8") == emit_c_cmd_table(emitted, sha)


def test_emitting_twice_gives_the_same_bytes(packets: list[Packet]) -> None:
    """No clock inside the emitters, proven the cheapest possible way."""
    assert emit_cosmos(packets) == emit_cosmos(packets)
    assert emit_c_table(packets, "088b2fa") == emit_c_table(packets, "088b2fa")


def test_no_generated_artifact_records_a_clock() -> None:
    """Every surface we fly or decode with, scanned for a timestamp.

    Not tidiness: a file that changes when nothing changed makes the diffs this
    project treats as evidence worthless, and it is what would make the
    "C tables changed - rebuild the mission" warning in `scripts/regen_defs.sh` a
    coin flip instead of a fact.
    """
    generated = [path for target in (*FLIGHT_TARGETS, *TRUTH_TARGETS) for path in _defs(target)]
    generated += [C_TABLE, CMD_TABLE, *sorted(INC.glob("*.h"))]
    stamped = [
        str(path.relative_to(REPO)) for path in generated if CLOCK.search(path.read_text("utf-8"))
    ]
    assert not stamped, f"generated files that record when they were made: {stamped}"
