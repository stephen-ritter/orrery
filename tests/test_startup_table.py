"""The wake list is a scenario fact: derived, grammatical, and pinned.

The startup table is now a generated artifact (codegen/emit_startup.py,
docs/what_the_scenario_rules.md), so the tests read like every other
generated surface: the generator agrees with the hand-written fallback it
displaces, its records obey the parser's grammar, payload authority wakes
only where a scenario mounts the part that brings it, entailments refuse
loudly, and the yard's landed tables are exactly what the door would land.
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest

from codegen.emit_startup import (
    StartupDoorError,
    check_yard,
    load_apps,
    render_rows,
    select_apps,
    startup_table,
    table_path,
    write_yard,
)
from sim.parts import REPO, load_registry

SCENARIOS = REPO / "sim" / "scenarios"
DEFS = REPO / "cfs_mission" / "mission" / "orrery_defs"


def _tokens(table: str) -> list[tuple[str, ...]]:
    """The semantic view of a table: fields, whitespace invisible (as ES sees)."""
    return [
        tuple(field.strip() for field in line.rstrip(";").split(",")) for line in table.splitlines()
    ]


def _cmake_default() -> list[tuple[str, ...]]:
    """The build-time fallback, read from its two source files."""
    cmake = (DEFS / "generate_startup.cmake").read_text(encoding="utf-8")
    always = [
        tuple(field.strip() for field in line.split(","))
        for line in re.findall(r'"(CFE_[A-Z_]+, [^"]*?);\\n"', cmake)
        if "${" not in line  # the tail loop's template line is not a record
    ]
    order = re.search(r"set\(CFS_APP_STARTUP_ORDER ([^)]+)\)", cmake)
    assert order, "generate_startup.cmake lost its startup-order list"
    targets = (DEFS / "targets.cmake").read_text(encoding="utf-8")
    built = " ".join(re.findall(r"(?i)APPEND MISSION_GLOBAL_APPLIST ([^\n)]+)", targets)).split()
    tail = [
        ("CFE_APP", app, f"{app.upper()}_AppMain", app.upper(), "70", "131072", "0x0", "0")
        for app in order.group(1).split()
        if app in built
    ]
    # the always rows carry their entry type inside the quoted line; the tail
    # records gain it here, the way the cmake loop writes it
    return always + tail


# --- the union and its grammar ------------------------------------------------


def test_the_union_equals_the_build_time_fallback() -> None:
    """The image's own table and the generator's all-parts union are one list.

    Two sources of one grammar (generate_startup.cmake installs what the
    hand-run yard boots; emit_startup derives what the door lands), checked
    against each other - move registry/apps.toml without moving the cmake
    file and this names both.
    """
    apps, parts = load_apps(), load_registry()
    union = _tokens(startup_table(apps, parts, None, None))
    fallback = _cmake_default()
    assert union == fallback, "registry/apps.toml and generate_startup.cmake drifted"


def test_union_records_obey_the_startup_grammar() -> None:
    """ES: eight comma-separated fields per record, ';' ends, '!' ends the file.

    No comments exist down there - so no field may smuggle a delimiter, and
    the generated table must survive the parser it was written for.
    """
    apps, parts = load_apps(), load_registry()
    records = render_rows(select_apps(apps, parts, None, None))
    assert records
    for line in records:
        assert line.endswith(";")
        assert "!" not in line
        fields = [field.strip() for field in line.rstrip(";").split(",")]
        assert len(fields) == 8, line
        assert fields[0] in ("CFE_APP", "CFE_TASK", "CFE_LIB", "CFE_DRIVER")
        assert all(fields[1:5]), line
        assert fields[5].isdigit() and fields[7].isdigit(), line


# --- authority wakes only where it is mounted ---------------------------------


@pytest.mark.parametrize(
    ("scenario", "vehicle", "woken"),
    [
        ("camera_ops.toml", "chief", True),
        ("camera_record.toml", "chief", True),
        ("camera_downlink.toml", "chief", True),
        ("chief.toml", "chief", False),
        ("twin_ops.toml", "chief", False),
        ("twin_ops.toml", "deputy", False),
    ],
)
def test_the_camera_authority_wakes_with_the_eye(scenario: str, vehicle: str, woken: bool) -> None:
    """[[camera]] in the scenario is the only thing that puts cam on the bus.

    Absent here means the core is told never to start it - not started and
    silent. And the difference must be exactly one app: nothing else may
    ride along with the declaration, in either direction.
    """
    apps, parts = load_apps(), load_registry()
    table = _tokens(startup_table(apps, parts, SCENARIOS / scenario, vehicle))
    names = [record[1] for record in table]
    assert ("cam" in names) is woken
    union = [
        record for record in _tokens(startup_table(apps, parts, None, None)) if record[1] != "cam"
    ]
    without_cam = [record for record in table if record[1] != "cam"]
    assert without_cam == union  # the same boot order, minus or plus exactly one authority


def test_an_unmounted_authority_is_a_refused_vehicle_not_a_broken_one() -> None:
    """A vehicle that names nothing payload still boots the full plumbing."""
    apps, parts = load_apps(), load_registry()
    table = _tokens(startup_table(apps, parts, SCENARIOS / "chief.toml", "chief"))
    names = [record[1] for record in table]
    for plumbing in ("bsk_link", "gs_tcp", "fcs", "cf_tcp", "cf"):
        assert plumbing in names, plumbing


def test_unknown_vehicle_names_the_door_not_a_stack_trace(tmp_path: Path) -> None:
    """A table for a vehicle the scenario never flies is a named refusal."""
    scenario = tmp_path / "solo.toml"
    scenario.write_text('[[spacecraft]]\nname = "chief"\n', encoding="utf-8")
    apps, parts = load_apps(), load_registry()
    with pytest.raises(StartupDoorError, match="no \\[\\[spacecraft\\]\\]"):
        startup_table(apps, parts, scenario, "ghost")


# --- entailments refuse loudly ------------------------------------------------

SYNTHETIC_APPS = """
[[app]]
name = "aardvark"
class = "CFE_APP"
entry = "AARDVARK_AppMain"
short = "AARDVARK"
priority = 50
stack = 32768
when = "baseline"

[[app]]
name = "bee"
class = "CFE_APP"
entry = "BEE_AppMain"
short = "BEE"
priority = 60
stack = 32768
when = "declared"
requires = ["crow"]

[[app]]
name = "crow"
class = "CFE_APP"
entry = "CROW_AppMain"
short = "CROW"
priority = 61
stack = 32768
when = "declared"
"""

SYNTHETIC_PARTS = """
[[part]]
name = "buzz"
display = "a part that brings the bee"
bskforge_type = "flower"
zmq_topic = "BUZZ"
eds_container = "LINK/Buzz"
container_bytes = 8
topic_offset = 300
topic_name = "LINK_BUZZ_TLM_TOPICID"
mid_name = "LINK_BUZZ_TLM"
consumer = "bsk_link"
fsw_apps = ["bee"]

[[part]]
name = "flock"
display = "a part that brings the crow"
bskforge_type = "tree"
zmq_topic = "FLOCK"
eds_container = "LINK/Flock"
container_bytes = 8
topic_offset = 301
topic_name = "LINK_FLOCK_TLM_TOPICID"
mid_name = "LINK_FLOCK_TLM"
consumer = "bsk_link"
fsw_apps = ["crow"]
"""


def _synthetic(tmp_path: Path) -> tuple[Path, Path, Path]:
    apps = tmp_path / "apps.toml"
    apps.write_text(SYNTHETIC_APPS, encoding="utf-8")
    parts = tmp_path / "parts.toml"
    parts.write_text(SYNTHETIC_PARTS, encoding="utf-8")
    scenario = tmp_path / "garden.toml"
    scenario.write_text(
        '[[spacecraft]]\nname = "chief"\n\n[[spacecraft.sensors]]\ntype = "flower"\n',
        encoding="utf-8",
    )
    return apps, parts, scenario


def test_entailments_refuse_with_a_sentence(tmp_path: Path) -> None:
    """A vehicle that asks for the bee without the crow hears the door, not a crash.

    "you asked for a service its wake list cannot serve" is the rung's whole
    failure class: a declaration that quietly half-wakes a payload.
    """
    apps_path, parts_path, scenario = _synthetic(tmp_path)
    apps, parts = load_apps(apps_path, load_registry(parts_path)), load_registry(parts_path)
    with pytest.raises(StartupDoorError, match=r"bee.*requires.*crow"):
        startup_table(apps, parts, scenario, "chief")


def test_the_same_vehicle_serves_when_the_entailment_is_mounted(tmp_path: Path) -> None:
    """Mount the crow alongside the bee and the same door wakes both, quietly."""
    apps_path, parts_path, scenario = _synthetic(tmp_path)
    scenario.write_text(
        scenario.read_text(encoding="utf-8") + '\n[[spacecraft.sensors]]\ntype = "tree"\n',
        encoding="utf-8",
    )
    apps, parts = load_apps(apps_path, load_registry(parts_path)), load_registry(parts_path)
    names = [record[1] for record in _tokens(startup_table(apps, parts, scenario, "chief"))]
    assert "bee" in names and "crow" in names


def test_inventory_hygiene_catches_the_lying_registries(tmp_path: Path) -> None:
    """Declared apps no part brings, and parts that bring unknown apps, die at load."""
    apps_path, parts_path, _ = _synthetic(tmp_path)
    lonely = parts_path.with_name("parts_lonely.toml")
    lonely.write_text(
        SYNTHETIC_PARTS.replace('fsw_apps = ["crow"]', "fsw_apps = []"), encoding="utf-8"
    )
    with pytest.raises(StartupDoorError, match="no part brings"):
        load_apps(apps_path, load_registry(lonely))
    invented = parts_path.with_name("parts_invent.toml")
    invented.write_text(
        SYNTHETIC_PARTS.replace('fsw_apps = ["crow"]', 'fsw_apps = ["crow", "dragon"]'),
        encoding="utf-8",
    )
    with pytest.raises(StartupDoorError, match="inventory lacks"):
        load_apps(apps_path, load_registry(invented))


def test_reserved_grammar_characters_are_refused_by_the_inventory(tmp_path: Path) -> None:
    """An app name carrying ';' would rewrite a wake list from inside the registry."""
    apps_path, parts_path, _ = _synthetic(tmp_path)
    broken = apps_path.with_name("apps_broken.toml")
    broken.write_text(
        SYNTHETIC_APPS.replace('short = "BEE"', 'short = "BEE; EVIL"'),
        encoding="utf-8",
    )
    with pytest.raises(StartupDoorError, match="reserved"):
        load_apps(broken, load_registry(parts_path))


# --- the yard's landing -------------------------------------------------------

REAL_SCENARIO = SCENARIOS / "twin_ops.toml"


def test_write_yard_lands_every_vehicle_sweeps_every_ghost(tmp_path: Path) -> None:
    """Every vehicle of the scenario gets a table; every leftover from a dead one dies."""
    ghost = tmp_path / "cfe_es_startup.deceased.scr"
    ghost.write_text("CFE_APP, ghost, NOBODY, G, 1, 1, 0x0, 0;\n", encoding="utf-8")
    written = write_yard(tmp_path, REAL_SCENARIO)
    assert {path.name for path in written} == {
        "cfe_es_startup.chief.scr",
        "cfe_es_startup.deputy.scr",
    }
    assert not ghost.exists(), "a stale table from a dead scenario outlived the sweep"
    assert check_yard(tmp_path, REAL_SCENARIO) == []


def test_write_yard_is_idempotent(tmp_path: Path) -> None:
    """Re-landing a yard changes nothing - the generator's word is stable."""
    first = {path.name: path.read_bytes() for path in write_yard(tmp_path, REAL_SCENARIO)}
    second = {path.name: path.read_bytes() for path in write_yard(tmp_path, REAL_SCENARIO)}
    assert first == second


def test_check_yard_names_a_stale_table(tmp_path: Path) -> None:
    """A landed table that no longer matches the declaration is named, not flown."""
    write_yard(tmp_path, REAL_SCENARIO)
    stale = table_path(tmp_path, "chief")
    stale.write_text(
        stale.read_text(encoding="utf-8").replace(
            "gs_tcp, GS_TCP_AppMain", "gs_tcp, GS_TCP_AppMain, GHOST"
        ),
        encoding="utf-8",
    )
    problems = check_yard(tmp_path, REAL_SCENARIO)
    assert any("stale" in line for line in problems)


def test_check_yard_refuses_a_vehicle_booting_the_fallback(tmp_path: Path) -> None:
    """A missing table is the silent failure this door exists to refuse."""
    write_yard(tmp_path, REAL_SCENARIO)
    table_path(tmp_path, "deputy").unlink()
    problems = check_yard(tmp_path, REAL_SCENARIO)
    assert any("deputy" in line and "not landed" in line for line in problems)


# --- the door's contract, statically ------------------------------------------


def test_the_yard_door_lands_tables_and_waits_for_the_sentence() -> None:
    """fleet.sh writes the tables before `up` and demands the boot line.

    The runtime proof belongs to the live ladder
    (scripts/check_startup_config.sh); this is the cheap half: the door
    actually calls the generator, and a craft that boots the build default
    fails the wait instead of passing quietly.
    """
    door = (REPO / "scripts" / "fleet.sh").read_text(encoding="utf-8")
    assert "codegen.emit_startup" in door and "--write" in door
    assert "startup table: scenario door" in door
    boot = (REPO / "cfs_mission/mission/orrery_defs/cpu1/container-start").read_text(
        encoding="utf-8"
    )
    assert "ORRERY_VEHICLE" in boot and "startup table:" in boot
    compose = (REPO / "compose.yml").read_text(encoding="utf-8")
    assert compose.count("ORRERY_VEHICLE:") == 2, "every craft service names its table"
