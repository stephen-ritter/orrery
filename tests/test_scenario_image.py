"""The build list is a scenario fact: cut by the wake list, pinned to the inventory.

Rung two of the source-of-truth ladder (docs/what_the_scenario_rules.md): the
manifest decides not only what the core boots but what the mission compiles.
The tests read like every other generated surface - the fragment equals the
fleet's union of wake lists, the eye appears only where a part mounted it (on
any vehicle), core-shipped organs sit outside the cut, cmake's all-parts
fallback stays equal to the inventory, the entailments refuse compilation with
the boot door's sentence, and the build-time wake table can only name what the
build actually compiled. The live half (scripts/check_scenario_image.sh) flies
the plane itself.
"""

from __future__ import annotations

import re
import subprocess
import sys
from pathlib import Path

import pytest

from codegen.emit_applist import (
    APPLIST_FRAGMENT,
    build_apps,
    render,
    sweep_fragment,
    write_fragment,
)
from codegen.emit_startup import StartupDoorError, load_apps
from sim.parts import REPO, load_registry

DEFS = REPO / "cfs_mission" / "mission" / "orrery_defs"
TARGETS = DEFS / "targets.cmake"
GEN_STARTUP = DEFS / "generate_startup.cmake"
SCENARIOS = REPO / "sim" / "scenarios"


def _names(scenario: str) -> list[str]:
    return build_apps(load_apps(), load_registry(), SCENARIOS / scenario)


# --- the cut follows the wake list ---------------------------------------------


def test_the_eye_is_built_only_for_scenarios_that_grew_it() -> None:
    """No eye declared anywhere in the fleet, no cam in the compile list."""
    assert "cam" in _names("camera_ops.toml")
    assert "cam" in _names("camera_downlink.toml")
    for bare in ("chief.toml", "twin_ops.toml"):
        assert "cam" not in _names(bare), f"{bare} built an organ nobody mounted"


def test_a_fleet_builds_what_any_vehicle_wakes(tmp_path: Path) -> None:
    """One plane per fleet: the eye on deputy is enough to build the eye.

    Heterogeneity never shrinks the build below the fleet (and never builds
    the same organ twice): chief flies without it, deputy carries it, the
    list names it once.
    """
    text = (SCENARIOS / "camera_ops.toml").read_text(encoding="utf-8")
    deputy_eye = tmp_path / "deputy_eye.toml"
    moved = text.replace('name = "chief"', 'name = "deputy"', 1).replace(
        'parent = "chief"', 'parent = "deputy"', 1
    )
    deputy_eye.write_text(moved, encoding="utf-8")
    assert "cam" in build_apps(load_apps(), load_registry(), deputy_eye)
    pair = tmp_path / "pair.toml"
    pair.write_text(text + '\n[[spacecraft]]\nname = "deputy"\n', encoding="utf-8")
    assert build_apps(load_apps(), load_registry(), pair).count("cam") == 1


# --- the inventory's own shape ---------------------------------------------------


def test_core_shipped_organs_are_outside_the_cut() -> None:
    """sch_lab and the assert lib answer for themselves on every image.

    The fragment may not name them (the core and the per-CPU lists build
    them); the scenario's cut is for mission organs only. The labs left
    the inventory in the 2026-11-27 prebaked purge - ci_lab and to_lab
    were apps the mission builds-or-not like any other, and they earned
    their retirement like everyone else: silence.
    """
    shipped = {row.name for row in load_apps() if row.core_shipped}
    assert shipped == {"cfe_assert", "sch_lab"}
    for scenario in ("chief.toml", "camera_ops.toml"):
        assert not shipped & set(_names(scenario))


def test_the_union_fallback_is_the_registry() -> None:
    """The hand-run yard's list and the inventory stay one fact.

    targets.cmake's fallback literal equals every mission-built row, and the
    core-shipped rows are exactly what the per-CPU list adds on top - move
    the inventory without moving the cmake file and this names both.
    """
    literal = re.search(r"set\(ORRERY_SCENARIO_APPLIST\s+([^)]+)\)", TARGETS.read_text("utf-8"))
    assert literal, "targets.cmake lost its all-parts fallback"
    rows = load_apps()
    assert set(literal.group(1).split()) == {row.name for row in rows if not row.core_shipped}
    cpu = re.search(r"SET\(cpu1_APPLIST ([^)]+)\)", TARGETS.read_text("utf-8"))
    assert cpu, "targets.cmake lost its per-CPU app list"
    assert {row.name for row in rows if row.core_shipped} - {"cfe_assert"} == set(
        cpu.group(1).split()
    )


# --- the door refuses what the boot door refuses ----------------------------------


def test_the_build_door_speaks_the_boot_door_sentence(tmp_path: Path) -> None:
    """A bee without its crow never reaches the compiler.

    The same refusal rung one wrote for the wake list, now upstream of cmake:
    what the manifest cannot serve, it must not compile.
    """
    apps = tmp_path / "apps.toml"
    apps.write_text(
        '[[app]]\nname = "bee"\nclass = "CFE_APP"\nentry = "BEE_AppMain"\nshort = "BEE"\n'
        'priority = 60\nstack = 32768\nwhen = "declared"\nrequires = ["crow"]\n\n'
        '[[app]]\nname = "crow"\nclass = "CFE_APP"\nentry = "CROW_AppMain"\nshort = "CROW"\n'
        'priority = 61\nstack = 32768\nwhen = "declared"\n',
        encoding="utf-8",
    )
    parts = tmp_path / "parts.toml"
    parts.write_text(
        '[[part]]\nname = "buzz"\ndisplay = "a part that brings the bee"\n'
        'bskforge_type = "flower"\nzmq_topic = "BUZZ"\neds_container = "LINK/Buzz"\n'
        'container_bytes = 8\ntopic_offset = 300\ntopic_name = "LINK_BUZZ_TLM_TOPICID"\n'
        'mid_name = "LINK_BUZZ_TLM"\nconsumer = "bsk_link"\nfsw_apps = ["bee"]\n\n'
        '[[part]]\nname = "flock"\ndisplay = "a part that brings the crow"\n'
        'bskforge_type = "tree"\nzmq_topic = "FLOCK"\neds_container = "LINK/Flock"\n'
        'container_bytes = 8\ntopic_offset = 301\ntopic_name = "LINK_FLOCK_TLM_TOPICID"\n'
        'mid_name = "LINK_FLOCK_TLM"\nconsumer = "bsk_link"\nfsw_apps = ["crow"]\n',
        encoding="utf-8",
    )
    scenario = tmp_path / "garden.toml"
    scenario.write_text(
        '[[spacecraft]]\nname = "chief"\n\n[[spacecraft.sensors]]\ntype = "flower"\n',
        encoding="utf-8",
    )
    with pytest.raises(StartupDoorError, match=r"bee.*requires.*crow"):
        build_apps(load_apps(apps, load_registry(parts)), load_registry(parts), scenario)


# --- the fragment itself ------------------------------------------------------------


def test_the_fragment_is_idempotent_and_swept_clean(tmp_path: Path) -> None:
    """Same scenario twice, byte for byte; a swept yard really is swept."""
    out = tmp_path / "mission_applist.cmake"
    scenario = SCENARIOS / "camera_ops.toml"
    write_fragment(scenario, path=out)
    once = out.read_text(encoding="utf-8")
    write_fragment(scenario, path=out)
    assert out.read_text(encoding="utf-8") == once
    assert once.startswith("# AUTO-GENERATED by codegen/emit_applist - DO NOT EDIT")
    assert "cam" in once
    sweep_fragment(out)
    assert not out.exists()
    sweep_fragment(out)  # sweeping a quiet yard is a no-op, not an error


def test_the_module_answers_when_the_door_calls_it(tmp_path: Path) -> None:
    """`python -m` must SPEAK: a gate-less module exits 0 in silence.

    A door that prints nothing when it ran is indistinguishable from one
    that never ran - fleet.sh trusts this line to exist and to mean it.
    """
    out = tmp_path / "mission_applist.cmake"
    done = subprocess.run(
        [
            sys.executable,
            "-m",
            "codegen.emit_applist",
            "--scenario",
            str(SCENARIOS / "camera_ops.toml"),
            "--out",
            str(out),
        ],
        capture_output=True,
        text=True,
        cwd=REPO,
    )
    assert done.returncode == 0, done.stderr
    assert "camera_ops.toml" in done.stdout
    assert out.exists()
    swept = subprocess.run(
        [sys.executable, "-m", "codegen.emit_applist", "--sweep", "--out", str(out)],
        capture_output=True,
        text=True,
        cwd=REPO,
    )
    assert swept.returncode == 0, swept.stderr  # --sweep alone IS a complete sentence
    assert not out.exists()


def test_the_fragment_names_no_scenario() -> None:
    """Two worlds that compile the same organs earn the same provenance.

    chief.toml and twin_ops.toml wake the same set, so their fragments agree
    byte for byte: the stamp is a content fact, never a name.
    """
    a = render(build_apps(load_apps(), load_registry(), SCENARIOS / "chief.toml"))
    b = render(build_apps(load_apps(), load_registry(), SCENARIOS / "twin_ops.toml"))
    assert a == b


# --- the build-time table cannot promise a missing organ -----------------------------


def test_the_build_table_can_only_wake_what_the_list_built() -> None:
    """Every mission-app record is presence-gated; the core's own lib is not.

    The grammar stays ES's (eight fields, quoted records); presence belongs
    to the build list. A cam record that could enter un-gated would promise
    a .so to a cut build that never compiled it - ES's boot failure waiting
    for the first hand-run yard.
    """
    cmake = GEN_STARTUP.read_text(encoding="utf-8")
    assert "list (FIND ARGN ${APP} SHOULD_START)" in cmake  # the gate's idiom
    gated = re.search(r"foreach \(APP ([^)]+)\)", cmake)
    assert gated, "the presence gate lost its app list"
    assert "cam" in gated.group(1).split()
    assert cmake.count('"CFE_APP, cam,') == 1  # one record, inside a LINE_ definition
    assert "LINE_cfe_assert" in cmake  # nothing decides whether the core's lib exists


# --- the stamp sees the list ------------------------------------------------------------


def test_the_landed_fragment_moves_the_provenance_stamp() -> None:
    """An image built for one list fails another's check - by hash, not memory.

    The fragment is untracked (it lives exactly as long as the yard), so the
    guard must see it: this asks scripts/firmware.sh itself, both ways, and
    leaves the tree exactly as it found it.
    """

    def _guard(verb: str) -> str:
        done = subprocess.run(
            ["bash", str(REPO / "scripts" / "firmware.sh"), verb],
            capture_output=True,
            text=True,
            check=True,
        )
        return done.stdout

    had = APPLIST_FRAGMENT.read_text(encoding="utf-8") if APPLIST_FRAGMENT.exists() else None
    try:
        write_fragment(SCENARIOS / "chief.toml")
        assert "mission_applist.cmake" in _guard("inputs")
        cut = _guard("hash")
        write_fragment(SCENARIOS / "camera_ops.toml")
        assert cut != _guard("hash"), "the stamp ignored a changed build list"
    finally:
        if had is not None:
            APPLIST_FRAGMENT.write_text(had, encoding="utf-8")
        else:
            APPLIST_FRAGMENT.unlink(missing_ok=True)
