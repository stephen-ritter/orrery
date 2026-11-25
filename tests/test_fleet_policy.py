"""The manifest law has teeth: hardware is declared by naming a type.

The law used to be written in three documents and enforced by none of them. It is
enforced by the loader now — bskforge's ``profile="audited"`` — which is where it
belongs: the escape surface stays available to every other user of the library,
and this project asks for it closed. What is left to test here is ours:

* **there is no unaudited door.** Every manifest that becomes a world comes
  through :func:`sim.fleet_policy.load_fleet`, so "was that file checked?" has
  exactly one answer. When the check was a separate call the failure mode was a
  second call site forgetting it; now it is a direct call to bskforge, which is
  why the codebase gets searched rather than trusted;
* **our half of the message.** bskforge names the offending paths; only we can
  say what a smuggled device would hide from here and which route is allowed. A
  check that only forbids gets routed around by someone with a good reason, and
  they usually have one.

The walk itself — depth, document paths, ``extends`` laundering, empty tables,
the typo'd profile — is upstream's to test, and is: ``bskforge/tests/test_profiles.py``.
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest

from sim.fleet_policy import FleetPolicyError, load_fleet

REPO = Path(__file__).resolve().parent.parent
FLEETS = sorted((REPO / "sim" / "scenarios").glob("*.toml"))
PRODUCTION = (REPO / "sim", REPO / "codegen", REPO / "scripts")
LOADER_CALL = re.compile(r"\b(load|validate)_scenario\(")

# Deliberately an incomplete scenario (no world, no hub): the audit runs before
# validation, so these probes prove the refusal is about the law, not about
# whatever pydantic happens to notice first.
BODY = """
[simulation]
name = "policy probe"
duration = 30.0

[[spacecraft]]
name = "probe"

[[spacecraft.sensors]]
type = "imuSensor"
rate = 1.0
"""

OFFENDERS = {
    "a root escape hatch": '[custom]\n[[custom.modules]]\nmodule = "spacecraft"\n',
    "an empty escape hatch (intent is still wiring)": "[custom]\n",
    "wiring on a task": '[[custom.tasks]]\nname = "extra"\nrate = 0.01\n',
    "a nested parameter table": """
[[spacecraft.effectors]]
type = "reactionWheels"

[spacecraft.effectors.params]
someBasiliskKnob = 0.01
""",
    "a sensor reaching past its schema": """
[spacecraft.sensors.params]
writeToDynamics = true
""",
}


def _write(tmp_path: Path, extra: str = "", body: str = BODY) -> Path:
    path = tmp_path / "probe.toml"
    path.write_text(f"{body}\n{extra}", encoding="utf-8")
    return path


def test_the_fleets_we_fly_load_through_the_audited_door() -> None:
    """The law describes the fleet, or the law is decoration.

    Loading the real manifests is also the receipt that the audit objects to
    nothing we fly: `docs/coverage.md`'s `banned` row is only honest while this
    passes, which is why it reads the checked-in files rather than a fixture.
    """
    assert FLEETS, "no fleet files found - this test would pass vacuously"
    for fleet in FLEETS:
        config = load_fleet(fleet)
        assert config.spacecraft, f"{fleet} loads with no vehicles"


@pytest.mark.parametrize("why", list(OFFENDERS), ids=list(OFFENDERS))
def test_wiring_is_refused_at_every_depth(tmp_path: Path, why: str) -> None:
    """Root `[custom]`, a nested custom table, and `params` under a device block.

    Depth matters because `params` lives inside the device blocks, which is
    exactly where a knob gets stretched: a refusal that only looked at the root
    would pass the case the law exists to catch.
    """
    with pytest.raises(FleetPolicyError):
        load_fleet(_write(tmp_path, OFFENDERS[why]))


def test_the_failure_names_the_path_a_human_must_edit(tmp_path: Path) -> None:
    """A path is a fix; "params is forbidden" on its own is a puzzle."""
    path = _write(
        tmp_path,
        """
[[spacecraft.effectors]]
type = "reactionWheels"

[spacecraft.effectors.params]
someBasiliskKnob = 0.01
""",
    )
    with pytest.raises(FleetPolicyError) as exc:
        load_fleet(path)

    message = str(exc.value)
    assert "spacecraft[0].effectors[0].params" in message, message
    assert "probe.toml" in message, message


def test_the_failure_teaches_instead_of_only_forbidding(tmp_path: Path) -> None:
    """The honest failure of this check is a bskforge issue, not a deleted check.

    So the pedagogy is pinned: what the smuggled part hides from (the fit
    manifest is the one that bites at boot), the precedent that worked
    (`gpsReceiver`), the repo that owns the fix, and the recipe's commands.
    """
    path = _write(tmp_path, '[custom]\n[[custom.modules]]\nmodule = "spacecraft"\n')
    with pytest.raises(FleetPolicyError) as exc:
        load_fleet(path)

    message = str(exc.value)
    for taught in ("fit manifest", "gpsReceiver", "bskforge", "docs/adding_a_sensor.md"):
        assert taught in message, f"{taught!r} missing from the lesson:\n{message}"


def test_no_manifest_enters_a_world_without_the_audit() -> None:
    """The audited loader is the only door, enforced by searching the code.

    A guard that lives in a function can be skipped by a new caller; this is the
    version that cannot: exactly one production file may ask bskforge for a
    scenario at all, and it must ask for the audited profile.
    """
    callers = {
        str(path.relative_to(REPO))
        for folder in PRODUCTION
        for path in folder.rglob("*")
        if path.suffix in {".py", ".sh"}
        and path.is_file()
        and LOADER_CALL.search(path.read_text(encoding="utf-8"))
    }
    assert callers == {"sim/fleet_policy.py"}, f"doors into the loader: {sorted(callers)}"

    door = (REPO / "sim" / "fleet_policy.py").read_text(encoding="utf-8")
    assert "profile=AUDITED" in door, "the door must ask for the audited profile"


def test_the_plant_refuses_before_it_builds_anything(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Nothing is constructed when the law is broken: no graph, no Basilisk object.

    `ScenarioRunner` becomes a bomb, so this reports an ordering mistake rather
    than trusting one — the refusal has to stay cheap enough to sit on every load
    path, including the ones added next month.
    """
    import sim.forge_scenario as forge

    def _bomb(*_args: object, **_kwargs: object) -> None:
        raise AssertionError("a plant was built from an unaudited manifest")

    monkeypatch.setattr(forge, "ScenarioRunner", _bomb)
    path = _write(tmp_path, "[custom]\n")
    for build in (forge.ForgeScenario, forge.ForgeFleet):
        with pytest.raises(FleetPolicyError):
            build(path)


def test_acceptance_refuses_to_grade_a_smuggled_world(tmp_path: Path) -> None:
    """The check that reads a manifest by env var refuses it too.

    `ORRERY_SCENARIO` lets one unmodified truth check judge two fleets, which
    also means a scratch manifest can be pointed at acceptance without touching
    the plant. Grading such a world would put an unaudited wire inside an
    exhibit, so the check reads manifests through the same door the plant does.
    """
    from sim.truth_exhibits import declared_sigmas

    path = _write(
        tmp_path,
        """
[spacecraft.sensors.params]
writeToDynamics = true
""",
    )
    with pytest.raises(FleetPolicyError):
        declared_sigmas(path)
