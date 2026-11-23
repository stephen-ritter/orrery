"""The FSW tick has one owner: the scenario's ``[simulation.dt] fsw``.

The rig's period used to default to 1.0 in Python while the TOML separately
declared the fswTask rate; the two dials could disagree silently (a 0.5 s
tick is a legal multiple of a 0.1 s scan grid, so the rig's own guard let it
through), and a world gating between sensor-frame boundaries is how stale
acks are born. Resolution now lives in one pure function, tested here
without building a sim: the TOML owns the number, and an explicit
``--control-period`` must match it exactly.

Digest invariance starts with the first test: every shipped scenario still
ticks at 1.0 s, so every anchored baseline must print unchanged.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from sim.fleet_policy import load_fleet
from sim.forge_scenario import ForgeError, _resolve_control_period

SCENARIOS = Path(__file__).resolve().parents[1] / "sim" / "scenarios"
CHIEF = SCENARIOS / "chief.toml"
TWIN = SCENARIOS / "twin_ops.toml"


def test_shipped_scenarios_still_tick_at_one_second() -> None:
    """The binding changed the source of the number, not its value."""
    for path in (CHIEF, TWIN):
        assert _resolve_control_period(load_fleet(path), None, path) == 1.0


def test_matching_override_is_accepted() -> None:
    """An override that agrees with the TOML is a no-op, not a fight."""
    assert _resolve_control_period(load_fleet(CHIEF), 1.0, CHIEF) == 1.0


def test_disagreeing_override_is_refused_naming_the_key() -> None:
    """The error teaches the fix: it points at the key that owns the rate."""
    with pytest.raises(ForgeError, match=r"\[simulation\.dt\]"):
        _resolve_control_period(load_fleet(CHIEF), 0.5, CHIEF)


def test_scenario_owns_the_number_it_declares(tmp_path: Path) -> None:
    """A scratch scenario at 2 Hz resolves to 2 Hz: the knob is real."""
    scratch = tmp_path / "chief_2hz.toml"
    scratch.write_text(CHIEF.read_text().replace("fsw = 1.0", "fsw = 0.5", 1))
    assert _resolve_control_period(load_fleet(scratch), None, scratch) == 0.5
