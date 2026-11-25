"""Shared test posture: the world's first-tick peer gate must not gate tests.

The harness ackers dial before the rig ticks (their connections predate any
monitor), so an accept event can never arrive for them - and the gate exists
for the yard's real race (a world whose 10 s barrier deadline is shorter than
a craft boot), which offline tests do not fly. The suite therefore keeps the
gate SHORT; the tests in test_lockstep_fleet.py pin the gate's two real
verdicts with their own explicit wait times.
"""

from __future__ import annotations

import pytest


@pytest.fixture(autouse=True)
def _short_first_tick_gate(monkeypatch: pytest.MonkeyPatch) -> None:
    """Keep the default gate out of the suite's way: dialable, not dodged."""
    monkeypatch.setenv("ORRERY_FIRST_TICK_WAIT_S", "0.3")
