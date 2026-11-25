"""The fleet's load rule: hardware is declared by naming a type.

A fleet file that wires hardware in by hand — bskforge's ``[custom]`` section, or
a device's free-form ``params`` table — describes a thing no schema names. Upstream
both are good design: the hatch is how a mission uses a Basilisk capability before
the schema grows a block for it, and it is how a prototype becomes a declared type
(``gpsReceiver`` came that way, and it is now one of our cleanest rows). Here they
are the one door through which hardware reaches the plant that nothing grades.

:func:`load_fleet` is the single door this repository loads a manifest through.
bskforge's loader enforces the rule (``profile="audited"`` — a profile exists
precisely so the library keeps the hatch for everyone else); what this module adds
is the half only we can say, because it is about *our* layers: a smuggled device
gets no registry row, is never declared in the fit manifest, cannot be classified
in the coverage ledger, earns no generated identity, and has no exhibit the truth
check could judge it by. The digest ledger would then pin a trajectory partly
produced by Python no schema ever saw.

A check that only forbids gets routed around by someone with a good reason, and
usually they have one: the device belongs in bskforge's schema. So the refusal
carries the route, not just the prohibition.
"""

from __future__ import annotations

from pathlib import Path

from bskforge.errors import AuditError
from bskforge.loader import load_scenario
from bskforge.profiles import AUDITED
from bskforge.schema.root import ScenarioConfig


class FleetPolicyError(RuntimeError):
    """A fleet file asked for wiring instead of declared hardware."""


_COST = (
    "Why this is a law and not a style rule: a part that is not a named schema "
    "type is not a part, it is wiring, and nothing downstream can see it. No "
    "registry row names it; the fit manifest never declares it, so the boot "
    "handshake cannot report a channel it was never told about; the coverage "
    "ledger cannot classify it; no ground defs carry it; and the truth check has "
    "no exhibit that could judge its numbers."
)

_ROUTE = (
    "Route it the way gpsReceiver came: grow the type in the bskforge repo "
    "(schema entry and builder, with its own tests), then land "
    "a row here - registry/parts.toml, the container's byte contract, a provider, "
    "an exhibit (docs/adding_a_sensor.md has the commands in order). A "
    "demonstration of the capability belongs upstream as an example; the fleet is "
    "the thing being audited."
)


def load_fleet(path: str | Path) -> ScenarioConfig:
    """Load a fleet manifest through bskforge's audited profile.

    This is the only place the repository asks bskforge for a scenario:
    ``tests/test_fleet_policy.py`` fails if a direct ``load_scenario`` reappears
    under ``sim/``, which is what turns "we check the manifest law" into "a
    manifest cannot get into a world without being checked".

    Args:
        path: the fleet or scenario TOML.

    Returns:
        The validated config, ready to build a plant from.

    Raises:
        FleetPolicyError: the manifest wires hardware in by hand. The message
            keeps bskforge's list of offending paths and adds what each one
            would hide from here, plus the route that is allowed.
        ConfigError: anything else wrong with the file. That sentence is
            bskforge's to say, and it says it with did-you-mean hints, so it is
            passed through rather than rewritten by us.
    """
    try:
        return load_scenario(path, profile=AUDITED)
    except AuditError as exc:
        raise FleetPolicyError(f"{exc}\n\n{_COST}\n\n{_ROUTE}") from exc
