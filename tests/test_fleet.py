"""Fleet addressing tests (D9 spine + D10 derivation): port blocks and names.

The binding contract is the FIRST test: instance 0 must be every pre-fleet
port exactly, because every frozen trajectory digest was flown on it and the
fleet promises physics-neutrality. The D10 contract: a scenario's vehicles
are *derived* - block index is the port slot, the name is the COSMOS
target - and the hand-authored ground map (plugin.txt) must agree.
"""

from __future__ import annotations

import math
import re
import tomllib
from pathlib import Path

import pytest

from sim.fleet import (
    Vehicle,
    flight_interface,
    ports,
    sat_target,
    scenario_spacecraft,
    scenario_vehicles,
    truth_interface,
    truth_target,
)

REPO = Path(__file__).resolve().parent.parent
TWIN_OPS = REPO / "sim" / "scenarios" / "twin_ops.toml"


def test_instance_zero_is_the_legacy_port_set() -> None:
    """Ports the frozen digests (25fc027b..., 4688fc5d...) were flown on.

    The twin fleet (D10) adds its own frozen pair, flown twice by
    ``scripts/twin_roundtrip.sh``'s anchored 180 s choreography: chief
    01c03187..., deputy 85241e3a... At a 120 s twin flight chief's digest
    was 25fc027b... - the solo one: vehicles share an environment, never
    physics.
    """
    p = ports(0)
    assert (p.beacon, p.uplink) == (5050, 5051)
    assert p.sensor_endpoint == "tcp://*:5580"
    assert p.link_cmd_endpoint == "tcp://*:5581"
    assert p.truth_endpoint == "tcp://*:5590"
    assert p.echo_endpoint == "tcp://*:5591"
    assert p.echo_dial == "tcp://127.0.0.1:5591"
    assert p.truth_tcp == 5150  # P5: truth lands beside, never among


def test_port_blocks_never_collide_across_the_fleet() -> None:
    """Two vehicles sharing a port is a fleet bug, not a misconfiguration."""
    seen: dict[int, str] = {}
    for n in range(32):
        p = ports(n)
        for field in (
            "beacon",
            "uplink",
            "sensor",
            "link_cmd",
            "truth",
            "echo",
            "truth_tcp",
        ):
            value = getattr(p, field)
            label = f"{field}@{n}"
            assert value not in seen, f"{label} collides with {seen[value]}"
            seen[value] = label


def test_negative_instance_is_a_loud_error() -> None:
    """No silent modulo: a negative fleet index is a caller bug."""
    with pytest.raises(ValueError, match="instance"):
        ports(-1)


def test_truth_targets_derive_and_never_share_a_namespace() -> None:
    """Every flight target derives exactly one truth target (P5).

    A derived rule, not a config field - and the two sides can never
    name-clash, for any fleet the scenarios define.
    """
    assert truth_target("SAT_CHIEF") == "TRUTH_CHIEF"
    assert truth_target("SAT_DEPUTY") == "TRUTH_DEPUTY"
    flight = {v.cosmos_target for v in scenario_vehicles(TWIN_OPS)}
    truth = {truth_target(t) for t in flight}
    assert flight.isdisjoint(truth), "a truth target must never BE a flight target"


# ---------------------------------------------------------------------------
# D10: the fleet derives from the scenario - no hand-declared identity.


def test_sat_target_derives_from_the_spacecraft_name() -> None:
    """One rule, no field: the name IS the target."""
    assert sat_target("chief") == "SAT_CHIEF"
    assert sat_target("deputy") == "SAT_DEPUTY"
    flight = {sat_target(n) for n in ("chief", "deputy")}
    assert flight.isdisjoint({truth_target(t) for t in flight})


def test_scenario_vehicles_derive_the_fleet_in_block_order() -> None:
    """Block order is the frequency plan; the name is the identity."""
    vehicles = scenario_vehicles(REPO / "sim" / "scenarios" / "chief.toml")
    assert vehicles == [
        Vehicle(
            name="chief",
            instance=0,
            cosmos_target="SAT_CHIEF",
            truth_target="TRUTH_CHIEF",
        )
    ]
    assert vehicles[0].ports.beacon == 5050  # block 0 is the legacy block
    assert scenario_spacecraft(REPO / "sim" / "scenarios" / "detumble_forge.toml") == ["w0"]


def test_port_base_shifts_slots_never_identities() -> None:
    """Two scenario fleets on one box: the WHOLE fleet moves, names stay."""
    base = scenario_vehicles(REPO / "sim" / "scenarios" / "twin_ops.toml")
    shifted = scenario_vehicles(REPO / "sim" / "scenarios" / "twin_ops.toml", port_base=2)
    assert [(v.name, v.cosmos_target) for v in shifted] == [(v.name, v.cosmos_target) for v in base]
    assert [v.instance for v in shifted] == [2, 3]
    assert shifted[0].ports.beacon == 5054  # 5050 + 2*2, the block arithmetic
    assert shifted[1].ports.sensor_endpoint == "tcp://*:5640"  # 5580 + 20*3


def test_interface_names_follow_the_plugin_convention() -> None:
    """R3 derived these names from the hand-authored map: pin the rule."""
    base = scenario_vehicles(REPO / "sim" / "scenarios" / "twin_ops.toml")
    assert [(flight_interface(v), truth_interface(v)) for v in base] == [
        ("GS_TCP_INT", "TRUTH_CHIEF_INT"),
        ("GS_TCP2_INT", "TRUTH_DEPUTY_INT"),
    ]


def test_plugin_map_agrees_with_the_derived_fleet() -> None:
    """The scenario file and the ground plugin must say the same thing.

    plugin.txt is hand-authored ground config (interfaces dial fixed
    ports); this pin is what turns a renumbered, renamed, or reordered
    scenario without a plugin edit into a test failure - instead of
    telemetry silently arriving under the wrong target. The plugin is
    ERB-templated, so resolve its VARIABLEs before parsing, and route
    each interface's ports through its MAP_TARGET line.
    """
    text = (REPO / "cosmos_plugin" / "plugin.txt").read_text()
    variables = dict(re.findall(r"^VARIABLE (\w+) (\S+)", text, re.MULTILINE))
    resolved = re.sub(r"<%=\s*(\w+)\s*%>", lambda m: variables.get(m.group(1), ""), text)
    target_ports: dict[str, tuple[int, int]] = {}
    current: tuple[int, int] | None = None
    for line in resolved.splitlines():
        iface = re.match(r"INTERFACE \S+ \S+ \S+ (\d+) (\d+)", line)
        if iface:
            current = (int(iface.group(1)), int(iface.group(2)))
            continue
        mapped = re.match(r"\s+MAP_TARGET (\S+)", line)
        if mapped and current is not None:
            target_ports[mapped.group(1)] = current
    for scenario in ("chief.toml", "twin_ops.toml"):  # the D10 fleet file too
        for vehicle in scenario_vehicles(REPO / "sim" / "scenarios" / scenario):
            assert vehicle.cosmos_target in target_ports, f"{vehicle.cosmos_target} unmapped"
            assert vehicle.truth_target in target_ports, f"{vehicle.truth_target} unmapped"
            block = vehicle.ports
            # flight iface: write = uplink, read = beacon
            assert target_ports[vehicle.cosmos_target] == (block.uplink, block.beacon)
            # truth iface: one port, read+write both (the gateway is tlm-only)
            assert target_ports[vehicle.truth_target] == (block.truth_tcp, block.truth_tcp)


# --- orbital sanity (the 2026-11-22 lesson) ---------------------------------

_MU_EARTH = 3.986004418e14  # m^3/s^2, WGS-84 gravitational constant


def _rv_blocks() -> list[tuple[str, list[float], list[float]]]:
    """(file, position, velocity) for every declared rv orbit, raw TOML."""
    rows: list[tuple[str, list[float], list[float]]] = []
    for path in sorted((REPO / "sim" / "scenarios").glob("*.toml")):
        with path.open("rb") as fh:
            doc = tomllib.load(fh)
        for sc in doc.get("spacecraft", []):
            orbit = sc.get("orbit", {})
            if orbit.get("type") == "rv" and "rv" in orbit:
                rows.append((path.name, orbit["rv"]["position"], orbit["rv"]["velocity"]))
    return rows


def test_declared_orbits_orbit_and_do_not_fall() -> None:
    """Every rv block must fly near-circular at its radius - in SI.

    The fleet lived with velocity = [0, 7.5461, 0] for weeks: a km/s value
    in the m/s interface. BSK accepted it, gravity obeyed it, the vehicles
    free-fell, and nothing in CI noticed until p5's receiver-scale rule met
    a one-frame epoch skew (free-fall g x 1 s = 8.1 m/s dwarfs a 1 cm/s
    receiver). The schema cannot catch this - 7.5 m/s is a legal velocity -
    so the fleet file itself is checked against the orbit it claims.
    """
    rows = _rv_blocks()
    assert rows, "no rv orbits found - did the scenario format change?"
    for name, r, v in rows:
        r_norm = math.sqrt(sum(c * c for c in r))
        v_norm = math.sqrt(sum(c * c for c in v))
        circular = math.sqrt(_MU_EARTH / r_norm)
        assert 0.9 * circular <= v_norm <= 1.1 * circular, (
            f"{name}: |v|={v_norm:.1f} m/s at r={r_norm:.0f} m, but circular "
            f"speed is {circular:.1f} m/s - this vehicle is falling (or "
            "escaping); check the units: BSK is SI, metres per second"
        )
