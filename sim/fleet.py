"""Fleet addressing (D9, refined by D10): port blocks and derived identity.

One scenario file is one world; every ``[[spacecraft]]`` block in it is one
vehicle, and NOTHING about a vehicle is hand-declared:

    instance       block index (block 0 is the legacy port set)
    beacon   5050 + 2N      uplink   5051 + 2N
    sensor   5580 + 20N     link cmd 5581 + 20N
    truth    5590 + 20N     echo     5591 + 20N
    truth TCP 5150 + 2N     target   SAT_<NAME>   truth   TRUTH_<SUFFIX>

Instance 0 is byte-identical to every pre-fleet port (the digest-safety
guarantee: ports are transport, never physics). The +20 stride preserves
each vehicle's internal offsets (+10 between the link pair and the
observability pair), and the ground stride is 2 so vehicle N owns exactly
two ground ports - the operator's mental model: four ports per vehicle,
two ground + two link, plus the two observability channels that never
cross to cFS.

The D9 manifest layer (fleet/birds/, the ``[bird]`` tag, hand-written
targets) is retired per D10: the scenario file IS the fleet, and
``scenario_vehicles()`` below is the derivation that replaced it.
"""

from __future__ import annotations

import argparse
import tomllib
from dataclasses import dataclass
from pathlib import Path

GROUND_BASE = 5050
LINK_BASE = 5580
# The truth ground block starts ABOVE anything the flight ground block can
# reach (5050 + 2*31 + 1 = 5113 for a 32-vehicle fleet) - the collision test
# over 32 instances is what turned "5070" from the whiteboard number into
# this one: beacon(10) would have shadowed truth(0).
TRUTH_GROUND_BASE = 5150
GROUND_STRIDE = 2
LINK_STRIDE = 20


@dataclass(frozen=True)
class Ports:
    """One vehicle's address plan: ground pair, link pair, observability pair."""

    beacon: int
    uplink: int
    sensor: int
    link_cmd: int
    truth: int
    echo: int
    truth_tcp: int

    @property
    def sensor_endpoint(self) -> str:
        """Bind-side endpoint for the sensor PUB (the tick)."""
        return f"tcp://*:{self.sensor}"

    @property
    def link_cmd_endpoint(self) -> str:
        """Bind-side endpoint for the link PULL (torque + barrier acks)."""
        return f"tcp://*:{self.link_cmd}"

    @property
    def truth_endpoint(self) -> str:
        """Bind-side endpoint for the truth PUB (ground only, never cFS)."""
        return f"tcp://*:{self.truth}"

    @property
    def echo_endpoint(self) -> str:
        """Bind-side endpoint for the applied-torque echo PUB."""
        return f"tcp://*:{self.echo}"

    @property
    def echo_dial(self) -> str:
        """Client-side endpoint: the sim binds, the ground dials localhost."""
        return f"tcp://127.0.0.1:{self.echo}"


def ports(instance: int) -> Ports:
    """The port block for one instance. Instance 0 is THE legacy port set."""
    if instance < 0:
        raise ValueError(f"instance must be >= 0, got {instance}")
    ground = GROUND_BASE + GROUND_STRIDE * instance
    link = LINK_BASE + LINK_STRIDE * instance
    return Ports(
        beacon=ground,
        uplink=ground + 1,
        sensor=link,
        link_cmd=link + 1,
        truth=link + 10,
        echo=link + 11,
        # P5: the TRUTH gateway's ground-side TCP (tlm-only, D2-shaped).
        # Its own block so a truth link can never shadow a flight port,
        # and the +20 ground stride keeps vehicle N's two ground pairs apart.
        truth_tcp=TRUTH_GROUND_BASE + GROUND_STRIDE * instance,
    )


def truth_target(cosmos_target: str) -> str:
    """The truth-side COSMOS target for a flight target.

    ``SAT_CHIEF -> TRUTH_CHIEF``: one naming rule, no declared
    field - the truth target is *derived* identity, always present, never
    editable, so a vehicle cannot forget it and cannot collide with its
    own flight side.
    """
    return "TRUTH_" + cosmos_target.removeprefix("SAT_")


def sat_target(spacecraft_name: str) -> str:
    """The COSMOS flight target for a spacecraft name (D10 derivation).

    ``"chief" -> "SAT_CHIEF"``, ``"deputy" -> "SAT_DEPUTY"``. bskforge
    already rejects duplicate spacecraft names inside a scenario, so this
    rule makes distinct blocks collide on nothing - the whole "distinct
    targets per concurrent vehicle" invariant falls out of the schema.
    """
    return "SAT_" + spacecraft_name.upper()


@dataclass(frozen=True)
class Vehicle:
    """One ``[[spacecraft]]`` block, addressed: everything derived (D10)."""

    name: str
    instance: int  # block index, 0-based - the port slot
    cosmos_target: str
    truth_target: str

    @property
    def ports(self) -> Ports:
        """The vehicle's address block."""
        return ports(self.instance)


def scenario_spacecraft(path: str | Path) -> list[str]:
    """Spacecraft names of a scenario file, in block order.

    Reads the raw TOML (not bskforge's loader): launcher scripts, codegen
    and the collision tests must run on a laptop with no Basilisk, so
    this stays a ``tomllib`` read of one field. The scenario file itself
    validates everything else.
    """
    with Path(path).open("rb") as fh:
        raw = tomllib.load(fh)
    return [str(sc["name"]) for sc in raw.get("spacecraft", [])]


def scenario_vehicles(path: str | Path, port_base: int = 0) -> list[Vehicle]:
    """The full derived fleet of one scenario file (D10).

    Block order is the frequency plan: block 0 gets the legacy block. A
    reordered file reassigns ports on purpose - the plugin pin test fails
    the build when the ground map is not updated with it. ``port_base``
    shifts the WHOLE fleet's slot numbers (two scenario fleets on one box
    must not collide; the operational ground is always base 0, so a shifted
    fleet is sim-side only - the COSMOS plugin follows base-0 names).
    """
    return [
        Vehicle(
            name=name,
            instance=port_base + index,
            cosmos_target=sat_target(name),
            truth_target=truth_target(sat_target(name)),
        )
        for index, name in enumerate(scenario_spacecraft(path))
    ]


def flight_interface(vehicle: Vehicle) -> str:
    """The COSMOS interface name serving a vehicle's flight stream.

    The hand-authored plugin's convention, made a rule so acceptance
    clients and the plugin pin test share one spelling: the legacy block
    owns ``GS_TCP_INT``, later slots number from two (``GS_TCP2_INT``).
    """
    return "GS_TCP_INT" if vehicle.instance == 0 else f"GS_TCP{vehicle.instance + 1}_INT"


def truth_interface(vehicle: Vehicle) -> str:
    """The COSMOS interface name serving a vehicle's truth gateway (P5)."""
    return f"{vehicle.truth_target}_INT"


def add_link_args(parser: argparse.ArgumentParser) -> None:
    """Endpoint args for any sim entry point.

    ``--instance`` selects the port block; the individual endpoint flags
    (diagnostics, mostly) still win when given.
    """
    parser.add_argument(
        "--instance",
        type=int,
        default=0,
        help="fleet port block (D9): beacon 5050+2N, link farm 5580+20N",
    )
    parser.add_argument(
        "--sensor-endpoint", default=None, help="default: this instance's sensor port"
    )
    parser.add_argument(
        "--cmd-endpoint", default=None, help="default: this instance's link-cmd port"
    )
    parser.add_argument(
        "--truth-endpoint", default=None, help="default: this instance's truth port"
    )
    parser.add_argument("--echo-endpoint", default=None, help="default: this instance's echo port")


def resolve_endpoints(args: argparse.Namespace) -> Ports:
    """Fill the endpoint fields from args.instance where unset.

    Returns the block so callers can print/report it.
    """
    block = ports(args.instance)
    args.sensor_endpoint = args.sensor_endpoint or block.sensor_endpoint
    args.cmd_endpoint = args.cmd_endpoint or block.link_cmd_endpoint
    args.truth_endpoint = args.truth_endpoint or block.truth_endpoint
    args.echo_endpoint = args.echo_endpoint or block.echo_endpoint
    return block
