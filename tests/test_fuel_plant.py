"""Fuel rung, world side: a declared tank prices the burn, dry means stopped.

The plant question these tests ride (no flight software anywhere): the
forge-wired tank drains NATIVELY as the effector fires, the world's fuel
door reads that stock and buys only what it can afford - squeezing an ask
that overshoots the stock, withholding whole ticks once it is dry - and the
truth ledger reports the tank's own books. The craft's claim ledger is a
later reader's business; here the divergence to prove is only that the
WORLD goes silent when the fuel does.

Flying tests build the plant without a rig (the burn door's pattern) on the
QUIET bench plant plus a declared tank: born-still, attitude-locked, so a
+Y-body quota is pure along-track and the fuel arithmetic is exact to the
printed digit. Prices use steadyIsp (Basilisk's own conversion); the native
drain can run slightly under the linear price while pulse ramps shape the
short burns, so drain assertions carry that tolerance and the EXACT
assertions live where the model itself is the authority (dry is exactly 0).
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np
import pytest
from bskforge import load_scenario

from codegen.eds_sources import CFS
from sim import forge_scenario as fz
from sim.eds_link import BurnCommand, _lib, available, entry
from sim.lockstep import Truth, truth_bytes

pytestmark = [
    pytest.mark.skipif(not CFS.is_dir(), reason="cFS is not fetched: run scripts/fetch_cfs.sh"),
    pytest.mark.skipif(not available(), reason="run edslink/build.sh to build the EDS database"),
]

BURN_TOML = fz.DEFAULT_SCENARIO.parent / "burn.toml"

G0 = 9.80665
ISPS = 100.0  # the bench plant's Blank_Thruster default, measured off the factory


def _tank_scenario(tmp_path: Path, fuel_kg: float | None) -> Path:
    """The quiet bench plant with an optional declared tank.

    Same edits as the burn plant's quiet variant (a body frame that IS an
    inertial frame), plus fuel_tank bound to the thrusters effector. With
    fuel_kg None the plant stays exactly the Rung-1 vehicle: the fuel rows
    must then speak absence.
    """
    text = BURN_TOML.read_text().replace(
        "sigma_BN = [0.05, -0.08, 0.11]\nomega_BN_B = [0.05, -0.04, 0.03]",
        "sigma_BN = [0.0, 0.0, 0.0]\nomega_BN_B = [0.0, 0.0, 0.0]",
    )
    assert text != BURN_TOML.read_text(), "the quiet variant must actually edit the attitude block"
    if fuel_kg is not None:
        marker = "# --- sensing: the suite the FSW gets to know the universe through -----------"
        tank = f"[spacecraft.effectors.fuel_tank]\nfuel_mass = {fuel_kg}\n\n"
        text = text.replace(marker, tank + marker)
    path = tmp_path / ("burn_fuel.toml" if fuel_kg else "burn_quiet_notank.toml")
    path.write_text(text)
    return path


def _fly_body(path: Path) -> tuple[fz.ForgeBody, Any]:
    """Build the plant with no rig, initialize it, hand back the body + sim."""
    config = load_scenario(path)
    (sc_cfg,) = config.spacecraft
    built = fz.ScenarioRunner(config).build()
    ctx = built.context
    body = fz.ForgeBody(ctx, sc_cfg, control_period_s=float(config.simulation.dt.fsw))
    ctx.sim.InitializeSimulation()
    return body, ctx.sim


def _advance(sim: Any, to_s: float) -> None:  # noqa: ANN401 - Basilisk SimBaseClass
    from Basilisk.utilities import macros

    horizon = macros.sec2nano(to_s)
    while sim.TotalSim.CurrentNanos < horizon:
        sim.TotalSim.SingleStepProcesses()


def _ask(tick: int, force_n: float) -> BurnCommand:
    return BurnCommand(
        sim_time_s=float(tick),
        force_n=force_n,
        axis_b=(0.0, 1.0, 0.0),
        frame=tick + 1,
        burn_id=1,
    )


def _burn_for_ticks(
    body: fz.ForgeBody,
    sim: Any,  # noqa: ANN401 - Basilisk SimBaseClass
    first_tick: int,
    ticks: int,
    force_n: float,
) -> None:
    for tick in range(ticks):
        body.apply_burn(_ask(first_tick + tick, force_n))
        _advance(sim, first_tick + tick + 1.0)


# ---- the physics: fuel buys impulse, and the tank's books are the books --------


def test_generous_tank_drains_natively_while_the_door_buys(tmp_path: Path) -> None:
    """Ten ticks of 3 N on a 50 g tank: full service, honest native drain.

    30 N s of commanded impulse prices 30/(Isp g0) = 30.6 g at the factory
    Isp; the tolerance (25%) covers what the pulse ramps keep off the books
    on short on-times - the drain is graded as PHYSICS, the exact numbers
    live in the dry tests below. Starved and throttled stay zero because the
    stock never binds, and the orbit still rises like the fuel-free world:
    fuel changes the END of a burn's story, not the middle.
    """
    body, sim = _fly_body(_tank_scenario(tmp_path, 0.05))
    assert body.has_thrusters and body._fuel_out is not None

    _advance(sim, 10.0)
    fresh = body.truth()
    assert fresh.fuel_remaining_kg == pytest.approx(0.05, rel=1e-9), (
        "the unwritten tank message must read as its declared load, not empty"
    )
    assert fresh.fuel_spent_kg == pytest.approx(0.0, abs=1e-12)

    _burn_for_ticks(body, sim, first_tick=10, ticks=10, force_n=3.0)

    burned = body.truth()
    assert burned.burn_ticks_applied == 10.0
    assert burned.burn_impulse_applied_ns == pytest.approx(30.0, rel=1e-9)
    assert burned.fuel_ticks_starved == 0.0 and burned.fuel_ticks_throttled == 0.0
    price = 30.0 / (ISPS * G0)
    assert burned.fuel_spent_kg == pytest.approx(price, rel=0.25)
    assert burned.fuel_remaining_kg == pytest.approx(0.05 - burned.fuel_spent_kg, abs=1e-12)


def test_thin_tank_throttles_then_starves_and_the_world_goes_silent(
    tmp_path: Path,
) -> None:
    """The door's full life in six ticks: buy, squeeze, dry, withhold.

    A 0.5 g tank against a 3.06 g/tick ask cannot buy a single full tick.
    The first ticks squeeze to the affordable force, the model clamps the
    stock to exactly zero, and every later ask meets a starve: no write, no
    applied tick. The craft keeps sending frames while this happens; the
    world's silence against that stream is the divergence, and it is now
    physics, not arithmetic.
    """
    load = 0.0005
    body, sim = _fly_body(_tank_scenario(tmp_path, load))
    _advance(sim, 10.0)

    _burn_for_ticks(body, sim, first_tick=10, ticks=6, force_n=3.0)

    dry = body.truth()
    assert dry.fuel_remaining_kg == 0.0, "the empty tank reads exactly zero, not a residue"
    assert dry.fuel_spent_kg == pytest.approx(load, rel=1e-9)
    assert dry.fuel_ticks_throttled >= 1 and dry.fuel_ticks_starved >= 1
    assert dry.fuel_ticks_starved + dry.fuel_ticks_throttled >= 3
    assert dry.burn_ticks_applied == 6.0 - dry.fuel_ticks_starved
    # The fuel bought every N s of that, and no more than that: the burn is
    # FINITE - strictly less than the six full asks would have bought. The
    # exact conversion between impulse and stock is not asserted here on
    # purpose: the door prices a tick linearly (F/(Isp g0)) while the plant
    # charges by its own pulse physics, whose ramps and minimum on-times
    # move a short pulse either way. The exact books are the tank's own,
    # and they were pinned above: spent == load, remaining == 0.
    assert 0.0 < dry.burn_impulse_applied_ns < 6.0 * 3.0


def test_dry_stays_dry_the_world_never_thrusts_on_empty_credit(tmp_path: Path) -> None:
    """After the tank clamps at zero, later burns cannot revive the plant.

    The starve is a state, not a race: a long silence between asks (a closed
    quota, a fresh one opening) changes nothing. Applied stays frozen at what
    the fuel bought, and every fresh burn_id meets the same dry door.
    """
    load = 0.0005
    body, sim = _fly_body(_tank_scenario(tmp_path, load))
    _advance(sim, 10.0)
    _burn_for_ticks(body, sim, first_tick=10, ticks=2, force_n=3.0)
    first = body.truth()
    assert first.fuel_remaining_kg == 0.0

    _advance(sim, 40.0)  # the coast buys nothing back
    body.apply_burn(
        BurnCommand(sim_time_s=40.0, force_n=5.0, axis_b=(1.0, 0.0, 0.0), frame=41, burn_id=2)
    )
    _advance(sim, 41.0)
    later = body.truth()
    assert later.burn_ticks_applied == first.burn_ticks_applied
    assert later.burn_impulse_applied_ns == first.burn_impulse_applied_ns
    assert later.fuel_ticks_starved == first.fuel_ticks_starved + 1


def test_no_tank_declares_absence_not_zero_stock(tmp_path: Path) -> None:
    """The Rung-1 vehicle, re-read through the fuel rows: NaN says "no tank".

    A tankless plant that burns says "I have no fuel books" with NaN on the
    two mass rows (a zero would claim an empty tank - a different and wrong
    fact) while the event counts stay true zeros: it never starved a tick,
    because it never priced one.
    """
    body, sim = _fly_body(_tank_scenario(tmp_path, None))
    assert body._fuel_out is None

    _advance(sim, 10.0)
    fresh = body.truth()
    assert np.isnan(fresh.fuel_remaining_kg) and np.isnan(fresh.fuel_spent_kg)
    assert fresh.fuel_ticks_starved == 0.0 and fresh.fuel_ticks_throttled == 0.0

    _burn_for_ticks(body, sim, first_tick=10, ticks=3, force_n=3.0)
    burned = body.truth()
    assert burned.burn_ticks_applied == 3.0
    assert np.isnan(burned.fuel_remaining_kg)
    assert burned.fuel_ticks_starved == 0.0 and burned.fuel_ticks_throttled == 0.0


# ---- the wire: the fuel rows ride the truth frame bit-honestly ------------------


def test_truth_wire_carries_the_fuel_rows() -> None:
    """22 doubles out, the same 22 back, NaN and all.

    The frame is EDS-encoded; this is the pin that a fuel row cannot silently
    change size or order: struct unpack by NAME via the decoded entries, the
    same way every later reader will.
    """
    truth = Truth(
        sim_time_s=7.5,
        r_bn_n=(7e6, 0.0, 0.0),
        v_bn_n=(0.0, 7.5, 0.0),
        sigma_bn=(0.0, 0.0, 0.0, 1.0),
        omega_bn_b=(0.0, 0.0, 0.0),
        burn_impulse_applied_ns=42.5,
        burn_ticks_applied=20.0,
        burn_ticks_saturated=1.0,
        burn_id=3.0,
        fuel_remaining_kg=0.0125,
        fuel_spent_kg=float("nan"),
        fuel_ticks_starved=4.0,
        fuel_ticks_throttled=2.0,
    )
    frame = truth_bytes(truth)
    assert len(frame) == 176, "TruthState is 22 float64s"
    obj = entry("BSK_LINK/TruthState")(_lib().PackedObject(frame))
    assert float(obj["FuelRemainingKg"]()) == pytest.approx(0.0125, rel=1e-12)
    assert float(obj["FuelTicksStarved"]()) == 4.0
    assert float(obj["FuelTicksThrottled"]()) == 2.0
    assert np.isnan(float(obj["FuelSpentKg"]())), "NaN must survive the wire as a declaration"
    # Everything the burn rung already promised still rides unchanged.
    assert float(obj["BurnImpulseAppliedNs"]()) == pytest.approx(42.5, rel=1e-12)
