"""Hohmann rung, world side: fuel budgets a transfer and the orbit keeps score.

The plant question these tests ride (no flight software, no containers - the
world and its tank, flying the yard's own burn door): a first leg buys a
measured apoapsis raise, coast delivers the spacecraft there, a second leg
at the DETECTED apoapsis collapses the eccentricity - and the fuel books
close on the printed numbers. Then the rung's other half: thin the tank and
the second leg is bought only PARTLY. The transfer strands mid-mission:
the orbit itself (not a counter) says the spacecraft never circularized,
while the yard gate's starved ticks count the refusal. Fuel earns its keep
HERE; on every rung before this one it was an accountant.

Numbers are MEASURED, not predicted (the same measurement flight that sized
the bench): leg 1 = 50 N x 36 s = 1800 N s -> apoapsis 7,042,659 m (a finite
burn arc, not the impulsive textbook value); leg 2 = 50 N x 34 s at the flip
-> e 3.0e-3 to 1.4e-4; prices ride the linear steadyIsp law, so fuel rows
are EXACT here (impulse/(Isp g0) to the printed digit) while the ORBIT
verdicts keep bands - J2 and burn shape live inside them.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

import numpy as np
import pytest
from bskforge import load_scenario

from codegen.eds_sources import CFS
from sim import forge_scenario as fz
from sim.eds_link import BurnCommand, _lib, available, entry
from sim.hohmann_probe import eccentricity, radial_rate

pytestmark = [
    pytest.mark.skipif(not CFS.is_dir(), reason="cFS is not fetched: run scripts/fetch_cfs.sh"),
    pytest.mark.skipif(not available(), reason="run edslink/build.sh to build the EDS database"),
]

HOHMANN_TOML = fz.DEFAULT_SCENARIO.parent / "hohmann.toml"

MU = 398600.4415e9
G0 = 9.80665
ISPS = 100.0
APOAPSIS_M = 7_042_659.0  # measured; band below covers detection +/- 1 tick


def _plant(tmp_path: Path, fuel_kg: float | None) -> Path:
    """The hohmann plant, optionally stripped of its tank (fuel_kg None)."""
    text = HOHMANN_TOML.read_text()
    if fuel_kg is not None:
        text = text.replace("fuel_mass = 5.5", f"fuel_mass = {fuel_kg}")
    else:
        block = "[spacecraft.effectors.fuel_tank]\nfuel_mass = 5.5\n\n"
        assert block in text
        text = text.replace(block, "")
    path = tmp_path / f"hohmann_{fuel_kg or 'notank'}.toml"
    path.write_text(text)
    return path


def _fly(path: Path) -> tuple[fz.ForgeBody, Any]:
    config = load_scenario(path)
    (sc_cfg,) = config.spacecraft
    built = fz.ScenarioRunner(config).build()
    ctx = built.context
    body = fz.ForgeBody(ctx, sc_cfg, control_period_s=float(config.simulation.dt.fsw))
    ctx.sim.InitializeSimulation()
    return body, ctx.sim


def _advance_to(sim: Any, to_s: float) -> None:  # noqa: ANN401 - Basilisk SimBaseClass
    from Basilisk.utilities import macros

    horizon = macros.sec2nano(to_s)
    while sim.TotalSim.CurrentNanos < horizon:
        sim.TotalSim.SingleStepProcesses()


def _truth_rv(truth: Any) -> tuple[list[float], list[float]]:  # noqa: ANN401 - Truth dataclass
    return list(truth.r_bn_n), list(truth.v_bn_n)


def _leg(
    body: fz.ForgeBody,
    sim: Any,  # noqa: ANN401 - Basilisk SimBaseClass
    first_tick: float,
    ticks: int,
    force_n: float,
    axis_b: tuple[float, float, float],
    burn_id: int,
) -> None:
    for k in range(ticks):
        t = first_tick + k
        body.apply_burn(
            BurnCommand(
                sim_time_s=t, force_n=force_n, axis_b=axis_b, frame=int(t) + 1, burn_id=burn_id
            )
        )
        _advance_to(sim, t + 1.0)


def _coast_to_apoapsis(
    body: fz.ForgeBody,
    sim: Any,  # noqa: ANN401 - Basilisk SimBaseClass
    from_s: float,
) -> tuple[float, float]:
    """Step until r-dot flips sign; return (apoapsis time, |r|max seen)."""
    prev: float | None = None
    r_max = 0.0
    t = from_s
    while t < from_s + 4000.0:
        t += 1.0
        _advance_to(sim, t)
        truth = body.truth()
        r, v = _truth_rv(truth)
        r_max = max(r_max, float(np.linalg.norm(r)))
        rdot = radial_rate(r, v)
        if prev is not None and prev > 0.0 >= rdot:
            return t, r_max
        prev = rdot
    raise AssertionError(f"no apoapsis inside 4000 s after t={from_s}")


def test_two_legs_bought_with_fuel_land_a_circular_orbit(tmp_path: Path) -> None:
    """The rung's full success story, on the yard's own door and tank.

    Leg 1 full authority (pair devices, no clip); MEASURED apoapsis within
    1% of the measurement flight; leg 2 at the flip collapses e; the books
    close EXACT (price == impulse/(Isp g0) is the gate's own arithmetic)
    with nothing throttled or starved - a transfer fuel can afford lands.
    """
    body, sim = _fly(_plant(tmp_path, 5.5))
    _advance_to(sim, 10.0)
    born = body.truth()
    r0, v0 = _truth_rv(born)
    assert eccentricity(r0, v0) < 1e-4, "born circular is a plant promise"

    _leg(body, sim, 10.0, 36, 50.0, (0.0, 1.0, 0.0), burn_id=1)
    after1 = body.truth()
    assert after1.burn_impulse_applied_ns == pytest.approx(1800.0, abs=1e-9)
    assert after1.burn_ticks_saturated == 0.0, "60 N of pair capability covers a 50 N ask"
    assert after1.fuel_spent_kg == pytest.approx(1800.0 / (ISPS * G0), abs=1e-9)

    apo_t, r_max = _coast_to_apoapsis(body, sim, 46.0)
    assert abs(r_max - APOAPSIS_M) <= 0.01 * APOAPSIS_M
    e_transfer = eccentricity(*_truth_rv(body.truth()))
    assert e_transfer > 2.5e-3, "on the ellipse: leg 2 has a real job to do"

    _leg(body, sim, apo_t, 34, 50.0, (0.0, -1.0, 0.0), burn_id=2)
    after2 = body.truth()
    assert after2.burn_impulse_applied_ns == pytest.approx(3500.0, abs=1e-9)
    r2, v2 = _truth_rv(after2)
    assert eccentricity(r2, v2) < 5e-4, "THE TRANSFER LANDED: the ellipse became a circle"
    assert after2.fuel_spent_kg == pytest.approx(3500.0 / (ISPS * G0), abs=1e-9)
    assert after2.fuel_remaining_kg == pytest.approx(5.5 - 3500.0 / (ISPS * G0), abs=1e-9)
    assert after2.fuel_ticks_starved == 0.0 and after2.fuel_ticks_throttled == 0.0

    # 240 s of confirmation coast: a circle holds its radius (an ellipse at
    # this e wanders kilometres over the same arc - see the stranded test).
    dr = 0.0
    r_ref = math.sqrt(sum(x * x for x in r2))
    for k in range(1, 241):
        _advance_to(sim, apo_t + 34.0 + k)
        r_now, _ = _truth_rv(body.truth())
        dr = max(dr, abs(math.sqrt(sum(x * x for x in r_now)) - r_ref))
    assert dr < 500.0, f"confirmation coast wandered {dr:.0f} m"


def test_a_thin_tank_strands_the_transfer_and_the_orbit_says_so(tmp_path: Path) -> None:
    """Fuel is the budget: afford one leg and the mission visibly completes half.

    A 2.5 kg tank buys leg 1 whole (1.836 kg) and leg 2 only partly: the
    survivors price ~13 of its 34 ticks. The gate throttles the tick that
    splits the difference, then starves the rest - and the ORBIT keeps the
    receipt: the eccentricity drops but nowhere near circular. A claim-side
    reader would see "burn 2 complete"; the world sees an ellipse. This is
    the stranded transfer the two-ledger law promises when fuel runs short
    mid-mission, visible in r/v without reading a single counter.
    """
    body, sim = _fly(_plant(tmp_path, 2.5))
    _advance_to(sim, 10.0)
    _leg(body, sim, 10.0, 36, 50.0, (0.0, 1.0, 0.0), burn_id=1)
    apo_t, r_max = _coast_to_apoapsis(body, sim, 46.0)
    assert abs(r_max - APOAPSIS_M) <= 0.01 * APOAPSIS_M

    _leg(body, sim, apo_t, 34, 50.0, (0.0, -1.0, 0.0), burn_id=2)
    stranded = body.truth()
    survivors = 2.5 - 1800.0 / (ISPS * G0)
    bought = 1800.0 + survivors * ISPS * G0
    assert stranded.fuel_remaining_kg == 0.0, "the thin tank gave everything it had"
    assert stranded.fuel_spent_kg == pytest.approx(2.5, abs=1e-9)
    assert stranded.fuel_ticks_throttled == 1.0, "exactly one tick splits the difference"
    assert 18.0 <= stranded.fuel_ticks_starved <= 22.0
    assert stranded.burn_impulse_applied_ns == pytest.approx(bought, rel=0.02)

    e_now = eccentricity(*_truth_rv(stranded))
    assert e_now > 1.5e-3, f"the orbit itself says UNFINISHED: e={e_now:.2e} vs 1.4e-4 bought"
    assert e_now < 3.0e-3, "the partial leg still helped - the world spent what fuel existed"


def test_a_hohmann_plant_without_a_tank_still_flys_the_transfer(tmp_path: Path) -> None:
    """Physics first, accountants second: the tankless transfer completes.

    Rung-1 authority survives the fuel era: with no tank declared the world
    buys every ask (no door, no price), the two legs land the circle anyway,
    and the fuel rows speak absence (NaN) while the event rows stay true
    zeros. The tank is opt-in; the orbit is for everyone.
    """
    body, sim = _fly(_plant(tmp_path, None))
    _advance_to(sim, 10.0)
    fresh = body.truth()
    assert math.isnan(fresh.fuel_remaining_kg) and math.isnan(fresh.fuel_spent_kg)

    _leg(body, sim, 10.0, 36, 50.0, (0.0, 1.0, 0.0), burn_id=1)
    apo_t, _ = _coast_to_apoapsis(body, sim, 46.0)
    _leg(body, sim, apo_t, 34, 50.0, (0.0, -1.0, 0.0), burn_id=2)
    after2 = body.truth()
    assert math.isnan(after2.fuel_remaining_kg)
    assert after2.fuel_ticks_starved == 0.0 and after2.fuel_ticks_throttled == 0.0
    assert after2.burn_impulse_applied_ns == pytest.approx(3500.0, abs=1e-9)
    assert eccentricity(*_truth_rv(after2)) < 5e-4, "a tankless world still honours a Hohmann"


def test_hohmann_legs_reach_the_wire_in_order() -> None:
    """The rows the hohmann probe reads by NAME are the rows the world writes.

    A positional walk of the 22-row frame against one packed Truth, pinned
    for the orbit quartet (r, v) and the fuel quartet - the probe subscribes
    through the same EDS entries these indices stand for, so this is the pin
    that protects the live bench from a silent reorder.
    """
    from sim.lockstep import Truth, truth_bytes

    truth = Truth(
        sim_time_s=2942.0,
        r_bn_n=(-7.04e6, 123.0, 0.5),
        v_bn_n=(0.01, -7511.7, 0.0),
        sigma_bn=(0.0, 0.0, 0.0, 1.0),
        omega_bn_b=(0.0, 0.0, 0.0),
        burn_impulse_applied_ns=1800.0,
        burn_ticks_applied=36.0,
        burn_ticks_saturated=0.0,
        burn_id=1.0,
        fuel_remaining_kg=3.664511,
        fuel_spent_kg=1.835489,
        fuel_ticks_starved=0.0,
        fuel_ticks_throttled=0.0,
    )
    obj = entry("BSK_LINK/TruthState")(_lib().PackedObject(bytes(truth_bytes(truth))))
    assert float(obj["FuelRemainingKg"]()) == pytest.approx(3.664511, rel=1e-12)
    assert float(obj["FuelSpentKg"]()) == pytest.approx(1.835489, rel=1e-12)
    assert float(obj["r_BN_N"][0]()) == pytest.approx(-7.04e6, rel=1e-12)
    assert float(obj["v_BN_N"][1]()) == pytest.approx(-7511.7, rel=1e-12)
