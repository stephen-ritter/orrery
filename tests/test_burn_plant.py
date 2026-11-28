"""Rung 1, world side: a declared thruster set plus a quota raises the orbit.

No flight software in sight: these tests ride the burn door directly (and,
once, the ZMQ dispatch a container's frames will use), so what they grade is
the plant-and-world half of the capability - allocation lands on the
effectors, the applied ledger counts what the plant was GIVEN, a missing
frame ends the burn, and an undeclared effector refuses. The craft's claim
ledger (fcs HK) is a later flight; the two ledgers meeting is a bench
finding, exactly as the two-ledger law demands.

The flying tests build the plant without a rig (the camera door's pattern)
and ride ``SingleStepProcesses`` themselves: the burn door is a plain message
write, so the interleaving lives here, at one-second boundaries, on the
QUIET variant of the bench plant - a born-still, attitude-locked bird whose
body frame is inertial frame +Y +Z +X, so a +Y-body quota is pure along-track
burn on a circular orbit and the analytic two-body number is real.
"""

from __future__ import annotations

import time
from pathlib import Path
from typing import Any

import numpy as np
import pytest
import zmq
from bskforge import load_scenario

from codegen.eds_sources import CFS
from sim import forge_scenario as fz
from sim.eds_link import (
    BurnCommand,
    _lib,
    available,
    decode_burn_command,
    encode_burn_command,
    encode_step_ack,
    entry,
)
from sim.lockstep import Truth, truth_bytes

pytestmark = [
    pytest.mark.skipif(not CFS.is_dir(), reason="cFS is not fetched: run scripts/fetch_cfs.sh"),
    pytest.mark.skipif(not available(), reason="run edslink/build.sh to build the EDS database"),
]

BURN_TOML = fz.DEFAULT_SCENARIO.parent / "burn.toml"
CHIEF_TOML = fz.DEFAULT_SCENARIO.parent / "chief.toml"

MU_EARTH = 3.986004418e14  # [m^3/s^2] Basilisk's centered-gravity constant


def apoapsis(truth: Truth) -> float:
    """Two-body apoapsis radius [m] from a truth frame (the bench's eye)."""
    r = np.asarray(truth.r_bn_n, dtype=float)
    v = np.asarray(truth.v_bn_n, dtype=float)
    r_norm = float(np.linalg.norm(r))
    v_norm = float(np.linalg.norm(v))
    e_vector = ((v_norm**2 - MU_EARTH / r_norm) * r - float(np.dot(r, v)) * v) / MU_EARTH
    eccentricity = float(np.linalg.norm(e_vector))
    semi_major = 1.0 / (2.0 / r_norm - v_norm**2 / MU_EARTH)
    return semi_major * (1.0 + eccentricity)


# ---- the flying seam: a rig-free plant, stepped one scan at a time ------------


def _quiet_scenario(tmp_path: Path) -> Path:
    """The bench plant born STILL: zero attitude, zero rate, +X +Y +Z inertial.

    A born-tumbled bird sweeps its thrust axis around; with the wheels owning
    only the tumbles the FSW will fight, the clean physics question - does a
    quota along an axis raise the orbit by the analytic amount - wants a body
    frame that IS an inertial frame. Same TOML otherwise: same mass, orbit,
    thruster set.
    """
    text = BURN_TOML.read_text().replace(
        "sigma_BN = [0.05, -0.08, 0.11]\nomega_BN_B = [0.05, -0.04, 0.03]",
        "sigma_BN = [0.0, 0.0, 0.0]\nomega_BN_B = [0.0, 0.0, 0.0]",
    )
    assert text != BURN_TOML.read_text(), "the quiet variant must actually edit the attitude block"
    path = tmp_path / "burn_quiet.toml"
    path.write_text(text)
    return path


def _fly_body(path: Path) -> tuple[fz.ForgeBody, Any]:
    """Build the plant with no rig, initialize it, hand back the body + sim.

    This is the camera door's build_bodies pattern: the harness surface a rig
    would add (barrier, laws, sockets) is exactly what the physics tests do
    not want between them and the effector. The scenario's fsw rate is the
    tick the door's quota covers, told to the body like any scenario would.
    """
    config = load_scenario(path)
    (sc_cfg,) = config.spacecraft
    built = fz.ScenarioRunner(config).build()
    ctx = built.context
    body = fz.ForgeBody(ctx, sc_cfg, control_period_s=float(config.simulation.dt.fsw))
    ctx.sim.InitializeSimulation()
    return body, ctx.sim


def _advance(sim: Any, to_s: float) -> None:  # noqa: ANN401 - Basilisk SimBaseClass
    """One-scan-at-a-time to a boundary (the rig's own stepping primitive)."""
    from Basilisk.utilities import macros

    horizon = macros.sec2nano(to_s)
    while sim.TotalSim.CurrentNanos < horizon:
        sim.TotalSim.SingleStepProcesses()


def _burn_for_ticks(
    body: fz.ForgeBody,
    sim: Any,  # noqa: ANN401 - Basilisk SimBaseClass
    first_tick: int,
    ticks: int,
    force_n: float,
) -> None:
    """Open the door for `ticks` one-second quotas, then close it by silence."""
    for tick in range(ticks):
        body.apply_burn(
            BurnCommand(
                sim_time_s=float(first_tick + tick),
                force_n=force_n,
                axis_b=(0.0, 1.0, 0.0),
                frame=first_tick + tick + 1,
                burn_id=1,
            )
        )
        _advance(sim, first_tick + tick + 1.0)


# ---- physics: the quota moves the orbit, and silence ends it -------------------


def test_quota_raises_apoapsis_and_silence_freezes_the_orbit(tmp_path: Path) -> None:
    """Ten ticks of 3 N along-track: the analytic rise appears, then holds.

    Born-still on a circular 7000 km orbit, +Y body IS the velocity vector:
    thirty newton-seconds on a 300 kg bus is 0.1 m/s. A tangential burn
    leaves the burn point the periapsis, so the FAR side of the orbit rises
    by twice the semi-major change: d(ra) ~ 4 a^2 v dv / mu ~ 370 m. The
    tolerance is wide because the finite burn bends the path slightly; the
    SECOND assertion is the tight one - apoapsis is an invariant once thrust
    stops, so any latched effector (a frame that bought more than its tick)
    shows up as the coast changing what nothing should be able to change.
    """
    body, sim = _fly_body(_quiet_scenario(tmp_path))
    assert body.has_thrusters
    start = body.truth()
    assert start.burn_ticks_applied == 0.0

    _advance(sim, 10.0)
    baseline = apoapsis(body.truth())
    assert body.truth().burn_ticks_applied == 0.0

    _burn_for_ticks(body, sim, first_tick=10, ticks=10, force_n=3.0)

    burned = body.truth()
    assert burned.burn_ticks_applied == 10.0
    assert burned.burn_impulse_applied_ns == pytest.approx(30.0, rel=1e-12)
    assert burned.burn_ticks_saturated == 0.0
    assert burned.burn_id == 1.0

    raised = apoapsis(burned)
    assert 240.0 < raised - baseline < 500.0, (
        f"a 30 N s along-track quota moved the apoapsis {raised - baseline:.1f} m, "
        "not the ~370 m the two-body estimate gives (allocation lost? effector order?)"
    )

    _advance(sim, 40.0)
    coasted = body.truth()
    assert coasted.burn_ticks_applied == 10.0, "a tick without a frame must not thrust"
    assert coasted.burn_impulse_applied_ns == pytest.approx(30.0, rel=1e-12)
    assert apoapsis(coasted) == pytest.approx(raised, abs=2.0), (
        "the orbit kept rising on a silent link: the effector latched a burn"
    )


def test_saturated_ticks_count_what_the_hardware_could_deliver() -> None:
    """9 N on a 5 N axis applies 5 N; a Z demand applies nothing - both logged."""
    body, _sim = _fly_body(BURN_TOML)
    body.apply_burn(
        BurnCommand(sim_time_s=0.0, force_n=9.0, axis_b=(1.0, 0.0, 0.0), frame=1, burn_id=7)
    )
    body.apply_burn(
        BurnCommand(sim_time_s=0.0, force_n=3.0, axis_b=(0.0, 0.0, 1.0), frame=2, burn_id=7)
    )
    body.apply_burn(
        BurnCommand(sim_time_s=0.0, force_n=-1.0, axis_b=(1.0, 0.0, 0.0), frame=3, burn_id=8)
    )
    body.apply_burn(
        BurnCommand(sim_time_s=0.0, force_n=3.0, axis_b=(0.0, 0.0, 0.0), frame=4, burn_id=8)
    )

    stats = body._burn_stats
    assert stats["applied"] == 2, "the two honest asks applied; the two bad ones were refused"
    assert stats["saturated"] == 2, "clipped X and wholly-unreachable Z both count"
    assert stats["refused_bad"] == 2
    assert stats["impulse_applied_ns"] == pytest.approx(5.0, rel=1e-12)
    assert stats["last_burn_id"] == 7
    truth = body.truth()
    assert truth.burn_ticks_applied == 2.0
    assert truth.burn_ticks_saturated == 2.0
    assert truth.burn_impulse_applied_ns == pytest.approx(5.0, rel=1e-12)


def test_a_vehicle_without_thrusters_refuses_out_loud() -> None:
    """The chief declares no thrusters: a burn frame is a counted absence."""
    body, _sim = _fly_body(CHIEF_TOML)
    assert not body.has_thrusters
    body.apply_burn(
        BurnCommand(sim_time_s=0.0, force_n=3.0, axis_b=(0.0, 1.0, 0.0), frame=1, burn_id=1)
    )
    assert body._burn_stats["refused_undeclared"] == 1
    assert body._burn_stats["applied"] == 0
    assert body.truth().burn_ticks_applied == 0.0


# ---- the wire: the frame a container will send, and truth's reply --------------


def test_burn_frame_round_trips_at_its_pinned_length() -> None:
    """56 B: the EDS container and the orrery_bus_msg.h assert's other half."""
    payload = bytes(encode_burn_command(12.0, 3.0, (0.0, 1.0, 0.0), 41, 2))
    assert len(payload) == 56
    cmd = decode_burn_command(payload)
    assert cmd.sim_time_s == pytest.approx(12.0)
    assert cmd.force_n == pytest.approx(3.0)
    assert cmd.axis_b == pytest.approx((0.0, 1.0, 0.0))
    assert cmd.frame == 41
    assert cmd.burn_id == 2


def test_truth_frame_carries_the_applied_ledger() -> None:
    """176 B: 14 orbit doubles plus the four-row burn and four-row fuel ledgers, ground only."""
    frame = truth_bytes(
        Truth(
            sim_time_s=1.0,
            r_bn_n=(7.0e6, 0.0, 0.0),
            v_bn_n=(0.0, 7546.1, 0.0),
            sigma_bn=(0.0, 0.0, 0.0, 1.0),
            omega_bn_b=(0.0, 0.0, 0.0),
            burn_impulse_applied_ns=30.5,
            burn_ticks_applied=10.0,
            burn_ticks_saturated=1.0,
            burn_id=3.0,
        )
    )
    assert len(frame) == 176
    obj = entry("BSK_LINK/TruthState")(_lib().PackedObject(frame))
    assert float(obj["BurnImpulseAppliedNs"]()) == pytest.approx(30.5)
    assert float(obj["BurnTicksApplied"]()) == 10.0
    assert float(obj["BurnTicksSaturated"]()) == 1.0
    assert float(obj["BurnId"]()) == 3.0


def test_burn_frame_rides_the_cmd_socket_and_never_gates() -> None:
    """The ZMQ dispatch, with no container: BURN applies, keeps waiting, is once.

    The pre-pushed answer script mirrors the fake-FSW torque test: frame 1
    rides a burn quota plus its ack, frame 2 answers bare. One frame bought
    exactly one tick - the gate never opened for it, and the coast frame
    after it thrusts nobody.
    """
    ports = {
        "sensor": "tcp://127.0.0.1:17640",
        "cmd": "tcp://127.0.0.1:17641",
        "truth": "tcp://127.0.0.1:17650",
        "echo": "tcp://127.0.0.1:17651",
    }
    context = zmq.Context.instance()
    scenario = fz.ForgeScenario(BURN_TOML, lockstep=True, ack_timeout_s=2.0)
    scenario.attach_link(fz.bind_link(ports["sensor"], ports["cmd"], ports["truth"], ports["echo"]))
    push = context.socket(zmq.PUSH)
    push.connect(ports["cmd"])
    time.sleep(0.5)  # PUSH handshake, same as the barrier tests

    scenario.flush_link()
    push.send_multipart([b"BURN", bytes(encode_burn_command(1.0, 3.0, (0.0, 1.0, 0.0), 1, 1))])
    push.send_multipart([b"ACK", bytes(encode_step_ack(1.0, 1, 0, 1))])
    push.send_multipart([b"ACK", bytes(encode_step_ack(2.0, 2, 0, 1))])

    scenario.control_tick()
    scenario.control_tick()

    stats = scenario.body._burn_stats
    assert stats["applied"] == 1, "one frame, one tick: apply-once across the wire too"
    assert stats["impulse_applied_ns"] == pytest.approx(3.0, rel=1e-12)
    assert scenario.rig.burn_frames == 1
    assert (scenario.stale_torques, scenario.foreign_frames, scenario.bad_frames) == (0, 0, 0)
    assert scenario.frame == 2

    push.close(linger=0)
    scenario.close()
