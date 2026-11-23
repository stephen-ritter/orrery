"""W3: the bskforge-built plant must be indistinguishable from the hand-written one.

The whole point of decision D7 is that scenarios come from TOML without the
lockstep contract changing shape. These tests demand the strongest version of
that: the two build paths produce a BIT-IDENTICAL trajectory sha256 - same
integration, same actuation order (the P3f trap re-armed inside bskforge and
disarmed by ``restore_effector_order``), same onboard-knowledge stream. A
toml that grows a bskforge control law is rejected outright: the flight
software lives in cFS, not in the plant.
"""

from __future__ import annotations

import time
from pathlib import Path

import numpy as np
import pytest
import zmq
from bskforge import load_scenario

from codegen.eds_sources import CFS
from sim import detumble_scenario as dz
from sim import forge_scenario as fz
from sim.eds_link import available, decode_onboard_state, encode_step_ack, encode_torque_command

pytestmark = [
    pytest.mark.skipif(not CFS.is_dir(), reason="cFS is not fetched: run scripts/fetch_cfs.sh"),
    pytest.mark.skipif(not available(), reason="run edslink/build.sh to build the EDS database"),
]

ORRERY_TOML = fz.DEFAULT_SCENARIO.parent / "chief.toml"


@pytest.fixture()
def forge_ports() -> dict[str, str]:
    """Loopback endpoints for the forge plant, clear of the detumble test ports."""
    return {
        "sensor": "tcp://127.0.0.1:17680",
        "cmd": "tcp://127.0.0.1:17681",
        "truth": "tcp://127.0.0.1:17690",
        "echo": "tcp://127.0.0.1:17691",
    }


_MINIMAL_TOML = """\
[simulation]
duration = 10.0
[simulation.dt]
dynamics = 1.0
fsw = 1.0
[world]
central_body = "earth"
[[spacecraft]]
name = "sc1"
"""


def _write(tmp_path: Path, body: str) -> Path:
    path = tmp_path / "scenario.toml"
    path.write_text(body, encoding="utf-8")
    return path


def test_forge_plant_matches_handwritten_plant_bit_for_bit() -> None:
    """The W3 parity gate: two build paths, one trajectory (D7).

    Gravity is the one thing the TOML adds (bskforge always connects the
    central body), and the digest stays identical: point-mass gravity bends
    the ORBIT, never the ATTITUDE - the hashed rate history cannot see it.
    """
    forge = fz.ForgeScenario()
    forge.run_to(60.0)
    forge_digest = fz.trajectory_sha256(forge.rate_history)
    forge_ratio = forge.rate_norm() / np.linalg.norm(dz.INITIAL_RATE_RADS)

    hand = dz.DetumbleScenario(control_period_s=1.0)
    hand.run_to(60.0)
    hand_digest = dz.trajectory_sha256(hand.rate_history)

    assert forge_ratio < dz.CONVERGENCE_RATIO, f"forge plant only fell to {forge_ratio:.3%}"
    assert forge_digest == hand_digest, (
        f"build paths disagree: forge {forge_digest[:16]}... != "
        f"hand-written {hand_digest[:16]}... (effector order? integration? seed?)"
    )


def test_forge_plant_is_deterministic_across_runs() -> None:
    """Same TOML, same seed, same binary: same digest (the P3e artifact holds)."""
    first = fz.ForgeScenario()
    first.run_to(30.0)
    second = fz.ForgeScenario()
    second.run_to(30.0)
    assert fz.trajectory_sha256(first.rate_history) == fz.trajectory_sha256(second.rate_history)


def test_effector_runs_before_the_spacecraft_after_the_build_fix() -> None:
    """``restore_effector_order`` reorders the REAL Basilisk task list.

    bskforge registers [sc, extFT]; the fix re-inserts extFT ahead of every
    -1 model. Asserting on the task's own model list (not on a digest) keeps
    a future bskforge version that reorders natively honest: if the extFT
    already leads, the re-insertion stays harmless, and this test stays green.
    """
    scenario = fz.ForgeScenario()
    (task,) = [t for t in scenario.sim.TaskList if t.Name == fz.DYNAMICS_TASK]
    tags = [model.ModelPtr.ModelTag for model in task.TaskData.TaskModels]
    ft = f"{scenario.name}_extFT"
    assert tags.count(ft) >= 1
    assert tags.index(ft) < tags.index(scenario.name), (
        f"spacecraft integrates before the torque port updates: {tags}"
    )


def test_bskforge_fsw_stack_is_rejected(tmp_path: Path) -> None:
    """D7 in force: a TOML that grows an onboard law cannot be loaded."""
    path = _write(tmp_path, _MINIMAL_TOML + "[spacecraft.fsw]\n")
    with pytest.raises(fz.ForgeError, match="control law lives"):
        fz.ForgeScenario(path)


def test_multi_vehicle_scenario_is_rejected(tmp_path: Path) -> None:
    """One rig commands one body; a second spacecraft means a second run."""
    path = _write(tmp_path, _MINIMAL_TOML + '[[spacecraft]]\nname = "sc2"\n')
    with pytest.raises(fz.ForgeError, match="exactly one spacecraft"):
        fz.ForgeScenario(path)


def test_fake_fsw_closes_the_gate_on_the_forge_plant(forge_ports: dict[str, str]) -> None:
    """The lockstep barrier rides the forge plant unchanged: same rig, same rules.

    Same pre-pushed answer script that pins the hand-written plant's gate:
    torque{N} applied on frame N, ack-only (loop disabled) applies bit-exact
    zero. A forge-only barrier bug - reading a bskforge sensor message where
    truth lives, say - would miss the frame-exact torque here.
    """
    context = zmq.Context.instance()
    scenario = fz.ForgeScenario(lockstep=True, ack_timeout_s=2.0)
    scenario.attach_link(
        fz.bind_link(
            forge_ports["sensor"], forge_ports["cmd"], forge_ports["truth"], forge_ports["echo"]
        )
    )
    push = context.socket(zmq.PUSH)
    push.connect(forge_ports["cmd"])
    time.sleep(0.5)  # PUSH handshake: connects are async, the queue is not shared yet

    scenario.flush_link()
    torque_1 = (-0.25, 0.5, -0.75)
    push.send_multipart([b"CMD", bytes(encode_torque_command(1.0, torque_1, 1))])
    push.send_multipart([b"ACK", bytes(encode_step_ack(1.0, 1, 1, 1))])
    push.send_multipart([b"ACK", bytes(encode_step_ack(2.0, 2, 0, 1))])  # disabled: no torque

    applied = [scenario.control_tick() for _ in range(2)]
    assert np.allclose(applied[0], torque_1, atol=0.0), "torque{N} must be applied on frame N"
    assert np.allclose(applied[1], 0.0), "a disabled loop applies bit-exact zero, frame-matched"
    assert (scenario.stale_torques, scenario.foreign_frames, scenario.bad_frames) == (0, 0, 0)
    assert scenario.frame == 2

    push.close(linger=0)
    scenario.close()


def test_cli_reports_convergence(capsys: pytest.CaptureFixture[str]) -> None:
    """ForgeScenario is drop-in for scripts/p3_roundtrip.sh: same verdict lines."""
    assert fz.main(["--duration", "30", "--control-period", "1.0", "--quiet"]) == 0
    out = capsys.readouterr().out
    assert "DETUMBLE OK" in out
    assert "trajectory sha256" in out


# ---- P4a: the operations plant (wheels + sensor suite) -----------------------


def test_wheeled_plant_detumbles_through_the_wheels() -> None:
    """The rig commands a WHEEL ARRAY, not a bare torque port (P4a).

    Same lockstep-independent law, same rig, same acceptance ratio - but
    actuation now goes through motor torques (body torque = -motor torque)
    with 2 N m clamps and real momentum bookkeeping: the momentum that left
    the body must show up as wheel speed (Js * omega_w ~= the per-axis body
    momentum at init). Determinism is re-proved here rather than trusted:
    the plant IMU is a new noise source, and its ModelTag seed must keep the
    trajectory a pure function of the scenario.
    """
    first = fz.ForgeScenario(ORRERY_TOML)
    start = first.rate_norm()
    first.run_to(120.0)
    ratio = first.rate_norm() / start
    speeds = first.wheel_speeds

    assert ratio < dz.CONVERGENCE_RATIO, f"wheeled detumble only fell to {ratio:.3%}"
    assert speeds is not None and speeds.shape == (3,), "one speed per fitted wheel"
    assert np.all(np.abs(speeds) > 10.0), f"wheels barely moved: {speeds}"

    second = fz.ForgeScenario(ORRERY_TOML)
    second.run_to(120.0)
    assert fz.trajectory_sha256(first.rate_history) == fz.trajectory_sha256(second.rate_history), (
        "the plant IMU made the trajectory non-reproducible (seed? scan order?)"
    )


def test_wheel_effector_runs_before_the_spacecraft() -> None:
    """The P3f freshness fix carries to wheel arrays - with a stateful twist.

    The re-registration duplicates the wheel effector's UpdateState; safe
    because that method only reads messages and assigns (the wheel
    integration itself belongs to the hub's coupled integrator). If a future
    Basilisk makes UpdateState accumulate, this test's count == 2 is the
    tripwire that the duplicate is no longer free.
    """
    scenario = fz.ForgeScenario(ORRERY_TOML)
    (task,) = [t for t in scenario.sim.TaskList if t.Name == fz.DYNAMICS_TASK]
    tags = [model.ModelPtr.ModelTag for model in task.TaskData.TaskModels]
    rw = f"{scenario.name}_rwEff"
    assert tags.count(rw) == 2, f"expected [lead, duplicate] registrations: {tags}"
    assert tags.index(rw) < tags.index(scenario.name), (
        f"spacecraft integrates before the wheel array updates: {tags}"
    )


def test_plant_imu_is_the_rate_knowledge_when_fitted() -> None:
    """With an IMU in the TOML, the measurement IS the sensor message (P4a).

    Shortly after init the gyro's random walk is still ~1e-8 rad/s, so the
    gap between measurement and truth is the configured bias - proof that
    the harness reads the plant's sensor, not truth plus a Python model.
    The TOML's bias mirrors the mission constant; both are pinned here so
    the two knowledge paths stay comparable runs, not apples and oranges.
    """
    config = load_scenario(ORRERY_TOML)
    imu = next(s for s in config.spacecraft[0].sensors if s.type == "imuSensor")
    assert np.allclose(imu.gyro_bias, dz.GYRO_BIAS_RADS), "TOML bias drifted from the constant"

    scenario = fz.ForgeScenario(ORRERY_TOML)
    scenario.run_to(0.5)  # five scans, no control boundary crossed: law never runs
    measured_minus_truth = scenario.gyro() - np.asarray(scenario.truth().omega_bn_b, dtype=float)
    np.testing.assert_allclose(measured_minus_truth, dz.GYRO_BIAS_RADS, atol=1e-6)


def test_operations_plant_sees_the_sun_from_boot() -> None:
    """The P4b provider must carry light, or it silently teaches fcs "eclipse".

    First flight of chief.toml caught a real geometry bug this way: a
    32 deg initial attitude put the sun 43.7 deg off the css1 axis, outside a
    40 deg FOV - every cosine zero, perfectly legal physics, and every later
    "no sun" mode logic would have inherited the mistake quietly. The sun is
    +X inertial here, so some member of the +/-X pair must see it at boot.
    """
    scenario = fz.ForgeScenario(ORRERY_TOML)
    scenario.run_to(1.0)
    state = decode_onboard_state(scenario._onboard_state_bytes())
    assert state.css_cos[0] > 0.1, f"css1 blind at boot: {state.css_cos}"  # the +X eye
    assert abs(state.st_quat_bn[0]) > 0.5, "ST quaternion must be unit-ish, not a zero message"
    assert max(abs(v) for v in state.mag_b) > 1e-7, "magnetometer must feel the dipole"
