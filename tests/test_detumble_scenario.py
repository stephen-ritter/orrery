"""The detumble scenario closes its rate loop through the mission's EDS schema.

Three things are pinned here, in order of how easily they break:

1. The closed loop works: a tumbling body damps to its gyro noise floor using only
   onboard knowledge (the sensor model, not truth).
2. Onboard knowledge stays distinct from simulator truth. If someone "optimises" the
   controller to read truth, the rates converge faster and this test notices.
3. What leaves the process is a ``BSK_LINK`` frame that EdsLib decodes - the same
   bytes ``bsk_link`` will ingest in P3d.
"""

from __future__ import annotations

import time

import numpy as np
import pytest
import zmq

from codegen.eds_sources import CFS
from sim import detumble_scenario as dz
from sim.eds_link import (
    _lib,
    available,
    decode_torque_command,
    encode_step_ack,
    encode_torque_command,
    entry,
)

pytestmark = [
    pytest.mark.skipif(not CFS.is_dir(), reason="cFS is not fetched: run scripts/fetch_cfs.sh"),
    pytest.mark.skipif(not available(), reason="run edslink/build.sh to build the EDS database"),
]


@pytest.fixture()
def link_ports() -> dict[str, str]:
    """Loopback endpoints, away from the real link ports so a live mission can coexist."""
    return {
        "sensor": "tcp://127.0.0.1:17580",
        "cmd": "tcp://127.0.0.1:17581",
        "truth": "tcp://127.0.0.1:17590",
        "echo": "tcp://127.0.0.1:17591",
    }


def _subscribe(context: zmq.Context, endpoint: str, topic: str) -> zmq.Socket:
    sock = context.socket(zmq.SUB)
    sock.connect(endpoint)
    sock.setsockopt_string(zmq.SUBSCRIBE, topic)
    return sock


def _drain(sock: zmq.Socket, limit: int = 10_000) -> list[bytes]:
    """Every payload currently queued on ``sock`` (non-blocking)."""
    payloads: list[bytes] = []
    while len(payloads) < limit and sock.poll(50, zmq.POLLIN):
        payloads.append(sock.recv_multipart()[1])
    return payloads


def test_rate_loop_damps_the_tumble_using_only_onboard_knowledge() -> None:
    """The acceptance bound: |omega| collapses from the initial tumble."""
    scenario = dz.DetumbleScenario(control_period_s=1.0)
    start = scenario.rate_norm()
    assert start == pytest.approx(np.linalg.norm(dz.INITIAL_RATE_RADS), rel=1e-9)

    scenario.run_to(60.0)
    ratio = scenario.rate_norm() / start

    assert ratio < dz.CONVERGENCE_RATIO, f"rate only fell to {ratio:.3%} of the tumble"
    # Settling at the gyro noise floor means the loop is limited by sensing, not torque.
    assert scenario.rate_norm() < 10 * dz.GYRO_NOISE_SIGMA_RADS


def test_sensor_frame_reports_the_gyro_not_the_truth() -> None:
    """The satellite's rate knowledge is biased and noisy; truth is not."""
    scenario = dz.DetumbleScenario(control_period_s=1.0)
    measured = scenario.gyro()
    truth = np.asarray(scenario.truth().omega_bn_b, dtype=float)

    assert np.allclose(measured - truth, dz.GYRO_BIAS_RADS, atol=1e-3)
    assert not np.allclose(measured, truth)

    # And once the transient is gone the closed loop settles at the gyro bias, not at
    # zero: the controller drives the *measured* rate to zero, so truth parks at
    # -bias. That residual is the honest signature of an onboard-knowledge-only loop.
    perfect = dz.DetumbleScenario(control_period_s=1.0, gyro_noise_sigma_rads=0.0)
    perfect.run_to(180.0)
    residual = np.asarray(perfect.truth().omega_bn_b, dtype=float)
    assert np.linalg.norm(residual - (-dz.GYRO_BIAS_RADS)) < 1e-5, (
        f"closed loop settled at {residual}, expected the negated gyro bias"
    )
    assert np.linalg.norm(residual) > 0.0


def test_published_frames_are_eds_link_frames(tmp_path: object, link_ports: dict[str, str]) -> None:
    """SENSOR/CMD/TRUTH carry BSK_LINK containers, decodable by the EdsLib database."""
    scenario = dz.DetumbleScenario(control_period_s=0.2)
    scenario.attach_link(
        dz.bind_link(
            link_ports["sensor"], link_ports["cmd"], link_ports["truth"], link_ports["echo"]
        )
    )
    context = zmq.Context.instance()
    subs = {
        "sensor": _subscribe(context, link_ports["sensor"], "SENSOR"),
        "echo": _subscribe(context, link_ports["echo"], "CMD"),
        "truth": _subscribe(context, link_ports["truth"], "TRUTH"),
    }
    time.sleep(0.5)  # let the SUB handshake complete (ZMQ drops pre-subscribe frames)

    scenario.run_to(2.0)
    frames = {name: _drain(sock) for name, sock in subs.items()}
    for sock in subs.values():
        sock.close(linger=0)
    scenario.close()

    ticks = len(scenario.rate_history)
    assert ticks > 1
    for name, payloads in frames.items():
        assert len(payloads) == ticks, f"{name} channel lost frames"

    lib = _lib()
    assert len(frames["sensor"][0]) == 40, "SensorFrame is 4 doubles + a counter"
    assert len(frames["echo"][0]) == 40, "TorqueCommand is 4 doubles + a counter"
    assert len(frames["truth"][0]) == 112, "TruthState is 14 doubles"

    sensor = entry("BSK_LINK/SensorFrame")(lib.PackedObject(frames["sensor"][-1]))
    assert sensor["Frame"]() == ticks
    measured = np.array([sensor["OmegaBnB"][i]() for i in range(3)])

    command = decode_torque_command(frames["echo"][-1])
    assert command.frame == ticks
    assert np.linalg.norm(command.torque_sum_b) > 0.0
    assert command.torque_sum_b == pytest.approx(scenario.torque_history[-1][1], abs=1e-12), (
        "the published command must be the torque the dynamics actually felt"
    )
    assert np.allclose(command.torque_sum_b, -dz.KP_RADS * measured, atol=1e-12), (
        "the command on the wire is not the control law applied to the published rate"
    )

    truth_obj = entry("BSK_LINK/TruthState")(lib.PackedObject(frames["truth"][-1]))
    published_truth = np.array([truth_obj["omega_BN_B"][i]() for i in range(3)])
    assert not np.allclose(published_truth, measured), (
        "truth and onboard knowledge arrived identical: the sensor model is bypassed"
    )
    assert np.linalg.norm(published_truth - measured) < 1e-2, (
        "the gyro stopped resembling the rate it measures"
    )


def test_lockstep_gate_aborts_loudly_without_acks(link_ports: dict[str, str]) -> None:
    """No flight software -> no StepAck -> the sim STOPS (never coast).

    This is the failure mode the check exists for: a dead mission container.
    The run must die inside the ack timeout, not fly on with stale knowledge.
    The wait doubles as a SHOUT: while the check is shut the sensor frame is
    retransmitted every ``ack_retry_s`` - that retransmission is what saves
    frame 1 from ZMQ's eaten-first-frame handshake, so it gets pinned here.
    """
    scenario = dz.DetumbleScenario(
        control_period_s=1.0, lockstep=True, ack_timeout_s=0.45, ack_retry_s=0.1
    )
    scenario.attach_link(
        dz.bind_link(
            link_ports["sensor"], link_ports["cmd"], link_ports["truth"], link_ports["echo"]
        )
    )
    context = zmq.Context.instance()
    sub = _subscribe(context, link_ports["sensor"], "SENSOR")
    time.sleep(0.5)  # this test's own SUB must not eat the frames it counts

    started = time.monotonic()
    with pytest.raises(dz.LockStalled, match="no StepAck for frame 1"):
        scenario.run_to(5.0)
    assert time.monotonic() - started < 3.0, "the timeout, not the duration, bounds the wait"
    assert scenario.sensor_retransmits >= 2, "a waiting check retransmits its sensor frame"
    assert len(_drain(sub)) == 1 + scenario.sensor_retransmits
    sub.close(linger=0)
    scenario.close()


def test_fake_fsw_closes_the_gate_frame_exact(link_ports: dict[str, str]) -> None:
    """The whole barrier contract, answered by a pre-pushed fake `fcs`.

    ZeroMQ PUSH queues survive until the check drains them, so the "flight
    software" here is a script of multipart answers - in real runs bsk_link
    pushes them mid-check, per ZMQ's per-pair ordering. Pins, frame by frame:
    torque{N}+ack{N} -> applied; ack-only (disabled) -> bit-exact zero torque
    applied; stale/future/undecodable answers -> tripwire counters, the run
    keeps its answer for THIS frame; a future ack -> hard abort. ``flush_link``
    starts the run, and the counters proving it clean come for free: a failed
    flush would land as a tripwire here.
    """
    context = zmq.Context.instance()
    scenario = dz.DetumbleScenario(control_period_s=1.0, lockstep=True, ack_timeout_s=2.0)
    scenario.attach_link(
        dz.bind_link(
            link_ports["sensor"], link_ports["cmd"], link_ports["truth"], link_ports["echo"]
        )
    )
    push = context.socket(zmq.PUSH)
    push.connect(link_ports["cmd"])
    time.sleep(0.5)  # PUSH handshake: connects are async, the queue is not shared yet

    scenario.flush_link()  # nothing queued: must not disturb the run
    torque_1 = (-0.25, 0.5, -0.75)
    push.send_multipart([b"CMD", bytes(encode_torque_command(1.0, torque_1, 1))])
    push.send_multipart([b"ACK", bytes(encode_step_ack(1.0, 1, 1, 1))])
    push.send_multipart([b"ACK", bytes(encode_step_ack(2.0, 2, 0, 1))])  # disabled: no torque
    push.send_multipart([b"ACK", bytes(encode_step_ack(1.0, 1, 1, 1))])  # stale ack
    push.send_multipart([b"CMD", b"junk"])  # undecodable
    push.send_multipart([b"CMD", bytes(encode_torque_command(9.0, (0.0, 0.0, 0.0), 99))])  # future
    push.send_multipart([b"CMD", bytes(encode_torque_command(3.0, (0.1, 0.2, 0.3), 3))])
    push.send_multipart([b"ACK", bytes(encode_step_ack(3.0, 3, 1, 2))])

    applied = [scenario.control_tick() for _ in range(3)]
    assert np.allclose(applied[0], torque_1, atol=0.0), "torque{N} must be applied on frame N"
    assert np.allclose(applied[1], 0.0), "a disabled loop applies bit-exact zero, frame-matched"
    assert np.allclose(applied[2], (0.1, 0.2, 0.3), atol=0.0), "stale/bad/foreign answers dropped"
    assert (scenario.stale_torques, scenario.foreign_frames, scenario.bad_frames) == (1, 1, 1)

    push.send_multipart([b"ACK", bytes(encode_step_ack(77.0, 77, 1, 9))])  # future ack
    with pytest.raises(dz.LockStalled, match="future frame 77"):
        scenario.control_tick()
    push.close(linger=0)
    scenario.close()


def test_cli_reports_convergence(capsys: pytest.CaptureFixture[str]) -> None:
    """The scenario is a human-facing script: its verdict line is the contract."""
    assert dz.main(["--duration", "30", "--control-period", "1.0"]) == 0
    out = capsys.readouterr().out
    assert "DETUMBLE OK" in out
    assert "initial |omega|" in out and "final   |omega|" in out
