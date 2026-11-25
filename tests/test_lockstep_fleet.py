"""Multi-vehicle lockstep tests: one world step, every ack first.

These are the ONLY tests that exercise the fleet check without containers:
fake ackers (ZMQ SUB+PUSH threads speaking real EDS frames) answer one or
both channels and prove the three fleet contracts:

1. a tick completes only after EVERY channel acked (frame counters match);
2. one silent mission stalls the world - loudly, naming the vehicle;
3. the legacy single-channel entry (loose callbacks) is unchanged, so every
   frozen digest rides the same code path it always did.
"""

from __future__ import annotations

import math
import threading
import time
from pathlib import Path

import numpy as np
import pytest
import zmq

from sim.eds_link import (
    _lib,
    available,
    decode_step_ack,
    encode_step_ack,
    encode_torque_command,
    entry,
)
from sim.lockstep import (
    TOPIC_ACK,
    TOPIC_CMD,
    TOPIC_SENSOR,
    LockStalled,
    LockstepRig,
    RigChannel,
    bind_link,
)

# The fake ackers speak REAL EDS frames, so this module needs the compiled
# database like the other link tests do. Without the guard the failure was
# cryptic in the worst way: the publisher silently encoded nothing and the
# world turned zero frames ("a perpetual world should just keep turning",
# assert 0 >= 3) - a missing artifact reporting as a physics bug. The yard
# proved this by wiping edslink/dist; the skip names the command instead.
pytestmark = pytest.mark.skipif(
    not available(), reason="run edslink/build.sh to build the EDS database"
)


class _FakeTotalSim:
    """A clock that scans: one 1-second process step per SingleStepProcesses."""

    def __init__(self) -> None:
        self.CurrentNanos = 0

    @property
    def NextTaskTime(self) -> int:
        return self.CurrentNanos + 1_000_000_000

    def SingleStepProcesses(self) -> None:
        self.CurrentNanos += 1_000_000_000


class _FakeSim:
    """Just enough sim for the rig without Basilisk: no TaskList -> grid 0."""

    def __init__(self) -> None:
        self.TotalSim = _FakeTotalSim()


def _sensor_frame_parts(payload: bytes) -> int:
    """The Frame number inside a packed BSK_LINK/SensorFrame."""
    lib = _lib()
    obj = entry("BSK_LINK/SensorFrame")(lib.PackedObject(payload))
    return int(obj["Frame"]())


class _Acker:
    """A fake mission container: hears SENSOR frames, answers StepAck over PUSH.

    With ``torque`` set it behaves like fcs in ENABLE mode: a TorqueCommand for
    the frame rides the same PUSH socket BEFORE the StepAck (the ordering the
    real app documents and the check relies on).
    """

    def __init__(
        self,
        sensor_endpoint: str,
        cmd_endpoint: str,
        *,
        ack: bool = True,
        torque: tuple[float, float, float] | None = None,
    ) -> None:
        context = zmq.Context.instance()
        self._sub = context.socket(zmq.SUB)
        self._sub.connect(sensor_endpoint)
        self._sub.subscribe(TOPIC_SENSOR)
        self._push = context.socket(zmq.PUSH)
        self._push.connect(cmd_endpoint)
        self.frames_acked: list[int] = []
        self.frames_seen: list[int] = []
        self._ack = ack
        self._torque = torque
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self) -> None:
        while not self._stop.is_set():
            if self._sub.poll(50, zmq.POLLIN) != zmq.POLLIN:
                continue
            try:
                topic, body = self._sub.recv_multipart(flags=zmq.NOBLOCK)
            except zmq.Again:
                continue
            if topic != TOPIC_SENSOR:
                continue
            frame = _sensor_frame_parts(body)
            self.frames_seen.append(frame)
            if not self._ack or frame in self.frames_acked:
                continue  # idempotent like fcs: one answer per frame
            if self._torque is not None:
                self._push.send_multipart(
                    [TOPIC_CMD, bytes(encode_torque_command(0.0, self._torque, frame))]
                )
            self._push.send_multipart([TOPIC_ACK, bytes(encode_step_ack(0.0, frame, 1, 0))])
            self.frames_acked.append(frame)

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)
        self._sub.close(linger=0)
        self._push.close(linger=0)


def _channel(name: str, sink: list[np.ndarray]) -> RigChannel:
    return RigChannel(
        name=name,
        gyro=lambda: np.array([0.05, -0.04, 0.03]),
        apply_torque=lambda torque: sink.append(np.asarray(torque, dtype=float).copy()),
        truth_provider=lambda: b"truth",
        kp=np.zeros(3),
    )


def test_channel_publishes_only_registered_topics() -> None:
    """Observer topics must be landed registry rows.

    A provider on an unregistered topic would reach bsk_link as its
    "unexpected topic" event mid-flight; better to refuse the wiring before
    the world exists - sim and spacecraft learn about parts from exactly
    one file.
    """
    with pytest.raises(ValueError, match="not landed registry parts"):
        RigChannel(
            name="bogus",
            gyro=lambda: np.zeros(3),
            apply_torque=lambda torque: None,
            truth_provider=lambda: b"",
            kp=np.zeros(3),
            observers={"NOTAPART": lambda: b""},
        )


def _rig(
    tmp_path: Path,
    channels: list[RigChannel],
    *,
    lockstep: bool = True,
    ack_timeout_s: float = 2.0,
    ack_retry_s: float = 0.1,
) -> tuple[LockstepRig, list[dict]]:
    rig = LockstepRig(
        _FakeSim(),
        control_period_s=1.0,
        channels=channels,
        lockstep=lockstep,
        ack_timeout_s=ack_timeout_s,
        ack_retry_s=ack_retry_s,
    )
    socket_sets = [
        bind_link(
            f"ipc://{tmp_path}/sensor{i}",
            f"ipc://{tmp_path}/cmd{i}",
            f"ipc://{tmp_path}/truth{i}",
            f"ipc://{tmp_path}/echo{i}",
        )
        for i in range(len(channels))
    ]
    for index, sockets in enumerate(socket_sets):
        rig.attach_channel(index, sockets)
    rig.flush_link()
    return rig, socket_sets


def test_every_channel_gates_one_world_tick(tmp_path: Path) -> None:
    """A tick returns only when BOTH fake containers acked the SAME frame."""
    sink_a: list[np.ndarray] = []
    sink_b: list[np.ndarray] = []
    rig, _ = _rig(tmp_path, [_channel("alpha", sink_a), _channel("beta", sink_b)])
    ackers = [_Acker(f"ipc://{tmp_path}/sensor{i}", f"ipc://{tmp_path}/cmd{i}") for i in range(2)]
    try:
        for _ in range(3):
            torque = rig.control_tick()  # zero torque: the ackers send no CMD frame
            assert torque == pytest.approx(np.zeros(3))
        assert rig.frame == 3
        for acker in ackers:
            assert sorted(set(acker.frames_acked)) == [1, 2, 3], (
                "an acker missed a frame or gated a different counter"
            )
        for name in ("alpha", "beta"):
            channel = next(ch for ch in rig.channels if ch.name == name)
            assert len(channel.rate_history) == 3
            assert len(channel.external_torque_history) == 3
            assert all(entry_t is None for _, entry_t in channel.external_torque_history)
        assert sink_a and sink_b  # every vehicle's effector got its zero
        assert (rig.stale_torques, rig.foreign_frames, rig.bad_frames) == (0, 0, 0)
    finally:
        for acker in ackers:
            acker.close()
        rig.close()


def test_each_gate_books_its_own_torque(tmp_path: Path) -> None:
    """ENABLE-mode fakes command DIFFERENT vectors; neither may leak."""
    sink_a: list[np.ndarray] = []
    sink_b: list[np.ndarray] = []
    rig, _ = _rig(tmp_path, [_channel("alpha", sink_a), _channel("beta", sink_b)])
    ackers = [
        _Acker(
            f"ipc://{tmp_path}/sensor0",
            f"ipc://{tmp_path}/cmd0",
            torque=(0.5, -0.25, 0.0),
        ),
        _Acker(
            f"ipc://{tmp_path}/sensor1",
            f"ipc://{tmp_path}/cmd1",
            torque=(-1.0, 0.0, 2.0),
        ),
    ]
    try:
        for _ in range(3):
            rig.control_tick()
        for name, want in (("alpha", (0.5, -0.25, 0.0)), ("beta", (-1.0, 0.0, 2.0))):
            channel = next(ch for ch in rig.channels if ch.name == name)
            assert len(channel.torque_history) == 3
            for _, applied in channel.torque_history:
                assert applied == pytest.approx(np.array(want))
            for _, answered in channel.external_torque_history:
                assert answered is not None and answered == pytest.approx(np.array(want))
        assert sink_a and all(t == pytest.approx(np.array(sink_a[0])) for t in sink_a)
        assert sink_b and all(t == pytest.approx(np.array(sink_b[0])) for t in sink_b)
        assert not np.allclose(sink_a[0], sink_b[0])  # distinct laws, no cross-talk
        assert (rig.stale_torques, rig.foreign_frames, rig.bad_frames) == (0, 0, 0)
    finally:
        for acker in ackers:
            acker.close()
        rig.close()


def test_silent_mission_stalls_the_world_and_is_named(tmp_path: Path) -> None:
    """One dead container is a named stall - the world never ticks past it."""
    sink_live: list[np.ndarray] = []
    sink_dead: list[np.ndarray] = []
    rig, _ = _rig(
        tmp_path,
        [_channel("alive", sink_live), _channel("dead-mission", sink_dead)],
        ack_timeout_s=0.6,
    )
    acker = _Acker(f"ipc://{tmp_path}/sensor0", f"ipc://{tmp_path}/cmd0")
    try:
        with pytest.raises(LockStalled, match="dead-mission"):
            rig.control_tick()
        assert rig.frame == 1  # the tick exists but the world will not advance
        assert sink_live == [] and sink_dead == []  # nothing applied on a stalled tick
    finally:
        acker.close()
        rig.close()


def test_legacy_single_channel_entry_is_unchanged(tmp_path: Path) -> None:
    """The loose-callback constructor (every frozen digest's door) still works."""
    applied: list[np.ndarray] = []
    rig, _ = (
        LockstepRig(
            _FakeSim(),
            control_period_s=1.0,
            gyro=lambda: np.array([0.1, 0.0, 0.0]),
            apply_torque=lambda torque: applied.append(torque.copy()),
            truth_provider=lambda: b"truth",
            kp=np.zeros(3),
            lockstep=True,
            ack_timeout_s=2.0,
            ack_retry_s=0.1,
        ),
        None,
    )
    sockets = bind_link(
        f"ipc://{tmp_path}/sensor",
        f"ipc://{tmp_path}/cmd",
        f"ipc://{tmp_path}/truth",
        f"ipc://{tmp_path}/echo",
    )
    rig.attach_link(sockets)  # the pre-fleet surface: channel 0
    rig.flush_link()
    acker = _Acker(f"ipc://{tmp_path}/sensor", f"ipc://{tmp_path}/cmd")
    try:
        for _ in range(2):
            rig.control_tick()
        assert rig.frame == 2
        assert len(rig.rate_history) == 2  # property maps to channel 0
        assert rig.rate_history is rig.channels[0].rate_history
        assert len(applied) == 2
    finally:
        acker.close()
        rig.close()


def test_rig_refuses_an_empty_fleet_and_a_bare_single() -> None:
    """Two constructors that command nothing are caller bugs, loudly."""
    with pytest.raises(ValueError, match="channels"):
        LockstepRig(_FakeSim(), control_period_s=1.0, channels=[])
    with pytest.raises(ValueError, match="gyro"):
        LockstepRig(_FakeSim(), control_period_s=1.0, kp=np.zeros(3))


def test_ack_decodes_what_fcs_would_answer(tmp_path: Path) -> None:
    """The fake acker speaks the real StepAck codec (round-trip through EDS)."""
    encoded = bytes(encode_step_ack(3.5, 7, 1, 2))
    ack = decode_step_ack(encoded)
    assert (ack.sim_time_s, ack.frame, ack.mode, ack.torques_issued) == (3.5, 7, 1, 2)


def test_perpetual_run_stops_on_request_at_a_boundary(tmp_path: Path) -> None:
    """run_to(inf) turns until request_stop(); the world ends at a tick."""
    sink: list[np.ndarray] = []
    rig, _ = _rig(tmp_path, [_channel("eternal", sink)])
    acker = _Acker(f"ipc://{tmp_path}/sensor0", f"ipc://{tmp_path}/cmd0")
    outcome: dict[str, object] = {}

    def _run() -> None:
        try:
            rig.run_to(math.inf)
            outcome["ok"] = True
        except BaseException as exc:  # surfaced through outcome, not swallowed
            outcome["error"] = exc

    thread = threading.Thread(target=_run, daemon=True)
    thread.start()
    try:
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline and len(acker.frames_acked) < 3:
            time.sleep(0.02)
        assert len(acker.frames_acked) >= 3, "a perpetual world should just keep turning"
        rig.request_stop()
        thread.join(timeout=10.0)
        assert not thread.is_alive(), "the stop request must end run_to at a boundary"
        assert outcome.get("ok") is True, outcome
        assert rig.frame >= 3
        assert len(rig.rate_history) == rig.frame
        assert len(sink) == rig.frame  # every tick applied, none half-done
        assert (rig.stale_torques, rig.foreign_frames, rig.bad_frames) == (0, 0, 0)
    finally:
        acker.close()
        rig.close()


def test_stop_before_run_ends_immediately_without_scanning(tmp_path: Path) -> None:
    """A pre-set stop is honored: zero scans, zero frames, a clean return."""
    sink: list[np.ndarray] = []
    rig, _ = _rig(tmp_path, [_channel("never", sink)])
    rig.request_stop()
    rig.run_to(math.inf)  # returns at once - no scan, so no acks needed
    assert rig.frame == 0
    assert sink == []
