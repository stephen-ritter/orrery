"""The orrery lockstep harness: the barrier, the clock, the link sockets.

One check implementation, many plants - and for a fleet, many vehicles
world: the rig checks ONE world step on an ack from EVERY attached mission
per container, one channel per vehicle. This module owns everything the
amendment made load-bearing - the socket farm, the publish-and-check tick,
the retransmitting barrier handshake with its tripwire counters, and the
boundary-aware stepping that lets the control period be any integer
multiple of the sim's scan grid. What a vehicle *is* (hand-written rigid
body, bskforge TOML build, one body of a multi-vehicle world) is a channel
of callbacks:

* ``gyro() -> omega``      - what the satellite measures, never truth;
* ``apply(torque)``        - write the effector command for this frame;
* ``truth_bytes() -> bytes`` - the ground-only truth frame.

``DetumbleScenario`` and ``ForgeScenario`` are thin plant builders on one
channel; ``ForgeFleet`` rides N channels on one sim. Two copies of a
barrier would drift, so there is exactly one copy: this file.
"""

from __future__ import annotations

import hashlib
import math
import os
import signal
import struct
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Any

import numpy as np
import zmq
from Basilisk.utilities import macros

from sim.eds_link import (
    BurnCommand,
    CamConfig,
    StepAck,
    decode_burn_command,
    decode_cam_config,
    decode_step_ack,
    decode_torque_command,
    encode_sensor_frame,
    encode_torque_command,
    encode_truth_state,
)
from sim.parts import load_registry

# Ports: the ground/sim split taught both sides. The sim
# BINDS everything: it starts and dies far more often than the mission.
TOPIC_SENSOR = b"SENSOR"
TOPIC_ACK = b"ACK"
TOPIC_CMD = b"CMD"
# Camera config: egressed on the cmd socket like CMD, but not part
# of the barrier - _await_barrier applies it and keeps waiting. It lives with
# the hardcoded constants because it is neither physics nor a registry part:
# a command surface, fire-and-forget by contract.
TOPIC_CAM = b"CAM"
# Burn: egressed on the cmd socket like CAM, its own topic, never gated. It
# joins the hardcoded barrier constants for the same reason - a command
# surface, not a registry part. Unlike CAM it is APPLY-ONCE: each frame buys
# exactly one tick of thrust (the on-times a frame produces never outlive the
# tick), so a craft that goes silent mid-burn stops thrusting on the next
# tick. A lost frame is a short burn, never an endless one.
TOPIC_BURN = b"BURN"
TOPIC_TRUTH = b"TRUTH"
# The boot handshake: the world's declared part fit, on the sensor socket
# every tick. Not a part and not telemetry - it is the link's own control
# chatter, so it joins the barrier constants rather than OBSERVER_TOPICS, and
# its EDS container has no TopicId: no MsgId, no ground def, no beacon.
TOPIC_FIT = b"FIT"
# Observability topics the rig may publish: the landed bsk_link rows
# of registry/parts.toml - the same table the spacecraft's generic consumer
# reads, so sim and flight software can never disagree about which parts
# exist. The constants above stay hardcoded on purpose: SENSOR/ACK/CMD are
# the barrier (physics) and TRUTH is the navigator's wire - neither is a
# part, neither lands through the registry.
OBSERVER_TOPICS = frozenset(
    p.zmq_topic for p in load_registry() if p.landed and p.consumer == "bsk_link"
)
SENSOR_ENDPOINT = "tcp://*:5580"
CMD_ENDPOINT = "tcp://*:5581"
TRUTH_ENDPOINT = "tcp://*:5590"
ECHO_ENDPOINT = "tcp://*:5591"

# Wall-clock seconds the lockstep gate waits for StepAck{N} before aborting.
# Generous on purpose: the first frame races bsk_link's SUB reconnect and its
# 0.5 s drain cycle, and an abort is loud either way - the timeout only picks
# loud-after-how-long, never whether the run fails.
ACK_TIMEOUT_S = 10.0

# The check retransmits sensor{N} at this interval while it waits. A fresh ZMQ
# PUB/SUB pair silently eats frames published before the subscription lands -
# measured here: the mission's SUB connects before the sim's PUB exists, so
# frame 1 dies exactly when a lockstep run needs it most. Retransmission makes
# the barrier a real handshake (shout until the shout lands), and fcs answers a
# repeat frame idempotently, so control-law runs and counters stay once-per-
# frame. Mid-run this also lets a mission RESTART heal the check within the ack
# timeout instead of dooming the run.
#
# Retransmission heals a LATE subscriber; it cannot heal an EARLY one. Measured
# (FIT probes, 2026-11-25): when the craft dials a port whose listener has
# already exited, the kernel completes the connect into a dead backlog - the
# SUB believes itself connected forever, libzmq never re-attaches, and the
# re-shout has no peer to reach: a green-booting craft receiving nothing,
# forever. That is what ``LockstepRig._await_subscribers`` exists to forbid.
ACK_RETRY_S = 0.5

# The first-subscriber gate's ceiling (see _await_subscribers). Generous
# because a cold compose boot is honestly slow; loud because a timeout there
# means the barrier is about to fly against a wall.
SUBSCRIBER_WAIT_S = 90.0


class LockStalled(RuntimeError):
    """The mission stopped acknowledging frames: the barrier cannot open."""


def _flat(vec: object) -> np.ndarray:
    """Flatten a Basilisk message vector (nested ``[[x],[y]]`` or flat) to numpy."""
    arr = np.asarray(vec, dtype=float)
    return arr.reshape(-1)


def _vec3(vec: np.ndarray) -> tuple[float, float, float]:
    """A payload-ready 3-tuple of plain floats (EdsLib rejects numpy scalars)."""
    flat = _flat(vec)
    if flat.shape != (3,):
        raise ValueError(f"expected a 3-vector, got shape {flat.shape}")
    return (float(flat[0]), float(flat[1]), float(flat[2]))


@dataclass(frozen=True)
class Truth:
    """One instant of the simulator's own solution, before any sensor error.

    The burn ledger is the WORLD's applied tally, not the craft's claim:
    what the plant's thrusters were actually given, tick by tick. Flight
    software never reads these bytes; the bench reads both ledgers and owns
    the difference. (Rung 1; counts ride the truth wire's float64 exactly.)
    """

    sim_time_s: float
    r_bn_n: tuple[float, float, float]
    v_bn_n: tuple[float, float, float]
    sigma_bn: tuple[float, float, float, float]
    omega_bn_b: tuple[float, float, float]
    burn_impulse_applied_ns: float = 0.0
    burn_ticks_applied: float = 0.0
    burn_ticks_saturated: float = 0.0
    burn_id: float = 0.0
    fuel_remaining_kg: float = float("nan")
    fuel_spent_kg: float = float("nan")
    fuel_ticks_starved: float = 0.0
    fuel_ticks_throttled: float = 0.0


def truth_bytes(truth: Truth) -> bytes:
    """Encode a :class:`Truth` as the ``BSK_LINK/TruthState`` wire frame."""
    return bytes(
        encode_truth_state(
            truth.sim_time_s,
            truth.r_bn_n,
            truth.v_bn_n,
            truth.sigma_bn,
            truth.omega_bn_b,
            burn_impulse_applied_ns=truth.burn_impulse_applied_ns,
            burn_ticks_applied=truth.burn_ticks_applied,
            burn_ticks_saturated=truth.burn_ticks_saturated,
            burn_id=truth.burn_id,
            fuel_remaining_kg=truth.fuel_remaining_kg,
            fuel_spent_kg=truth.fuel_spent_kg,
            fuel_ticks_starved=truth.fuel_ticks_starved,
            fuel_ticks_throttled=truth.fuel_ticks_throttled,
        )
    )


def trajectory_sha256(rate_history: list[tuple[float, float]]) -> str:
    """The determinism fingerprint: the whole measured trajectory.

    Time and rate, hashed. Two seeded runs on the same binary MUST print the
    same digest - single-scan stepping in the caller's thread makes the
    trajectory a pure function of the seed, with no wall-clock races left.
    """
    digest = hashlib.sha256()
    for tick_time, rate in rate_history:
        digest.update(struct.pack("<dd", tick_time, rate))
    return digest.hexdigest()


def bind_link(
    sensor_endpoint: str = SENSOR_ENDPOINT,
    cmd_endpoint: str = CMD_ENDPOINT,
    truth_endpoint: str = TRUTH_ENDPOINT,
    echo_endpoint: str = ECHO_ENDPOINT,
) -> dict[str, zmq.Socket]:
    """Bind the Basilisk-side sockets: PUB sensor/truth/command-echo, PULL commands.

    The PULL on ``cmd_endpoint`` carries flight software's torque frames AND
    the lockstep barrier acks (topics ``CMD``/``ACK``, dispatched in
    ``LockstepRig._await_barrier``); standalone runs leave it idle. The rig
    publishes the torque it actually applied on ``echo_endpoint``, so a test or
    a human watches exactly what Basilisk felt, in either mode.
    """
    context = zmq.Context.instance()
    bound = {
        "sensor": context.socket(zmq.PUB),
        "cmd": context.socket(zmq.PULL),
        "truth": context.socket(zmq.PUB),
        "echo": context.socket(zmq.PUB),
    }
    endpoints = (sensor_endpoint, cmd_endpoint, truth_endpoint, echo_endpoint)
    for sock, endpoint in zip(bound.values(), endpoints, strict=True):
        sock.bind(endpoint)
    return bound


@dataclass
class RigChannel:
    """One gated vehicle in the world: its callbacks and its link sockets.

    The frame counter and the clock belong to the WORLD (the rig); the
    counters and histories belong to a channel - whose check shouted,
    stalled, or answered is a per-vehicle fact, and a fleet run reports
    them side by side.
    """

    name: str
    gyro: Callable[[], np.ndarray]
    apply_torque: Callable[[np.ndarray], None]
    truth_provider: Callable[[], bytes]
    kp: np.ndarray
    # Registry-driven observability: one provider per registered
    # topic, publish order = insertion order (determinism: the digest
    # ledger depends on the world never racing itself). A provider may
    # return one payload or a LIST of them: a multi-instance row (N eyes)
    # rides its topic once per instance, in the provider's order. Keys must
    # be landed registry rows' topics - see __post_init__.
    observers: dict[str, Callable[[], bytes | list[bytes]]] = field(default_factory=dict)
    # The boot handshake: this vehicle's declared fit, published on the
    # sensor socket every tick so the spacecraft can compare three views - the
    # manifest, what the binary was BUILT with, and what its bus actually
    # streams. ``None`` means the scenario says nothing about its fit, and the
    # mission reports the link as legacy (no presence verdict either way).
    fit_provider: Callable[[], bytes] | None = None
    # The camera config door: called with each decoded CamConfig
    # the instant it arrives, between ticks - the module write lands at the
    # next Basilisk update by the sim's own law (camera.cpp re-reads its
    # fields every tick, proven by an early spike). ``None`` means this
    # vehicle has no commanded eye; a config for it is decoded, counted, and
    # dropped - visible in the exit report, never silently absorbed.
    cam_sink: Callable[[CamConfig], None] | None = None
    # The burn door: called with each decoded BurnCommand the instant it
    # arrives, between ticks - the frame buys exactly the NEXT tick (apply-
    # once, see TOPIC_BURN). ``None`` means this vehicle declares no
    # thrusters; the frame is decoded, counted here, and dropped - an
    # absence on the exit tally, never a silently absorbed impulse.
    burn_sink: Callable[[BurnCommand], None] | None = None
    sockets: dict[str, zmq.Socket] = field(default_factory=dict)

    stale_torques: int = 0  # frames (torque or ack) the check already passed
    sensor_retransmits: int = 0  # check shouts that had to be repeated
    foreign_frames: int = 0  # frames ahead of the check: a protocol breach
    bad_frames: int = 0  # undecodable or malformed multipart frames
    cam_configs: int = 0  # camera configs received (applied is the sink's tally)
    burn_frames: int = 0  # burn frames received (applied is the sink's tally)
    rate_history: list[tuple[float, float]] = field(default_factory=list)
    torque_history: list[tuple[float, np.ndarray]] = field(default_factory=list)
    external_torque_history: list[tuple[float, np.ndarray | None]] = field(default_factory=list)

    def __post_init__(self) -> None:
        """A channel may only publish what the registry has landed."""
        unknown = [topic for topic in self.observers if topic not in OBSERVER_TOPICS]
        if unknown:
            raise ValueError(
                f"{self.name}: observer topics {unknown} are not landed registry parts - "
                "the rig publishes only what registry/parts.toml lands"
            )


class LockstepRig:
    """The tick/barrier/stepper half of a orrery scenario.

    A scan of the sim is the metronome; a tick is what happens AFTER the scan
    lands on a control boundary: for every channel measure with ``gyro`` and
    publish the sensor frame (it IS the tick), then wait at each gate for
    that vehicle's ``fcs`` to answer THIS frame, write every answered torque
    (or zero - a disabled loop is inert, frame-matched), and only then let
    the next scan exist. One world step therefore needs one ack from every
    mission container; a dead container stalls the world, loudly.
    In standalone mode the barriers are absent and each internal law
    ``-kp * omega`` closes its own loop.
    """

    def __init__(
        self,
        sim: Any,  # noqa: ANN401 - Basilisk SimBaseClass (stub-less SWIG)
        *,
        control_period_s: float,
        gyro: Callable[[], np.ndarray] | None = None,
        apply_torque: Callable[[np.ndarray], None] | None = None,
        truth_provider: Callable[[], bytes] | None = None,
        kp: np.ndarray | None = None,
        channels: list[RigChannel] | None = None,
        lockstep: bool = False,
        ack_timeout_s: float = ACK_TIMEOUT_S,
        ack_retry_s: float = ACK_RETRY_S,
        pace: bool = False,
        observers: dict[str, Callable[[], bytes | list[bytes]]] | None = None,
        fit_provider: Callable[[], bytes] | None = None,
        cam_sink: Callable[[CamConfig], None] | None = None,
        burn_sink: Callable[[BurnCommand], None] | None = None,
    ) -> None:
        """Wire the channels; the plant(s) must already be built and initialized.

        The classic single-vehicle form passes the loose callbacks (``gyro``,
        ``apply_torque``, ``truth_provider``, ``kp``, optional
        ``observers``) - DetumbleScenario, ForgeScenario and every frozen
        digest enter through that door, and get exactly one channel. The
        fleet form passes ready ``channels`` instead.

        ``control_period_s`` must be an integer multiple of the sim's scan
        grid (the gcd of its task rates) or ticks would silently skip
        boundaries - checked up front AND guarded at runtime.
        """
        if channels is None:
            missing = [
                label
                for label, value in (
                    ("gyro", gyro),
                    ("apply_torque", apply_torque),
                    ("truth_provider", truth_provider),
                    ("kp", kp),
                )
                if value is None
            ]
            if missing:
                raise ValueError(f"single-channel rig needs {', '.join(missing)} (or channels)")
            assert gyro is not None and apply_torque is not None
            assert truth_provider is not None and kp is not None
            channels = [
                RigChannel(
                    name="solo",
                    gyro=gyro,
                    apply_torque=apply_torque,
                    truth_provider=truth_provider,
                    kp=np.asarray(kp, dtype=float),
                    observers=observers or {},
                    fit_provider=fit_provider,
                    cam_sink=cam_sink,
                    burn_sink=burn_sink,
                )
            ]
        if not channels:
            raise ValueError("a rig without channels commands nothing")

        self.sim = sim
        self.channels = channels
        self.control_period_s = control_period_s
        self.lockstep = lockstep
        self.ack_timeout_s = ack_timeout_s
        self.ack_retry_s = ack_retry_s
        self.pace = pace

        self.frame = 0  # ONE frame counter for the whole world
        self._link_ready = False  # the first-subscriber gate has been through
        # perpetual mode: requested anytime, honored at a boundary
        self._stop = threading.Event()

        grid_ns = self._scan_grid_ns()
        period_ns = macros.sec2nano(control_period_s)
        if grid_ns and period_ns % grid_ns:
            raise ValueError(
                f"control period {control_period_s:g}s is not an integer multiple of the "
                f"sim scan grid {grid_ns * 1e-9:g}s; ticks would fall between scans"
            )

    # ---- single-vehicle surface: the pre-fleet harness, unchanged ----------

    @property
    def stale_torques(self) -> int:
        """Tripwire across all channels: answers from frames the check passed."""
        return sum(ch.stale_torques for ch in self.channels)

    @property
    def sensor_retransmits(self) -> int:
        """Handshakes the checks had to repeat, summed over channels."""
        return sum(ch.sensor_retransmits for ch in self.channels)

    @property
    def foreign_frames(self) -> int:
        """Tripwire across all channels: answers from frames ahead of the check."""
        return sum(ch.foreign_frames for ch in self.channels)

    @property
    def bad_frames(self) -> int:
        """Tripwire across all channels: undecodable or malformed frames."""
        return sum(ch.bad_frames for ch in self.channels)

    @property
    def burn_frames(self) -> int:
        """Burn frames received across all channels (applied is the bodies' tally)."""
        return sum(ch.burn_frames for ch in self.channels)

    @property
    def rate_history(self) -> list[tuple[float, float]]:
        """The first channel's trajectory (the digest stream, pre-fleet form)."""
        return self.channels[0].rate_history

    @property
    def torque_history(self) -> list[tuple[float, np.ndarray]]:
        """The first channel's applied torques."""
        return self.channels[0].torque_history

    @property
    def external_torque_history(self) -> list[tuple[float, np.ndarray | None]]:
        """The first channel's fcs answers (None = disabled loop that frame)."""
        return self.channels[0].external_torque_history

    def _scan_grid_ns(self) -> int:
        """GCD of every task rate, or 0 when the sim does not expose tasks.

        0 means "trust the plant builder": hand-written scenarios register a
        single task at the control period, which is the pre-rig behaviour.
        """
        rates = [
            int(task.TaskData.TaskPeriod) for task in (getattr(self.sim, "TaskList", None) or [])
        ]
        return math.gcd(*rates) if rates else 0

    @property
    def sim_time_s(self) -> float:
        """The sim instant this process is commanding at.

        A scan sets ``CurrentNanos`` to the boundary it just fired at, which is
        the instant of the tick that follows it (see ``run_to``: fire, then
        command). This is also exactly where chunk stepping left the clock.
        """
        return self.sim.TotalSim.CurrentNanos * 1e-9

    def attach_link(self, sockets: dict[str, zmq.Socket]) -> None:
        """Publish every link frame on the given bound sockets - channel 0.

        The single-vehicle surface (pre-fleet scripts and scenarios call
        it directly); a fleet wires one socket set per channel.
        """
        self.attach_channel(0, sockets)

    def attach_channel(self, index: int, sockets: dict[str, zmq.Socket]) -> None:
        """Give one channel its socket set (sensor/cmd/truth/echo)."""
        self.channels[index].sockets = sockets

    def close(self) -> None:
        """Drop every channel's link sockets (no shutdown frames)."""
        for ch in self.channels:
            for sock in ch.sockets.values():
                sock.close(linger=250)  # let the last publishes drain (P3d linger lesson)
            ch.sockets = {}

    def flush_link(self) -> None:
        """Discard whatever the command queues still hold from a previous run.

        A PUSH socket redelivers its surviving queue to the next PULL that
        binds the port, and those frames carry the *previous* scenario's frame
        numbers; nothing in this run may mistake them for its own barrier.
        """
        for ch in self.channels:
            sock = ch.sockets.get("cmd")
            if sock is None:
                continue
            while sock.poll(0, zmq.POLLIN) == zmq.POLLIN:
                try:
                    sock.recv_multipart(flags=zmq.NOBLOCK)
                except zmq.Again:  # pragma: no cover - poll said readable
                    break

    def _await_subscribers(self, timeout_s: float = SUBSCRIBER_WAIT_S) -> bool:
        """Hold the first tick until every gated vehicle has a LIVE peer.

        The barrier retransmits (ACK_RETRY_S) to heal a mission that
        subscribes LATE. It cannot heal one that subscribed EARLY to nobody:
        the kernel completes a connect to a just-departed listener's backlog,
        libzmq considers the peer established, and the re-shout reaches no
        one (measured 2026-11-25: boots green, receive zero, stall frame 1).
        Only the listener can break that phantom peer, by existing before
        the dial. So the world binds first and waits for the craft to knock:
        a monitor on the sensor listener reports ACCEPTED + HANDSHAKE_SUCCEEDED
        per peer - a real, ZMTP-verified connection, not a kernel backlog
        maybe. One proof per vehicle, in channel order: a fleet whose first
        craft answers is a fleet whose listener existed before any dial, and
        the remaining crafts' re-shouts are the barrier's own healing. A
        timeout is not fatal - the tick proceeds and the barrier stalls
        loudly with its named message - but it prints first, because "the
        world waited 90 s and nobody dialed" is the sentence an operator needs.
        """
        for ch in self.channels:
            sensor = ch.sockets.get("sensor")
            if sensor is None:
                continue
            endpoint = f"inproc://orrery-peer-gate-{id(ch)}"
            monitor = sensor.get_monitor_socket(events=zmq.EVENT_ACCEPTED, addr=endpoint)
            deadline = time.monotonic() + timeout_s
            accepted = False
            try:
                while time.monotonic() < deadline:
                    if monitor.poll(250, zmq.POLLIN):
                        monitor.recv()
                        accepted = True
                        break
            finally:
                sensor.disable_monitor()
                monitor.close()
            if accepted:
                continue
            print(
                f"WAIT TIMEOUT: no live peer dialed channel {ch.name} within {timeout_s:g}s"
                " - the barrier may stall on frame 1 (craft down? port squatted?)",
                flush=True,
            )
            return False
        return True

    def _await_barrier(self, ch: RigChannel, frame: int, sensor_bytes: bytes) -> np.ndarray | None:
        """The lockstep barrier: hold until ``fcs`` acks this frame.

        Returns the torque commanded for *this* frame, or ``None`` when none
        came - the loop was disabled, yet the ack still arrives and time moves
        on. Frames from the past are counted and dropped (tripwire, not
        recovery logic); a frame from the future cannot exist in lockstep and
        aborts as a protocol breach. No ack before the deadline raises
        ``LockStalled``: that satellite stopped reacting, and the alternative -
        keep flying on stale knowledge - is what this check exists to forbid.
        In a fleet the stall message names the channel: one dead container is
        one named mission, not an anonymous hang.

        While waiting, the sensor frame is RE transmitted every ``ack_retry_s``
        (ZMQ's first-frame-eaten-by-subscription-handshake makes a bare single
        publish deadlock the very first tick); ``fcs`` answers repeats without
        re-running the law, so retransmission costs nothing but saves the run.

        Ordering is load-bearing and real: fcs publishes torque{N} before
        StepAck{N}, both ride the one PUSH socket, and ZeroMQ preserves order
        per pair - so torque{N} is already queued when the ack is read.
        """
        sock = ch.sockets["cmd"]
        sensor_sock = ch.sockets["sensor"]
        # A single-vehicle run stalls with the classic message; only a fleet
        # can leave the operator asking WHICH check stalled, so only then does
        # the message name the channel.
        who = f" from {ch.name}" if len(self.channels) > 1 else ""
        now = time.monotonic()
        deadline = now + self.ack_timeout_s
        next_shout = now + self.ack_retry_s
        torque_for_frame: np.ndarray | None = None
        while True:
            try:
                parts = sock.recv_multipart(flags=zmq.NOBLOCK)
            except zmq.Again:
                now = time.monotonic()
                if now >= deadline:
                    raise LockStalled(
                        f"no StepAck{who} for frame {frame} within "
                        f"{self.ack_timeout_s:g}s (mission container down? "
                        "see FCS_STEP_ACK counters)"
                    ) from None
                if now >= next_shout:
                    sensor_sock.send_multipart([TOPIC_SENSOR, sensor_bytes])
                    ch.sensor_retransmits += 1
                    next_shout = now + self.ack_retry_s
                time.sleep(0.0005)  # half-ms poll; the ack's trip time is ms-scale
                continue
            if len(parts) != 2:
                ch.bad_frames += 1
                continue
            topic, body = parts
            try:
                if topic == TOPIC_CAM:
                    # Fire-and-forget: apply the config, keep
                    # waiting at the gate. A camera answer must never open a
                    # physics barrier - that law survived from v1 unchanged.
                    cfg = decode_cam_config(body)
                    ch.cam_configs += 1
                    if ch.cam_sink is not None:
                        ch.cam_sink(cfg)
                    continue
                if topic == TOPIC_BURN:
                    # The same fire-and-forget law as CAM - a burn answer
                    # never opens a barrier - but apply-ONCE physics: this
                    # frame buys the next tick only, never a held state.
                    burn = decode_burn_command(body)
                    ch.burn_frames += 1
                    if ch.burn_sink is not None:
                        ch.burn_sink(burn)
                    continue
                if topic == TOPIC_CMD:
                    cmd = decode_torque_command(body)
                elif topic == TOPIC_ACK:
                    cmd = decode_step_ack(body)
                else:
                    ch.bad_frames += 1
                    continue
            except Exception:
                ch.bad_frames += 1
                continue
            if isinstance(cmd, StepAck):
                if cmd.frame == frame:
                    return torque_for_frame
                if cmd.frame < frame:
                    ch.stale_torques += 1  # acks obey the same staleness bookkeeping
                    continue
                ch.foreign_frames += 1
                raise LockStalled(
                    f"StepAck{who} for future frame {cmd.frame} while gating "
                    f"{frame}: frame counters disagree between sim and flight "
                    "software"
                )
            if cmd.frame == frame:
                torque_for_frame = np.asarray(cmd.torque_sum_b, dtype=float)
            elif cmd.frame < frame:
                ch.stale_torques += 1
            else:
                ch.foreign_frames += 1

    def control_tick(self) -> np.ndarray:
        """One world tick: measure all -> publish all -> check all -> apply all.

        With ``lockstep`` each vehicle's control law lives in its own flight
        software: publish every sensor frame first (they *are* the tick, and
        every reading is taken before any answer can perturb anything), let
        each check return whatever its fcs commanded for exactly this frame,
        then apply every torque and integrate - the satellites' compute takes
        zero sim time, which is what makes the loop both deterministic and
        one-way dependent: the sim steps on flight-software answers, never the
        other way round. Standalone mode keeps each channel's internal
        ``-kp * omega``.

        Channels are gated (and their torques applied) in fixed channel
        order, so a fleet run is as deterministic as a single one: the world
        step depends on every container's answer, but on no wall-clock race.
        Returns the torque applied to the FIRST channel - the single-vehicle
        contract fleets do not use (they read per-channel histories).
        """
        if self.lockstep and not self._link_ready and any(ch.sockets for ch in self.channels):
            # frame 0 is the only tick that can be published into a phantom
            # subscription, or into a run whose craft has not booted yet (the
            # 10 s barrier deadline is shorter than a craft boot - the world
            # must not be the first to give up): gate it. The env knob exists
            # for the harness tests, which dial before they tick.
            wait_s = float(os.environ.get("ORRERY_FIRST_TICK_WAIT_S", SUBSCRIBER_WAIT_S))
            if wait_s > 0:
                self._await_subscribers(wait_s)
            self._link_ready = True
        samples: list[tuple[RigChannel, np.ndarray, bytes]] = []
        for ch in self.channels:
            omega_meas = np.asarray(ch.gyro(), dtype=float)
            samples.append((ch, omega_meas, b""))
        self.frame += 1
        sim_time_s = self.sim_time_s

        published: list[tuple[RigChannel, np.ndarray, bytes]] = []
        for ch, omega_meas, _ in samples:
            sensor_bytes = b""
            if ch.sockets:
                if ch.fit_provider is not None:
                    # Declared fit FIRST, on the same socket: ZeroMQ preserves
                    # order per pair, so the spacecraft always hears what the
                    # world claims before it sees the frames that claim covers.
                    ch.sockets["sensor"].send_multipart([TOPIC_FIT, ch.fit_provider()])
                sensor_bytes = bytes(encode_sensor_frame(sim_time_s, _vec3(omega_meas), self.frame))
                ch.sockets["sensor"].send_multipart([TOPIC_SENSOR, sensor_bytes])
                ch.sockets["truth"].send_multipart([TOPIC_TRUTH, ch.truth_provider()])
                for topic, provider in ch.observers.items():
                    # Observability: registry-driven, sampled at the same
                    # instant as the sensor frame,
                    # on the same socket, and NO check waits for it - fcs never
                    # sees knowledge through this path. Knowledge flows,
                    # physics stays gated. A multi-instance row (an eye per
                    # frame) rides its topic once per payload, in order.
                    sampled = provider()
                    for payload in sampled if isinstance(sampled, list) else [sampled]:
                        ch.sockets["sensor"].send_multipart([topic.encode("ascii"), payload])
            published.append((ch, omega_meas, sensor_bytes))

        gated: list[tuple[RigChannel, np.ndarray, np.ndarray]] = []
        for ch, omega_meas, sensor_bytes in published:
            if self.lockstep:
                external = self._await_barrier(ch, self.frame, sensor_bytes)
                ch.external_torque_history.append(
                    (sim_time_s, None if external is None else external.copy())
                )
                torque = np.zeros(3) if external is None else external
            else:
                torque = -ch.kp * omega_meas
            gated.append((ch, omega_meas, torque))

        # Every check answered before any torque is written: a tick that
        # stalls changes nothing the next scan could integrate.
        applied_first: np.ndarray | None = None
        for ch, omega_meas, torque in gated:
            ch.apply_torque(torque)
            ch.rate_history.append((sim_time_s, float(np.linalg.norm(omega_meas))))
            ch.torque_history.append((sim_time_s, torque.copy()))
            if ch.sockets:
                # The torque actually applied, echoed for ground and tests:
                # what Basilisk felt on this body, frame-matched to its
                # command stream.
                ch.sockets["echo"].send_multipart(
                    [TOPIC_CMD, bytes(encode_torque_command(sim_time_s, _vec3(torque), self.frame))]
                )
            if applied_first is None:
                applied_first = torque
        assert applied_first is not None  # channels is non-empty by construction
        return applied_first

    def request_stop(self) -> None:
        """Ask a perpetual run to end (signal-safe: sets a flag, nothing else).

        The world keeps ticking until the CURRENT tick completes - every
        container acked, every torque applied - and only then does ``run_to``
        return. Mid-check abandonment is the failure this harness exists to
        prevent; a stop request does not exempt itself from that rule.
        """
        self._stop.set()

    @property
    def stop_requested(self) -> bool:
        """Whether a stop request is pending (or ended the run)."""
        return self._stop.is_set()

    def run_to(self, sim_time_s: float) -> None:
        """Advance the dynamics to ``sim_time_s``, one control tick per period.

        ``TotalSim.SingleStepProcesses()`` is Basilisk's single-scan primitive:
        every model due at the current sim instant fires exactly once - the
        dynamics integrating up to that boundary and writing state there - and
        the clock advances. Fire-then-command is the order that reads true: the
        sensor at boundary k sees the state the scan at k just produced, and
        torque commanded at tick k is consumed by the scan at k+1, applied over
        ``[k, k+1)`` - the clean one-sample hold. A plant scanned FASTER than
        the control period (bskforge's dynamics/fsw split rates, W3) simply
        ticks only when a scan lands on a control boundary; the intervening
        scans integrate without commanding.

        The controller - and in lockstep, the barrier - sits between scans, so
        the sim owns time absolutely: nothing across the link can accelerate
        it, and the wall clock only matters if ``--pace`` asked for it.
        ``--pace`` (the human watch layer) sleeps after the check so 1 sim-s
        takes 1 real s. It throttles the barrier, never replaces it.

        ``sim_time_s = inf`` is **perpetual mode**: no horizon, the
        loop runs until ``request_stop()`` - honored only after a complete
        world tick, so Ctrl-C / SIGTERM end a fleet run the same way a
        duration does: at a boundary, with every check answered.
        """
        if self._stop.is_set():
            return
        horizon = None if math.isinf(sim_time_s) else macros.sec2nano(sim_time_s)
        period_ns = macros.sec2nano(self.control_period_s)
        wall_start = time.monotonic()
        prev_boundary_index = -1
        while horizon is None or self.sim.TotalSim.NextTaskTime <= horizon:
            self.sim.TotalSim.SingleStepProcesses()
            current_ns = self.sim.TotalSim.CurrentNanos
            if horizon is not None and current_ns >= horizon:
                break  # final integration done; no tick beyond the horizon
            boundary_index = current_ns // period_ns
            if current_ns % period_ns:
                # Off-boundary scan: a fast-dynamics plant integrating toward
                # the next control instant. If the scan index jumped over a
                # whole boundary, the grid does not divide the period after
                # all - landing exactly on boundaries is the tick contract.
                if boundary_index - prev_boundary_index > 1:
                    raise RuntimeError(
                        f"scans skipped control boundary {boundary_index}: "
                        "scan grid does not divide the control period"
                    )
                continue
            if boundary_index == prev_boundary_index:
                continue  # boundary already ticked; scans never rewind, belt and braces
            prev_boundary_index = boundary_index
            self.control_tick()
            if self.pace:
                lag = wall_start + self.sim_time_s - time.monotonic()
                if lag > 0:
                    time.sleep(lag)
            if self._stop.is_set():
                return  # perpetual: the tick completed - a clean boundary exit


def install_stop_handlers(rig: LockstepRig) -> None:
    """End a perpetual run on Ctrl-C (SIGINT) or service stop (SIGTERM).

    The handler only raises the rig's stop flag - honored at the next
    control boundary, after every container has acked - so ``fleet.sh down``
    (SIGTERM) and an operator's Ctrl-C both end the world the way a finite
    run ends it: complete tick, clean sockets, digest printed. A run stuck
    at a dead check still exits through ``LockStalled``, exactly as with a
    duration; ``kill -9`` remains the rude door.
    """

    def _request_stop(_signum: int, _frame: object) -> None:
        rig.request_stop()  # flag only - signal-safe by construction

    signal.signal(signal.SIGINT, _request_stop)
    signal.signal(signal.SIGTERM, _request_stop)
