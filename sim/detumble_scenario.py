"""Detumble scenario: a tumbling rigid body that damps its own rate (ROADMAP P3a).

Runs Basilisk's rigid-body dynamics with an initial tumble and closes the attitude
rate loop through the mission's own link schema: every control tick the scenario
takes a ``BSK_LINK/SensorFrame`` of what the *onboard gyro* knows (biased and noisy,
never truth), and applies a torque - standalone, a proportional rate-damping law
runs here; with ``--zmq`` the law lives in ``fcs``, and this file only measures,
publishes, gates and actuates what flight software commanded. Exit criterion: the
rate norm collapses from its initial tumble and settles on the gyro noise floor.

The barrier, the sockets and the stepping discipline (D5 amendment) live in
``sim/lockstep.py`` - one gate implementation that every orrery plant rides,
hand-written here or bskforge-built in ``sim/forge_scenario.py``::

    --zmq   SENSOR  PUB  :5580   BSK_LINK/SensorFrame    (onboard knowledge, the tick)
            CMD     PULL :5581   BSK_LINK/TorqueCommand  (fcs torque, per frame)
            ACK     PULL :5581   BSK_LINK/StepAck        (lockstep barrier, D5 amendment)
            CMD_ECHO PUB :5591   BSK_LINK/TorqueCommand  (the torque actually applied)
            TRUTH   PUB  :5590   BSK_LINK/TruthState     (P5 ground only, never cFS)

Truth goes out on its own channel and never on the sensor feed: what the satellite
knows and what the simulator knows stay separate, which is what the two COSMOS targets
in P5 exist to show.

    uv run python -m sim.detumble_scenario --duration 120 --zmq

Without ``--zmq`` nothing binds a port and the internal law keeps the loop: the
physics model stays testable with zero infrastructure.
"""

from __future__ import annotations

import argparse

import numpy as np
from Basilisk.architecture import messaging
from Basilisk.simulation import extForceTorque, spacecraft
from Basilisk.utilities import SimulationBaseClass, macros, simHelpers

from sim.fleet import add_link_args, resolve_endpoints

# Link-side names re-exported: tools and tests import them from the scenario
# module, but the contract itself lives in sim.lockstep.
from sim.lockstep import (
    ACK_TIMEOUT_S,
    CMD_ENDPOINT,  # noqa: F401 - re-export
    SENSOR_ENDPOINT,  # noqa: F401 - re-export
    TRUTH_ENDPOINT,  # noqa: F401 - re-export
    LockStalled,
    LockstepRig,
    Truth,
    bind_link,
    install_stop_handlers,
    trajectory_sha256,
    truth_bytes,
)

MODEL_TAG = "chiefBus"
SIM_PROCESS = "dynProcess"
CONTROL_TASK = "ctrlTask"

# Hub: asymmetric inertia so a tumble actually couples axes.
INERTIA_DIAG = (750.0, 620.0, 540.0)
MASS_KG = 300.0
INITIAL_RATE_RADS = (0.05, -0.04, 0.03)
INITIAL_MRP = (0.05, -0.08, 0.11)

# Gyro error model. Truth stays in the dynamics; the controller only ever sees this.
GYRO_BIAS_RADS = np.array([2.0e-5, -1.5e-5, 1.0e-5])
GYRO_NOISE_SIGMA_RADS = 1.0e-4

# Standalone rate damping: torque = -Kp * omega_measured. The per-axis closed-loop
# time constant is about I/Kp, so these gains put tau near 12 s on this inertia:
# comfortably inside the acceptance window, and still far below the Kp = I limit at
# which a 1 Hz command update goes unstable. These are also fcs_app's FCS_DEFAULT_KP
# and the chief TOML scenario's law - three names, one intent, pinned by tests.
KP_RADS = np.array([60.0, 50.0, 45.0])

# Detumble acceptance: rate norm below this fraction of the initial tumble.
CONVERGENCE_RATIO = 0.1


class DetumbleScenario:
    """Basilisk rigid body + gyro error model, ridden by the shared lockstep rig."""

    def __init__(
        self,
        control_period_s: float = 1.0,
        seed: int = 20261120,
        gyro_noise_sigma_rads: float = GYRO_NOISE_SIGMA_RADS,
        gyro_bias_rads: np.ndarray = GYRO_BIAS_RADS,
        kp_rads: np.ndarray = KP_RADS,
        lockstep: bool = False,
        ack_timeout_s: float = ACK_TIMEOUT_S,
        ack_retry_s: float = 0.5,
        pace: bool = False,
    ) -> None:
        """Build the rigid body, the torque effector and the gyro error model.

        ``lockstep`` moves the control law to flight software: each tick
        publishes its sensor frame, then holds the next Basilisk scan until
        ``fcs`` answers that frame (the rig's barrier). Standalone runs and the
        unit tests keep the internal law; there is no third mode.
        """
        self.kp = np.asarray(kp_rads, dtype=float)
        self._bias = np.asarray(gyro_bias_rads, dtype=float)
        self._noise_sigma = gyro_noise_sigma_rads
        self._rng = np.random.default_rng(seed)

        self.sim = SimulationBaseClass.SimBaseClass()
        process = self.sim.CreateNewProcess(SIM_PROCESS)
        process.addTask(self.sim.CreateNewTask(CONTROL_TASK, macros.sec2nano(control_period_s)))

        self.sc = spacecraft.Spacecraft()
        self.sc.ModelTag = MODEL_TAG
        self.sc.hub.mHub = MASS_KG
        self.sc.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
        self.sc.hub.IHubPntBc_B = simHelpers.np2EigenMatrix3d(np.diag(INERTIA_DIAG).ravel())
        self.sc.hub.r_CN_NInit = [7.0e6, 0.0, 0.0]
        self.sc.hub.v_CN_NInit = [0.0, 7.5461, 0.0]
        self.sc.hub.sigma_BNInit = [[v] for v in INITIAL_MRP]
        self.sc.hub.omega_BN_BInit = [[v] for v in INITIAL_RATE_RADS]

        self.effector = extForceTorque.ExtForceTorque()
        self.effector.ModelTag = "torqueEffector"
        self.sc.addDynamicEffector(self.effector)

        # Task order is a physics decision: models update in registration order,
        # and ExtForceTorque converts its input message into effector force at
        # its own update. Registered BEFORE the spacecraft, a scan reads the
        # torque commanded for interval [k, k+1) and the spacecraft's update in
        # that same scan integrates with it. Registered after, every actuation
        # would lag one extra period. (The old chunk-stepped loop fired at both
        # ends of each interval, which accidentally papered this over.)
        self.sim.AddModelToTask(CONTROL_TASK, self.effector)
        self.sim.AddModelToTask(CONTROL_TASK, self.sc)

        # The effector input is written by the controller once per control tick.
        self._torque_msg = messaging.CmdTorqueBodyMsg()
        self.effector.cmdTorqueInMsg.subscribeTo(self._torque_msg)

        # Onboard sensor: the scenario reads the state message and adds its error
        # model, the same role bsk_link will play with real sensor messages later.
        self._state_sub = self.sc.scStateOutMsg.addSubscriber()

        self.sim.InitializeSimulation()

        self._rig = LockstepRig(
            self.sim,
            control_period_s=control_period_s,
            gyro=self.gyro,
            apply_torque=self._write_torque,
            truth_provider=lambda: truth_bytes(self.truth()),
            kp=self.kp,
            lockstep=lockstep,
            ack_timeout_s=ack_timeout_s,
            ack_retry_s=ack_retry_s,
            pace=pace,
        )

    # ---- rig delegations: the harness surface stays on the scenario ----------

    def attach_link(self, sockets: dict) -> None:
        """Publish every link frame (SENSOR/CMD/TRUTH) on the given bound sockets."""
        self._rig.attach_link(sockets)

    def flush_link(self) -> None:
        """Discard stale queue contents from a previous run before the first tick."""
        self._rig.flush_link()

    def close(self) -> None:
        """Release the link sockets (the ZMQ context is process-wide)."""
        self._rig.close()

    def control_tick(self) -> np.ndarray:
        """One measure -> publish -> barrier -> torque step (see the rig)."""
        return self._rig.control_tick()

    def run_to(self, sim_time_s: float) -> None:
        """Advance the dynamics to ``sim_time_s``, one control tick per period."""
        self._rig.run_to(sim_time_s)

    @property
    def rig(self) -> LockstepRig:
        """The harness itself: perpetual stop wiring, counters, channels."""
        return self._rig

    @property
    def frame(self) -> int:
        """Frame number of the most recent tick (the barrier's counter)."""
        return self._rig.frame

    @property
    def sim_time_s(self) -> float:
        """The sim instant this process is commanding at."""
        return self._rig.sim_time_s

    @property
    def rate_history(self) -> list[tuple[float, float]]:
        """(sim_time_s, |omega_measured|) per tick - the determinism input."""
        return self._rig.rate_history

    @property
    def torque_history(self) -> list[tuple[float, np.ndarray]]:
        """(sim_time_s, applied torque) per tick."""
        return self._rig.torque_history

    @property
    def external_torque_history(self) -> list[tuple[float, np.ndarray | None]]:
        """(sim_time_s, fcs torque or None) per lockstep tick."""
        return self._rig.external_torque_history

    @property
    def stale_torques(self) -> int:
        """Tripwire: answers from frames the gate already passed."""
        return self._rig.stale_torques

    @property
    def sensor_retransmits(self) -> int:
        """Handshake shouts the gate had to repeat while waiting."""
        return self._rig.sensor_retransmits

    @property
    def foreign_frames(self) -> int:
        """Tripwire: answers from frames ahead of the gate."""
        return self._rig.foreign_frames

    @property
    def bad_frames(self) -> int:
        """Tripwire: undecodable or malformed multipart frames."""
        return self._rig.bad_frames

    # ---- plant side: what the satellite measures, what the effector feels ----

    def _write_torque(self, torque: np.ndarray) -> None:
        """Write this tick's effector command (zero included: no latching)."""
        self._torque_msg.write(
            messaging.CmdTorqueBodyMsgPayload(torqueRequestBody=torque.astype(float).tolist())
        )

    def truth(self) -> Truth:
        """The simulator's own solution: ideal, unbiased, never on the sensor feed."""
        payload = self._state_sub()
        return Truth(
            sim_time_s=self.sim_time_s,
            r_bn_n=tuple(np.asarray(payload.r_BN_N, dtype=float).reshape(-1).tolist()),
            v_bn_n=tuple(np.asarray(payload.v_BN_N, dtype=float).reshape(-1).tolist()),
            sigma_bn=tuple(np.asarray(payload.sigma_BN, dtype=float).reshape(-1).tolist()),
            omega_bn_b=tuple(np.asarray(payload.omega_BN_B, dtype=float).reshape(-1).tolist()),
        )

    def gyro(self) -> np.ndarray:
        """What the satellite measures: truth rate + fixed bias + white noise."""
        truth = np.asarray(self.truth().omega_bn_b, dtype=float)
        return truth + self._bias + self._rng.normal(0.0, self._noise_sigma, 3)

    def rate_norm(self) -> float:
        """Current *true* rate norm [rad/s] from the dynamics, not the gyro."""
        return float(np.linalg.norm(np.asarray(self.truth().omega_bn_b, dtype=float)))


def main(argv: list[str] | None = None) -> int:
    """Run the scenario and report whether the tumble damped out."""
    ap = argparse.ArgumentParser(prog="detumble_scenario", description=__doc__.splitlines()[0])
    ap.add_argument(
        "--duration",
        type=float,
        default=120.0,
        help="sim seconds to run; inf = perpetual (stop with Ctrl-C/SIGTERM at a boundary)",
    )
    ap.add_argument("--control-period", type=float, default=1.0, help="control period [s]")
    ap.add_argument(
        "--zmq",
        action="store_true",
        help="lockstep link: publish EDS frames, own the torque to fcs, gate on step acks",
    )
    ap.add_argument(
        "--pace",
        action="store_true",
        help="align sim seconds to wall seconds (human watch mode; the gate stays in force)",
    )
    ap.add_argument(
        "--ack-timeout", type=float, default=ACK_TIMEOUT_S, help="lockstep gate timeout [s]"
    )
    add_link_args(ap)
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args(argv)
    resolve_endpoints(args)

    scenario = DetumbleScenario(
        control_period_s=args.control_period,
        lockstep=args.zmq,
        ack_timeout_s=args.ack_timeout,
        pace=args.pace,
    )
    if args.zmq:
        scenario.attach_link(
            bind_link(
                args.sensor_endpoint,
                args.cmd_endpoint,
                args.truth_endpoint,
                args.echo_endpoint,
            )
        )
        scenario.flush_link()

    start_rate = scenario.rate_norm()
    if not args.quiet:
        mode = "lockstep" if args.zmq else "standalone"
        print(f"initial |omega| = {start_rate:.6f} rad/s, running {args.duration:.0f} s ({mode})")

    install_stop_handlers(scenario.rig)
    try:
        scenario.run_to(args.duration)
    except LockStalled as exc:
        print(f"LOCKSTEP STALL: {exc}", flush=True)
        scenario.close()
        return 2
    if scenario.rig.stop_requested and not args.quiet:
        print(f"perpetual: stopped on request at sim time {scenario.sim_time_s:.0f} s", flush=True)

    final_rate = scenario.rate_norm()
    peak_torque = max(np.linalg.norm(t) for _, t in scenario.torque_history)
    scenario.close()

    ratio = final_rate / start_rate if start_rate else 0.0
    if not args.quiet:
        print(f"final   |omega| = {final_rate:.6f} rad/s ({ratio:.3%} of initial)")
        print(f"ticks = {len(scenario.rate_history)}, peak torque = {peak_torque:.4f} N m")
        if args.zmq:
            print(
                f"link: stale={scenario.stale_torques} foreign={scenario.foreign_frames} "
                f"bad={scenario.bad_frames} retransmits={scenario.sensor_retransmits} "
                "(stale/foreign/bad must be 0 in a healthy lockstep run)"
            )
    # Machine-readable acceptance line: printed even with --quiet, because
    # "two seeded runs, identical digest" IS the P3e determinism gate.
    print(f"trajectory sha256 {trajectory_sha256(scenario.rate_history)}")
    print("DETUMBLE OK" if ratio < CONVERGENCE_RATIO else "DETUMBLE FAILED", flush=True)
    return 0 if ratio < CONVERGENCE_RATIO else 1


if __name__ == "__main__":
    raise SystemExit(main())
