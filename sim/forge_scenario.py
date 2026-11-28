"""A bskforge TOML plant ridden by the orrery lockstep rig.

The plant is no longer written in Python: ``bskforge`` assembles the Basilisk
graph from a scenario TOML (``sim/scenarios/*.toml``), and this module wraps
the build with everything bskforge deliberately does not do - the lockstep
barrier, the EDS link codecs and the onboard knowledge model. The contract of
The division in one sentence: **the forge builds the satellite, orrery flies it**.

Two entries, one per plurality: ``ForgeScenario`` flies a one-vehicle
world (the pre-fleet surface every frozen digest was flown on) and
``ForgeFleet`` flies an N-vehicle world - one Basilisk process, one clock,
one check per vehicle: the world steps only when EVERY mission container has
acked the frame. A scenario with ``[[spacecraft]]`` blocks is a fleet; a
second vehicle is a block, never a second process fighting for the world.

Two pieces of surgery make the forge plant behave exactly like the
hand-written one (the W3 parity check demands a bit-identical trajectory):

* ``restore_effector_order`` - bskforge registers the spacecraft BEFORE its
  effectors (all priority -1), so the spacecraft update pulls the effector's
  buffer one scan stale: the P3f actuation-lag trap, re-armed by a different
  author. It applies to ``extForceTorque`` and to a reaction-wheel array
  alike. Basilisk exposes no reordering API, but ``TaskData.AddNewObject``
  re-inserts into the live C++ list, so the effector is registered a second
  time at higher priority. The trailing duplicate update is provably a no-op:
  ``ExtForceTorque`` is stateless (its UpdateState only copies
  ``cmdTorqueInMsg`` into buffers), and the wheel effector's UpdateState only
  reads messages and assigns (``u_current``, clamps, wheel speed outputs) -
  the wheel integration itself belongs to the hub's coupled integrator, which
  the duplicate registration does not touch. THIS FIX IS PER VEHICLE: a fleet
  that misses it on one body flies one body on stale actuation, and the
  digest of nobody else notices.
  A thruster effector is the third case, resolved by a spike through the C++
  before any world code landed. Its force never travels through a buffer the
  spacecraft pulls - the coupled integrator CALLS ``computeForceTorque`` -
  so the STALE half of the trap does not structurally exist for it. What IS
  task-ordered state is the firing SCHEDULE: ``UpdateState`` reads the
  on-time message and ``ConfigureThrustRequests`` arms the firing timers, and
  the tick's integration must see that tick's quota, so the freshness half of
  the fix applies unchanged. The trailing duplicate ``UpdateState`` is also
  provably a no-op here, by the effector's own guard rather than by
  statelessness: ``ReadInputs`` rejects an unchanged command message
  (``prevCommandTime == timeWritten``) before ``ConfigureThrustRequests`` can
  run twice, so a re-sent frame can never buy a second firing, and on-times
  that expire simply stop the thrust - apply-once, end to end.
* The FSW section is forbidden: a bskforge control law would put the
  "flight software" in the simulator, and the link would command a body that
  is already being commanded from inside.

What the plant *does* with a tick is the TOML's decision, not the harness's
a ``reactionWheels`` effector makes the scenario a wheel commander
(motor torque = -body torque, Basilisk's allocation convention), otherwise
the body is torqued through the external port; a fitted ``imuSensor`` makes
the plant's own sensor message the satellite's rate knowledge (bias from
the TOML, random walk from Basilisk's own sensor default seed - measured,
not assumed: this Basilisk builds every sensor with a non-zero RNGSeed, so
bskforge's ModelTag-derived reseed never fires and renaming a vehicle
cannot reseed the noise), otherwise the Python error model of the W3
witness stands. The frozen ``detumble_forge.toml`` has neither, so the
parity digests ride the old paths untouched.

    uv run python -m sim.forge_scenario --duration 60 --zmq --pace
    uv run python -m sim.forge_scenario --scenario sim/scenarios/twin_ops.toml --zmq
"""

from __future__ import annotations

import argparse
import math
import struct
import tempfile
from collections.abc import Callable, Iterable, Mapping
from pathlib import Path
from typing import Any

import numpy as np
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import horizonOpNav, limbFinding
from Basilisk.utilities import macros
from bskforge import ScenarioRunner
from bskforge.build.context import DYN_PROCESS

# The law and the sensor error model are a mission decision, not a plant
# decision: one set of constants, shared by both build paths, pinned by tests.
from sim.detumble_scenario import (
    CONVERGENCE_RATIO,
    GYRO_BIAS_RADS,
    GYRO_NOISE_SIGMA_RADS,
    KP_RADS,
)
from sim.eds_link import BurnCommand, CamConfig, encode_container, encode_fit_manifest
from sim.fleet import (
    add_link_args,
    resolve_endpoints,
    scenario_spacecraft,
    scenario_vehicles,
)
from sim.fleet_policy import load_fleet
from sim.lockstep import (
    ACK_TIMEOUT_S,
    LockStalled,
    LockstepRig,
    RigChannel,
    Truth,
    bind_link,
    install_stop_handlers,
    trajectory_sha256,
    truth_bytes,
)
from sim.lockstep import CMD_ENDPOINT as _CMD_ENDPOINT  # noqa: F401 - re-export
from sim.lockstep import SENSOR_ENDPOINT as _SENSOR_ENDPOINT  # noqa: F401
from sim.lockstep import TRUTH_ENDPOINT as _TRUTH_ENDPOINT  # noqa: F401
from sim.parts import (
    declared_id_masks,
    declared_mask,
    frame_rows,
    id_rows,
    load_registry,
    mount_law_violations,
    registry_digest,
)
from sim.thruster_alloc import allocate_on_times

#: Standard gravity: the Isp-to-mass-flow conversion Basilisk itself uses
#: (mdot = F / (Isp * g0)), stated here because the world's fuel door does
#: the same arithmetic to price a tick's ask before the plant burns it.
_G0 = 9.80665

DEFAULT_SCENARIO = Path(__file__).parent / "scenarios" / "detumble_forge.toml"

# Registry rows by part name: the world-side provider table looks its
# topics up here, so the sim publishes exactly what the catalog lands and a
# topic string is never written down twice.
PART_ROWS = {p.name: p for p in load_registry()}

# The fit vocabulary: the rows the firmware carries, in the order that
# defines the mask's bits, plus the digest of their SEMANTICS. Both are baked
# into the mission binary by codegen/emit_frames.py; a world that started with
# a different one is stale, and says so at boot instead of mis-decoding.
FIT_ROWS = frame_rows(load_registry())
FIT_HASH = registry_digest(load_registry())


# The celestial id this Basilisk's message stream assigns Earth (CAM). Not a
# guess and not a name: the Vizard scene tags its limb findings with it
# (measured: planetIds sample == 2), and horizonOpNav reads its planet RADIUS
# through the same index - aiming it elsewhere keeps the LOS honest but
# silently borrows another body's radius for the range.
OPNAV_PLANET_TARGET = 2


def plant_observers(
    body: ForgeBody, clock: Callable[[], tuple[float, int]]
) -> dict[str, Callable[[], bytes | list[bytes]]]:
    """Wire the registered observer frames this plant can actually feed.

    The topic a provider rides comes from the registry row; WHETHER the
    provider exists at all comes from the manifest - a plant without the
    device ships no frame (absence declared, not zero-filled). Publish order
    is insertion order and fixed right here, once: the digest ledger hashes
    the stream, so the world must never shuffle its own publishes.

    Fitting a registered part on a vehicle is meant to cost one
    ``if`` line in this function plus its provider method. The rig, the
    codec, the spacecraft, and the check learn the part from the registry.
    """
    observers: dict[str, Callable[[], bytes | list[bytes]]] = {}
    if body.has_plant_state:
        observers[PART_ROWS["onboard"].zmq_topic] = lambda: body.onboard_state_bytes(*clock())
    if body.has_plant_nav:
        observers[PART_ROWS["gnss"].zmq_topic] = lambda: body.nav_state_bytes(*clock())
    if body.has_plant_attitude:
        observers[PART_ROWS["attitude"].zmq_topic] = lambda: body.attitude_state_bytes(*clock())
    if body.has_plant_opnav:
        # The rig itself refuses this wiring until the registry row lands
        # (RigChannel validates every provider against the landed topics):
        # the camera world is pinned to the registry's phase line, and the
        # refusal is the point - no silent unlanded traffic, no TODO.
        observers[PART_ROWS["opnav"].zmq_topic] = lambda: body.opnav_state_bytes(*clock())
    if body.has_plant_cam:
        # Same hardware, second honest view: what the device IS,
        # beside what the pixels SAY. Same rig-side refusal if the row is
        # not landed - presence is only ever declared through the registry.
        observers[PART_ROWS["camstate"].zmq_topic] = lambda: body.cam_state_bytes(*clock())
    return observers


def resolve_fit_knobs(
    declare: list[str], hide: list[str], digest: str | None, declare_ids: list[str]
) -> tuple[dict[str, bool], dict[str, set[int]], str | None]:
    """Validate the fit-injection knobs into (override map, id claims, digest or None).

    Every lie this tool can tell is checked here, in one place, before a plant
    is built: a part name no row owns, a part both claimed and concealed, an
    id aimed at a row that carries no instance field, an id the mask cannot
    hold, a digest that is not eight hex digits. Rejecting early is what lets
    the acceptance script trust that the verdict it read came from the lie it
    meant to plant.
    """
    overlap = sorted(set(declare) & set(hide))
    if overlap:
        raise ValueError(f"--fit-declare and --fit-hide both name {overlap}: pick one per part")
    override = {name: True for name in declare} | {name: False for name in hide}
    unknown = sorted(set(override) - {row.name for row in FIT_ROWS})
    if unknown:
        rows = [row.name for row in FIT_ROWS]
        raise ValueError(f"no such registered part(s): {unknown} (fit rows: {rows})")
    id_inject: dict[str, set[int]] = {}
    id_names = {row.name for row in id_rows(FIT_ROWS)}
    for spec in declare_ids:
        name, sep, value = spec.partition("=")
        if not sep or not value.isdigit() or name not in id_names:
            raise ValueError(
                f"--fit-declare-id wants ROW=ID for an id row {sorted(id_names)}, got {spec!r}"
            )
        if not 1 <= int(value) <= 32:
            raise ValueError(f"--fit-declare-id: instance id {value} does not fit the 1..32 mask")
        id_inject.setdefault(name, set()).add(int(value))
    if digest is not None:
        try:
            digest = f"{int(digest, 16):08x}"
        except ValueError:
            raise ValueError(f"--fit-digest must be 8 hex digits, got {digest!r}") from None
    return override, id_inject, digest


def announce_fit_injections(args: argparse.Namespace, quiet: bool) -> None:
    """Print every handshake lie this run tells.

    The acceptance checks inject a wrong manifest on purpose. A knob that can
    make the spacecraft abort or degrade must never operate silently: whoever
    reads the log later must be able to tell a planted lie from a real one.
    """
    if quiet:
        return
    if getattr(args, "fit_override", None):
        print(f"FIT OVERRIDE (acceptance injection): {args.fit_override}", flush=True)
    if getattr(args, "fit_id_inject", None):
        print(
            f"FIT ID INJECTION (acceptance): manifest claims instances {args.fit_id_inject}",
            flush=True,
        )
    if getattr(args, "fit_digest", None):
        print(
            f"FIT DIGEST INJECTION (acceptance): world claims {args.fit_digest}, "
            f"catalog says {FIT_HASH}",
            flush=True,
        )


def fit_provider(
    body: ForgeBody,
    clock: Callable[[], tuple[float, int]],
    override: dict[str, bool] | None = None,
    digest: str | None = None,
    declare_ids: Mapping[str, Iterable[int]] | None = None,
) -> Callable[[], bytes]:
    """The boot handshake provider: this vehicle's declared fit, per tick.

    Published every tick rather than once, for the same reason the check
    re-shouts its first sensor frame: ZeroMQ eats messages published before a
    subscription lands, and a mission that RESTARTS mid-run heals its verdict
    within a tick instead of flying on a stale handshake forever.
    """
    return lambda: body.fit_manifest_bytes(
        *clock(), override=override, digest=digest, declare_ids=declare_ids
    )


# bskforge puts every domain model on dynamicsTask at priority -1 and its
# recorders at 100. 0 puts the effector ahead of the spacecraft (record-order
# among recorders is read-only traffic and stays untouched).
EFFECTOR_TASK_PRIORITY = 0
DYNAMICS_TASK = "dynamicsTask"


class ForgeError(RuntimeError):
    """The TOML asks for something a orrery lockstep plant cannot have."""


def restore_effector_order(sim: Any, ext_ft: Any) -> None:  # noqa: ANN401 - SWIG types
    """Re-register the external-torque port so the spacecraft sees fresh torque.

    Run after ``ScenarioRunner.build()`` and BEFORE ``InitializeSimulation``:
    Basilisk fixes model order when models are added (the C++ insert is sorted
    by priority, never re-sorted), so this is the only window in which the
    schedule can still change. See the module docstring for why the duplicate
    registration is free.
    """
    for task in sim.TaskList:
        if task.Name == DYNAMICS_TASK:
            tags = [model.ModelPtr.ModelTag for model in task.TaskData.TaskModels]
            if ext_ft.ModelTag not in tags:
                raise ForgeError(f"{ext_ft.ModelTag} is not on {DYNAMICS_TASK}")
            task.TaskData.AddNewObject(ext_ft, EFFECTOR_TASK_PRIORITY)
            return
    raise ForgeError(f"no {DYNAMICS_TASK} in the built sim")


def _reject_fsw(sc_cfg: Any, scenario_path: str | Path) -> None:  # noqa: ANN401 - bskforge model
    """Per vehicle: the control law is flight software, never the plant."""
    if sc_cfg.fsw is not None:
        raise ForgeError(
            f"[spacecraft.fsw] in {sc_cfg.name!r} ({scenario_path}): the control "
            "law lives in cFS fcs_app, not in the simulator; "
            "delete the [spacecraft.fsw] section"
        )


def _resolve_control_period(
    config: Any,  # noqa: ANN401 - bskforge model
    override: float | None,
    scenario_path: str | Path,
) -> float:
    """The world tick comes from the scenario's ``[simulation.dt] fsw``.

    One knob, one source of truth: the number that sets the Basilisk fswTask
    rate is the number the lockstep world checks on. A matching override is
    accepted; a disagreeing one is refused, because a rig ticking between
    sensor-frame boundaries is how stale acks are born - and the rig's
    scan-grid guard alone does not catch every misalignment (0.5 s is a
    legal multiple of a 0.1 s grid).
    """
    fsw_s = config.simulation.dt.fsw
    if override is not None and override != fsw_s:
        raise ForgeError(
            f"--control-period {override:g} s disagrees with [simulation.dt] "
            f"fsw = {fsw_s:g} s in {scenario_path}: the scenario owns the "
            "FSW tick - change the TOML, or pass a matching value"
        )
    return fsw_s


class ForgeBody:
    """One bskforge-built vehicle plus the harness surface riding it.

    Everything vehicle-local: the effector-order fix, the knowledge
    subscriptions, the effector writing, the truth readout and the
    OnboardState packing. A fleet builds one of these per
    ``[[spacecraft]]`` block in the SAME Basilisk world and hands each to
    its own rig channel.
    """

    def __init__(
        self,
        ctx: Any,  # noqa: ANN401 - bskforge SimContext (SWIG types inside)
        sc_cfg: Any,  # noqa: ANN401 - bskforge SpacecraftSection
        *,
        seed: int = 20261120,
        gyro_noise_sigma_rads: float = GYRO_NOISE_SIGMA_RADS,
        gyro_bias_rads: np.ndarray = GYRO_BIAS_RADS,
        control_period_s: float | None = None,
    ) -> None:
        """Fix the schedule, subscribe to the plant, guard the wire schema.

        The plant must be built but NOT yet initialized; the fleet scenario
        initializes the one shared sim once, after every body exists.
        """
        bundle = ctx.bundles[sc_cfg.name]
        self.name = sc_cfg.name
        self.bundle = bundle
        # The manifest's own claim, kept as data: WHICH hardware this
        # vehicle's TOML declares, independent of the providers that publish
        # its frames. The handshake exists to catch the two disagreeing, so it
        # must not be computed from the thing it judges. The rendered eye
        # joins as manifest hardware (CAM): a vehicle with a [[camera]] line
        # would stream the OPNAV frame, and the fit vocabulary asks exactly
        # that question of every part.
        cams = [c for c in ctx.config.camera if c.parent == sc_cfg.name]
        self.fitted_types = frozenset(str(sensor.type) for sensor in sc_cfg.sensors) | (
            {"camera"} if cams else frozenset()
        )
        # Instances, kept as manifest data like the fit: WHICH ids of a
        # hardware type this vehicle's TOML mounts. The handshake counts
        # these per id-carrying frame row, so a verdict can name the
        # instance that stalled. The eye is the first source; the shape is
        # per hardware type, not per eye.
        self._instance_ids: dict[str, set[int]] = (
            {"camera": {int(c.camera_id) for c in cams}} if cams else {}
        )
        self._sim = ctx.sim

        restore_effector_order(ctx.sim, bundle.ext_ft)
        self._torque_msg = messaging.CmdTorqueBodyMsg()
        # No FSW was built, so cmdTorqueInMsg has no writer: this body is
        # the writer, in both modes (standalone: internal law; lockstep: fcs).
        bundle.ext_ft.cmdTorqueInMsg.subscribeTo(self._torque_msg)
        self._state_sub = bundle.sc.scStateOutMsg.addSubscriber()

        # Actuator: the TOML decides. A reactionWheels effector takes
        # the motor-torque path; without one, torque enters as body torque
        # through the external port (the frozen W3 witness plant). The wheel
        # array needs the same freshness fix as extFT.
        self._motor_msg = None
        self._wheel_sub = None
        self._num_wheels = 0
        if bundle.rw is not None:
            restore_effector_order(ctx.sim, bundle.rw.effector)
            self._motor_msg = messaging.ArrayMotorTorqueMsg()
            bundle.rw.effector.rwMotorCmdInMsg.subscribeTo(self._motor_msg)
            self._wheel_sub = bundle.rw.effector.rwSpeedOutMsg.addSubscriber()
            rw_cfg = sc_cfg.reaction_wheels()
            self._num_wheels = len(rw_cfg.wheels) if rw_cfg is not None else 0

        # Actuator half two: the TOML decides thrusters too. The effector
        # speaks only THRArrayOnTimeCmdMsgPayload (seconds fired per thruster
        # per command period), and bskforge's thrusterFiring law would be a
        # control law - forbidden in the plant - so cmdsInMsg has no writer
        # and the world side is this body's burn door. The geometry read
        # back off the factory's devices is the allocation's hardware truth.
        # Ordering: the module docstring's thruster case of the P3f trap.
        self._burn_msg = None
        self._thruster_dirs: np.ndarray | None = None
        self._thruster_umax: np.ndarray | None = None
        # Fuel (fuel rung): the tank is plant hardware and Basilisk's books
        # on it are native - but measured on 2.12, an empty tank does NOT
        # gate the force: the set keeps thrusting. So the world reads the
        # tank each tick and withholds what it cannot afford: starve when
        # dry, squeeze the ask when partly full. The craft's claim ledger
        # stays what it was; the divergence when the tank dies is physics,
        # and only the bench reads both ledgers.
        self._fuel_out = None
        self._fuel_per_ns_per_n: np.ndarray | None = None  # kg per N of thrust-sec, per device
        self._fuel_load0_kg = 0.0
        self._fuel_stats = {"starved": 0, "throttled": 0}
        # The tick the quota covers: the rig's control period, told to the
        # body by its scenario; the TOML's fsw rate is the fallback so a
        # hand-built ForgeBody still knows its own rhythm.
        self.control_period_s = (
            control_period_s
            if control_period_s is not None
            else float(ctx.config.simulation.dt.fsw)
        )
        # The world's APPLIED ledger (two-ledger law): what the plant was
        # actually given, tick by tick - achieved, not asked. The craft's
        # claim lives in fcs HK; only the bench reads both.
        self._burn_stats = {
            "applied": 0,
            "saturated": 0,
            "refused_undeclared": 0,
            "refused_bad": 0,
            "impulse_applied_ns": 0.0,
            "last_burn_id": 0,
        }
        thruster = bundle.thruster
        if thruster is not None:
            restore_effector_order(ctx.sim, thruster.effector)
            self._burn_msg = messaging.THRArrayOnTimeCmdMsg()
            thruster.effector.cmdsInMsg.subscribeTo(self._burn_msg)
            devices = list(thruster.factory.thrusterList.values())
            self._thruster_dirs = np.array(
                [np.asarray(device.thrDir_B, dtype=float).ravel() for device in devices]
            )
            self._thruster_umax = np.array([float(device.MaxThrust) for device in devices])
            self._fuel_per_ns_per_n = np.array(
                [1.0 / (float(device.steadyIsp) * _G0) for device in devices]
            )
            if thruster.tank is not None:
                self._fuel_out = thruster.tank.fuelTankOutMsg
                self._fuel_load0_kg = float(thruster.tank_model.maxFuelMass)

        # Knowledge: the TOML decides too. A fitted plant IMU becomes the
        # satellite's rate source - its bias and random walk belong to the
        # plant (deterministic per scenario, no Python RNG: the walk rides
        # Basilisk's sensor default seed, which no rename reseeds). Without
        # one, the Python error model of the W3 parity witness stands.
        self._imu_sub = None
        imu = next((s for s in sc_cfg.sensors if s.type == "imuSensor"), None)
        if imu is not None:
            out_msg, _ = bundle.sensor_msgs[imu.name]
            self._imu_sub = out_msg.addSubscriber()

        # The rows say how many: every hardware type the parts registry
        # names is capped per vehicle by that registry's mounts law (single
        # unless the row fixes more). First-wins was the old silence - a
        # scenario quietly mounting hardware no wire reports; a duplicate
        # past the law now stops the world build, naming the part and the
        # count. Unlegislated types (the plant IMU, command-path effectors)
        # are physics the wire does not carry, and stay uncapped.
        law = mount_law_violations(
            [s.type for s in sc_cfg.sensors] + [e.type for e in sc_cfg.effectors]
        )
        if law:
            raise ForgeError(f"{self.name}: {law[0]}")

        # Onboard state providers: every sensor the plant fitted, plus
        # the wheel array, sampled at tick time into OnboardState frames.
        # What the plant lacks stays zero - the frame is the OBC's view, and
        # an empty channel is that view saying "nothing here".
        self._st_sub = None
        self._css_subs: list[Any] = []
        self._mag_sub = None
        for sensor in sc_cfg.sensors:
            out_msg, _ = bundle.sensor_msgs[sensor.name]
            if sensor.type == "starTracker":
                self._st_sub = out_msg.addSubscriber()
            elif sensor.type == "coarseSunSensor":
                self._css_subs.append(out_msg.addSubscriber())
            elif sensor.type == "magnetometer":
                self._mag_sub = out_msg.addSubscriber()
        if self._num_wheels > 3:
            raise ForgeError(
                f"{self.name}: the OnboardState wire fixes three wheel axes, "
                f"this plant has {self._num_wheels}"
            )

        self._bias = np.asarray(gyro_bias_rads, dtype=float)
        self._noise_sigma = gyro_noise_sigma_rads
        self._rng = np.random.default_rng(seed)
        # The GNSS receiver is plant hardware now. A TOML
        # ``gpsReceiver`` line builds a native SimpleNav in the world model;
        # this reads its output message the way the IMU line above reads the
        # gyro's - the scenario-layer error model retired with sim/gps.py.
        # No receiver block means no opinion: the NAVSTATE channel stays off,
        # absence visible rather than zero-filled.
        self._gps_sub = None
        gps = next((s for s in sc_cfg.sensors if s.type == "gpsReceiver"), None)
        if gps is not None:
            out_msg, _ = bundle.sensor_msgs[gps.name]
            self._gps_sub = out_msg.addSubscriber()

        # CAM: the estimators that turn Vizard's pixels into a planet vector.
        # [[camera]] is plant hardware the forge builds (module, Vizard
        # registration, render-rate task); limbFinding and horizonOpNav are
        # the onboard knowledge model riding it - the harness's half of
        # so they join here, between the build and InitializeSimulation, on a
        # task created AFTER the forge's camera task: each tick reads the
        # frame the camera just received.
        # One chain per mounted eye, ascending camera id: the order the
        # CAMSTATE stream rides the wire (append-only, deterministic).
        self._eyes: list[Any] = []
        self._cam_stats = {
            "applied": 0,
            "refused_id": 0,
            "refused_rate": 0,
            "refused_snap": 0,
            "pulses": 0,
        }
        # Store outlets (one per eye whose record_rate > 0, inside its eye):
        # rate, source dir, PNG filenames already chunked onto the wire,
        # and the FramesRecorded tally. The emitter is attached at rig wiring
        # - the outlets stream frames but never own the socket they ride.
        self._store_emit: Callable[[bytes], None] | None = None
        if cams:
            self._eyes = self._build_eyes(ctx, cams)

    def _build_eyes(
        self,
        ctx: Any,  # noqa: ANN401 - bskforge SimContext (SWIG types inside)
        cams: list[Any],
    ) -> list[Any]:
        """Wire limbFinding -> horizonOpNav onto every rendered eye this body mounts.

        The trimmed wheel's spellings are the API (``limbFinding.LimbFinding``,
        ``horizonOpNav.horizonOpNav`` - proven on the matched wheel), and
        ``attInMsg`` wants a ``NavAttMsg`` the harness writes per tick from
        the plant state: this wheel ships no relative-navigation module to
        build one, and the lockstep harness is the only party holding tick
        truth anyway. The estimators ride their own task at the render rate,
        created after the forge's camera task so the frame they read was
        received in the same tick.

        Returns one tuple per eye, ascending ``camera_id`` - the order the
        CAMSTATE stream rides the wire: ``(camera_id, camera module, horizon
        module, attitude message, knowledge recorder, image recorder,
        store)`` with ``store`` None for a nav-only eye.

        Naming law: the first eye keeps the v1 model and task tags untouched
        (the frozen trajectory digests are named artifacts - a second eye
        mounted beside it must not relabel the first one's models), and each
        additional eye numbers its own. Every eye gets the full chain even
        while OPNAV reports only the first: the device view (CAMSTATE) needs
        the image recorder, and the knowledge frame goes plural when its own
        channel grows - not before.

        Raises:
            ForgeError: The forge built no module for a ``[[camera]]`` line
                naming this vehicle.
        """
        eyes: list[Any] = []
        for index, cam in enumerate(sorted(cams, key=lambda c: int(c.camera_id))):
            suffix = "" if index == 0 else str(int(cam.camera_id))
            cam_module = ctx.modules_by_tag.get(f"{cam.parent}_camera{cam.camera_id}")
            if cam_module is None:
                raise ForgeError(
                    f"{self.name}: the forge built no camera module "
                    f"'{cam.parent}_camera{cam.camera_id}' for this body"
                )
            limb = limbFinding.LimbFinding()
            limb.ModelTag = f"{self.name}_limbFinder{suffix}"
            horizon = horizonOpNav.horizonOpNav()
            horizon.ModelTag = f"{self.name}_horizonNav{suffix}"
            limb.imageInMsg.subscribeTo(cam_module.imageOutMsg)
            horizon.limbInMsg.subscribeTo(limb.opnavLimbOutMsg)
            horizon.cameraConfigInMsg.subscribeTo(cam_module.cameraConfigOutMsg)
            att_msg = messaging.NavAttMsg()
            horizon.attInMsg.subscribeTo(att_msg)
            horizon.planetTarget = OPNAV_PLANET_TARGET
            task = f"{self.name}OpNavTask{suffix}"
            process = ctx.ensure_process(DYN_PROCESS, priority=3)
            process.addTask(ctx.sim.CreateNewTask(task, macros.sec2nano(cam.render_rate)))
            ctx.sim.AddModelToTask(task, limb)
            ctx.sim.AddModelToTask(task, horizon)
            # This wheel's Python read door is msg.recorder() (field arrays on
            # the recorder); addSubscriber does not exist on OpNavMsg_C. The
            # recorder rides last in the task so each tick records the write
            # horizon just made.
            recorder = horizon.opNavOutMsg.recorder()
            ctx.sim.AddModelToTask(task, recorder)
            # A recorder on the IMAGE port, same task, is the captured
            # witness. Recorder granularity is this task's tick: images that
            # complete between two ticks share one record, so Captured is a
            # floor, and its docstring says so rather than pretending otherwise.
            image_recorder = cam_module.imageOutMsg.recorder()
            ctx.sim.AddModelToTask(task, image_recorder)
            # Store outlet: Basilisk's own PNG writer (camera.cpp
            # saveImages, written synchronously inside updateState, file named by
            # the capture time to 6 decimals) becomes the byte source - no memory
            # trickery on imagePointer, and the reassembled file is literally the
            # rendered file, byte for byte. The OUTLET owns the switch: a nav-only
            # camera (record_rate = 0) never has these fields touched and writes
            # zero PNGs; absence stays visible on disk as well as on the wire.
            store: dict[str, Any] | None = None
            record_rate = int(getattr(cam, "record_rate", 0))
            record_mode = str(getattr(cam, "record_mode", "cadence"))
            if record_rate > 0 or record_mode == "snap":
                store_dir = Path(tempfile.mkdtemp(prefix=f"cam_store_{cam.camera_id}_"))
                cam_module.saveImages = 1
                cam_module.saveDir = f"{store_dir}/"
                store = {
                    "mode": record_mode,
                    "rate": record_rate,
                    "dir": store_dir,
                    "sent": [],
                    "count": 0,
                    # shutter-pulse state: outstanding SNAP asks (each
                    # carries the capture count at its command instant),
                    # the delivered total, and the shutter to restore
                    # when the queue drains (the ground's last steady
                    # command - a pulse owns the shutter, it does not
                    # redefine it)
                    "snap": [],
                    "snaps": 0,
                    "steady_shutter": 1,
                }
                door = (
                    f"every {record_rate}th capture"
                    if record_mode == "cadence"
                    else "pulse deliveries only"
                )
                print(f"{self.name}: camera{cam.camera_id} store outlet ON ({door} -> {store_dir})")
            eyes.append(
                (int(cam.camera_id), cam_module, horizon, att_msg, recorder, image_recorder, store)
            )
        return eyes

    @property
    def has_plant_state(self) -> bool:
        """Whether this plant fitted anything the OnboardState frame reports."""
        return any(
            (
                self.bundle.rw is not None,
                self._st_sub is not None,
                self._css_subs,
                self._mag_sub is not None,
            )
        )

    @property
    def has_plant_nav(self) -> bool:
        """Whether the plant fitted the GNSS receiver that feeds NAVSTATE."""
        return self._gps_sub is not None

    @property
    def has_plant_attitude(self) -> bool:
        """Whether the plant fitted the star tracker that feeds ATTSTATE.

        Same device the bundle already samples: the star-tracker row
        part to a vehicle whose manifest already had the hardware - the
        TOML line predates the registry, and its law is that presence of
        a REGISTERED part adds a FRAME, not a rewrite.
        """
        return self._st_sub is not None

    @property
    def has_plant_opnav(self) -> bool:
        """Whether this plant mounts rendered cameras that feed OPNAV."""
        return bool(self._eyes)

    def opnav_state_bytes(self, sim_time_s: float, frame: int) -> bytes:
        """Pack one ``OpNavState``: what this body's camera pixels say (CAM).

        Knowledge-channel content, nothing else - the body-frame estimate,
        its target and eye, and the estimator's own verdict flags. The image
        never travels: pixels that crossed the airgap would stop being a
        knowledge channel and start being a bandwidth problem.

        The frame carries one eye's opinion: the lowest-numbered mounted eye.
        This is the knowledge channel's scope, not the plant's - CAMSTATE
        already streams every eye. When OpNav goes plural (its own change,
        by the same append-only law) this provider returns one frame per eye
        like its device-view sibling.
        """
        assert self._eyes, "opnav_state_bytes without a rendered eye"
        camera_id, _, _, _, recorder, _, _ = self._eyes[0]
        if recorder.size():
            latest = recorder.size() - 1
            los = tuple(float(v) for v in recorder.r_BN_B[latest])
            planet_id = int(recorder.planetID[latest])
            valid = int(bool(recorder.valid[latest]))
            faulted = int(bool(recorder.faultDetected[latest]))
        else:
            # No frame has come back yet: the channel reports its own silence.
            los, planet_id, valid, faulted = (0.0, 0.0, 0.0), OPNAV_PLANET_TARGET, 0, 0
        return encode_container(
            "BSK_LINK/OpNavState",
            {
                "SimTimeS": float(sim_time_s),
                "Frame": int(frame),
                "RBnB": los,
                "PlanetId": planet_id,
                "CameraId": int(camera_id),
                "Valid": valid,
                "FaultDetected": faulted,
            },
        )

    @property
    def has_plant_cam(self) -> bool:
        """Whether this plant mounts a commandable eye.

        The same [[camera]] lines that feed OPNAV: one hardware, two honest
        views - the estimator's opinion (opnav row) and the device's own
        confirmed state (camstate row, one frame per eye).
        """
        return bool(self._eyes)

    def cam_state_bytes(self, sim_time_s: float, frame: int) -> list[bytes]:
        """Pack one ``CamState`` per mounted eye: what each camera IS.

        The device side of the commanded-vs-confirmed pair: read from each
        module's live fields, which Basilisk re-reads every update
        (camera.cpp) - an early spike flew this conclusion before the
        schema was written. ``captured`` counts distinct image timeTags at
        estimator-task granularity, boot stream included; two images
        completing between consecutive recorder ticks would share one count,
        so this is a floor, and the storage ledger is where the exact
        number lives. Order is the eye order (ascending id): the world's
        multi-instance frames ride their topic append-only.
        """
        assert self._eyes, "cam_state_bytes without a rendered eye"
        return [
            self._cam_state_bytes_one(eye, sim_time_s, frame)
            for eye in sorted(self._eyes, key=lambda e: e[0])
        ]

    def _cam_state_bytes_one(
        self,
        eye: Any,  # noqa: ANN401 - eye tuple
        sim_time_s: float,
        frame: int,
    ) -> bytes:
        """Pack one eye's ``CamState`` (see cam_state_bytes for the pair law)."""
        camera_id, module, _, _, _, image_recorder, _ = eye
        # timeTag is the camera's own stamp on each received frame - in
        # NANOSECONDS (measured live: t=2 s arrives as 2e9), while the PNG
        # name is std::to_string of the SECONDS value to 6 decimals. One
        # division reconciles them; the same double formatted on both sides
        # of the airgap is what makes the pairing exact. Distinct positive
        # values at recorder granularity is the honest capture door
        # (CameraImageMsg carries no writeTime - the recorder's payload
        # fields are exactly the message's, proven the hard way by a live
        # flight).
        captures = sorted({t / 1e9 for t in image_recorder.timeTag if t > 0.0})
        captured = len(captures)
        # The store outlet rides this same per-tick door: every Nth
        # capture's PNG bytes go out as IMG chunks before the state is packed,
        # so FramesRecorded in THIS frame counts sends made in THIS tick.
        frames_recorded = self._pump_store(eye, captures)
        # A shutter pulse is served from the same capture list: any outstanding
        # SNAP ask takes the NEXT capture outside the cadence, here, so the
        # Snapshots count in THIS frame counts a pulse delivered THIS tick.
        snapshots = self._pump_snap(eye, captures, sim_time_s)
        return encode_container(
            "BSK_LINK/CamState",
            {
                "SimTimeS": float(sim_time_s),
                "Frame": int(frame),
                "CameraId": int(camera_id),
                "Shutter": int(module.cameraIsOn),
                "RenderRateS": float(module.renderRate) / 1e9,
                "Captured": captured,
                "FramesRecorded": frames_recorded,
                "Snapshots": snapshots,
            },
        )

    def apply_cam_config(self, cfg: CamConfig) -> None:
        """The sim-side config door: validate, write, tally.

        Second guard on the same law the cam app enforces - zero render
        rate is a divide-by-zero inside vizInterface's request modulo (B1),
        and both sides of the airgap refuse it because a stale firmware on
        either side must not be able to kill the vehicle. Writes land on
        the named eye's module handle; the next Basilisk update sees them
        (spike-proven) and every digest stays byte-equal either way. A
        config naming an eye the bird does not mount is refused - the bird
        answers to its own fit, not to the ground's wishes. Refusals
        tally instead of acking: this door answers nobody, and the ground
        reads the divergence (CAM_HK vs CamState), not a receipt.
        """
        assert self._eyes, "apply_cam_config without a rendered eye"
        eyes = {eye[0]: eye for eye in self._eyes}
        if cfg.camera_id not in eyes:
            self._cam_stats["refused_id"] += 1
            fitted = ",".join(str(eye_id) for eye_id in sorted(eyes))
            print(
                f"[{self.name}] cam config refused: eye {cfg.camera_id} "
                f"not fitted (bird has {fitted})"
            )
            return
        camera_id, module, _, _, _, image_recorder, store = eyes[cfg.camera_id]
        if not math.isfinite(cfg.render_rate_s) or cfg.render_rate_s <= 0.0:
            self._cam_stats["refused_rate"] += 1
            print(f"[{self.name}] cam refused: rate {cfg.render_rate_s} must be > 0 (B1)")
            return
        module.cameraIsOn = 1 if cfg.shutter else 0
        module.renderRate = round(cfg.render_rate_s * 1e9)
        self._cam_stats["applied"] += 1
        print(
            f"[{self.name}] cam{camera_id}: shutter={'OPEN' if cfg.shutter else 'CLOSED'} "
            f"rate={cfg.render_rate_s:g}s (t={cfg.sim_time_s:.1f}s frame={cfg.frame})"
        )
        if store is not None:
            # The shutter to restore when the pulse queue drains: this
            # frame's OWN steady command. A pulse frame carries it too (the
            # cam app forwards the current settings beside the edge), so the
            # restore target can never be a state the ground never asked for.
            store["steady_shutter"] = 1 if cfg.shutter else 0
        if cfg.pulse:
            self._arm_snap(camera_id, module, image_recorder, store)

    def _arm_snap(
        self,
        camera_id: int,
        module: Any,  # noqa: ANN401 - swig module handle
        image_recorder: Any,  # noqa: ANN401 - swig recorder
        store: dict | None,
    ) -> None:
        """The shutter-pulse door: one ask queued, the shutter opened for it.

        A pulse owns the shutter until its frame leaves: closed stands open,
        and the restore target is the ground's own last steady command
        (stored on the same frame). The ask remembers how many captures had
        already happened - its frame is the NEXT one, never a re-send of an
        old file. An eye without a store outlet has nowhere for a snapshot
        to go: refused by counter, printed, and above all NOT counted in
        CamState.Snapshots, so the divergence with CAM_HK.SnapshotCmds names
        the stall instead of a silent shrug.
        """
        if store is None or self._store_emit is None:
            self._cam_stats["refused_snap"] += 1
            print(
                f"[{self.name}] cam{camera_id}: snap refused: no store outlet "
                "on this eye (record_rate = 0)"
            )
            return
        baseline = len({t for t in image_recorder.timeTag if t > 0.0})
        # The frame must RENDER before it can leave: an open shutter meets
        # vizInterface's rate modulo, so the deadline gives six render periods
        # and a floor of eight seconds. A pulse that misses it is dropped and
        # said so - it is not counted, and CamState will not pretend.
        deadline = self.sim_time_s + max(6.0 * module.renderRate * 1e-9, 8.0)
        store["snap"].append({"baseline": baseline, "deadline_s": deadline})
        module.cameraIsOn = 1
        self._cam_stats["pulses"] += 1
        print(
            f"[{self.name}] cam{camera_id}: shutter pulse armed (baseline={baseline}, "
            f"restore={'OPEN' if store['steady_shutter'] else 'CLOSED'}, due by t={deadline:.1f}s)"
        )

    # -- the store outlet        -------------------------------------------------
    #
    # The framing is ours, deliberately OUTSIDE the EDS model (the map's
    # instruction, the STEP_ACK precedent for non-ground frames): topic "IMG",
    # payload = packed header + binary slice. It rides the sensor PUB as pure
    # fire-and-forget - PUB sockets drop rather than block at HWM, so a slow
    # consumer can never stall a tick, and the stream answers to no barrier
    # and no ack. A capture whose file is not on disk yet (task order is
    # camera-update-then-observer, but the file door is still checked) is
    # retried next tick - the outlet is patient, never eager.

    IMG_TOPIC = b"IMG"
    IMG_HEADER = "<QQII"  # cam id, capture ordinal, part index, part total
    IMG_CHUNK_BYTES = 16384

    def attach_store_emitter(self, emit: Callable[[bytes], None]) -> None:
        """Give the store outlet a voice (rig wiring: send one IMG payload)."""
        self._store_emit = emit

    @property
    def store_active(self) -> bool:
        """True when any eye on this body has a store outlet (cadence or snap door)."""
        return any(eye[6] is not None for eye in self._eyes)

    def _send_capture(
        self, camera_id: int, ordinal: int, t: float, store: dict, *, note: str = ""
    ) -> str:
        """Chunk one capture's PNG onto the wire, with one of three verdicts.

        ``sent``      its last part left THIS call (the tally moved);
        ``standing``  it was already sent - the cadence door or an earlier
                      pass got here first, and a second send would duplicate;
        ``absent``    the file is not on disk (or empty) yet - patience,
                      the outlet retries rather than pretends.
        """
        name = f"{t:.6f}.png"  # camera.cpp: std::to_string(sim_s), 6 decimals
        if name in store["sent"]:
            return "standing"
        path = store["dir"] / name
        if not path.is_file():
            return "absent"  # not on disk yet: patience, retry next tick
        data = path.read_bytes()
        if len(data) == 0:
            return "absent"  # a writer that flushed nothing is not a capture
        parts = [
            data[i : i + self.IMG_CHUNK_BYTES] for i in range(0, len(data), self.IMG_CHUNK_BYTES)
        ]
        total = len(parts)
        for index, part in enumerate(parts):
            header = struct.pack(self.IMG_HEADER, camera_id, ordinal, index, total)
            emit = self._store_emit
            if emit is not None:  # the outlet has no voice before rig wiring
                emit(header + part)
        store["sent"].append(name)
        store["count"] += 1
        # The source file name is printed beside the ordinal on purpose: it is
        # the outlet's own claim of WHICH render it read, so a byte-law bench
        # binds the landed file to that exact source instead of guessing the
        # ordinal's position in a directory that holds one file per RENDER.
        tally = f"{total} part(s), {len(data)} B"
        print(f"[{self.name}] store: capture {ordinal} [{name}] -> {tally}{note}")
        return "sent"

    def _pump_store(self, eye: Any, capture_times: list[float]) -> int:  # noqa: ANN401 - eye tuple
        """Chunk one eye's every ``record_rate``-th capture's PNG onto the wire.

        Returns FramesRecorded for THIS eye: the count of captures whose
        COMPLETE byte set has been handed to the emitter (a capture enters
        the tally only when its last part has left - a part-truncated
        capture is not recorded). Eyes pump in their own provider's pass, so
        one capture's parts leave contiguously on the socket - the store's
        reassembler names files by (camera, capture) and never interleaves.

        A pulse-only outlet ("mode": "snap") has NO cadence door: steady
        captures never ship here by declaration, and the pulse pump owns
        every send. The check sits before the modulo on purpose - snap mode
        carries rate 0, and modulo would ask what divides by it.
        """
        camera_id, _, _, _, _, _, store = eye
        if store is None or self._store_emit is None:
            return 0
        if store["mode"] == "snap":
            return int(store["count"])
        rate = store["rate"]
        for ordinal, t in enumerate(capture_times, start=1):
            if ordinal % rate != 0:
                continue
            self._send_capture(camera_id, ordinal, t, store)
        return int(store["count"])

    def _pump_snap(
        self,
        eye: Any,  # noqa: ANN401 - eye tuple
        capture_times: list[float],
        sim_time_s: float,
    ) -> int:
        """Serve outstanding shutter-pulse asks, each taking the NEXT capture.

        Served outside the record cadence, the queue owns the shutter while full.

        The ask remembers the capture count at its command instant, so its
        frame is strictly newer than the ask - a snapshot is what the shutter
        sees AFTER "look now", never an old file handed over early. An
        already-standing send counts for the ask only if it happened after
        the ask (captures past the baseline cannot have been sent before it),
        which is what the baseline+1 walk keeps honest. An ask whose deadline
        passes is dropped with a line and stays UNCOUNTED: CamState.Snapshots
        below CAM_HK.SnapshotCmds is then the true story on the wire.

        Returns the eye's cumulative delivered-snapshot count (CamState field).
        """
        camera_id, module, _, _, _, _, store = eye
        if store is None or self._store_emit is None:
            return 0
        queue = store["snap"]
        for ask in list(queue):
            if sim_time_s >= ask["deadline_s"]:
                queue.remove(ask)
                print(
                    f"[{self.name}] cam{camera_id}: snap deadline passed - pulse dropped, uncounted"
                )
                continue
            delivered = False
            first = ask["baseline"] + 1
            if first <= len(capture_times):
                # ONLY the capture right after the ask can pay for it. If its
                # bytes already left (the cadence door opened on exactly this
                # ordinal), the ask is satisfied - a capture past the baseline
                # cannot have been sent BEFORE the ask, because it did not
                # exist then. Handing the ask a later frame instead would
                # sell one pulse twice.
                status = self._send_capture(
                    camera_id, first, capture_times[first - 1], store, note=" (shutter pulse)"
                )
                delivered = status != "absent"  # absent: patience, the ask keeps its turn
            if delivered:
                queue.remove(ask)
                store["snaps"] += 1
        # The queue owns the shutter while full: any ask needs an open eye,
        # and the moment the last one is served the ground's steady command
        # comes back. (This line re-asserts it every tick, so a SET_CONFIG
        # landing mid-pulse updates the restore target without losing the
        # pulse's claim on the shutter.)
        module.cameraIsOn = 1 if queue else store["steady_shutter"]
        return int(store["snaps"])

    @property
    def sim_time_s(self) -> float:
        """The shared world's instant (the sim owns time; this only reads it)."""
        return self._sim.TotalSim.CurrentNanos * 1e-9

    def write_torque(self, torque: np.ndarray) -> None:
        """Write this tick's effector command (zero included: no latching).

        The extFT port takes the body torque as-is. The wheel array is its
        mirror image - a positive motor torque accelerates the wheel and the
        vehicle feels ``-gsHat*u`` (reactionWheelStateEffector's contribution
        subtracts ``gsHat*u_current`` from the hub torque; Basilisk's own
        rwMotorTorque allocation states it: "Lr is assumed to be a positive
        torque onto the body, the [Gs]us must generate -Lr"). For the
        axis-aligned sets this mission flies, the projection is the identity.
        """
        values = torque.astype(float).tolist()
        if self._motor_msg is not None:
            self._motor_msg.write(
                messaging.ArrayMotorTorqueMsgPayload(motorTorque=[-v for v in values])
            )
        else:
            self._torque_msg.write(messaging.CmdTorqueBodyMsgPayload(torqueRequestBody=values))
        if self._eyes:
            # The knowledge channel's attitude truth rides the same door:
            # every eye's horizonOpNav reads the LAST-written NavAttMsg on
            # its own task, and this is the only per-tick write point the rig
            # gives a body. The estimate was 0.73 degrees from truth with
            # exactly this feed - one tick old, which at these rates is this
            # tick. One reading, written to every mounted eye: the body has
            # one attitude, the eyes merely disagree about its planet.
            state = self._state_sub()
            payload = messaging.NavAttMsgPayload()
            payload.timeTag = self.sim_time_s
            payload.sigma_BN = np.asarray(state.sigma_BN, dtype=float).reshape(-1).tolist()
            payload.omega_BN_B = np.asarray(state.omega_BN_B, dtype=float).reshape(-1).tolist()
            for eye in self._eyes:
                eye[3].write(payload)

    def _fuel_remaining(self) -> float:
        """Live tank stock, with the unwritten-message quirk resolved.

        Before the tank's first dynamics step its output message is
        untouched and reads ALL-ZERO - which is not "empty", it is "the
        books have not opened". The tell is the declaration: a written
        message always carries maxFuelMass > 0 (the schema forbids a zero
        load), so an unwritten read falls back to the declared load, and a
        drained-to-exact-zero tank stays exactly zero. Measured on 2.12:
        the model clamps propMass to 0.0 itself ("depletion is stopped").
        """
        msg = self._fuel_out.read()  # ty: ignore[missing-union-attribute] - set with the tank
        if float(msg.maxFuelMass) <= 0.0:
            return self._fuel_load0_kg
        return float(msg.fuelMass)

    def apply_burn(self, cmd: BurnCommand) -> None:
        """Spend one tick of a commanded burn's force quota (apply-ONCE).

        The frame's on-times never outlive the tick they were bought for, so
        the next tick thrusts only if a fresh frame arrived: a craft that
        goes silent mid-burn stops thrusting, the safe direction a lost frame
        must take. Demands the plant cannot meet are not refused but
        SATURATED - the hardware's cone is physics, and the shortfall lands
        in the applied ledger as a saturated tick, where the bench can see
        it. A vehicle with no declared thrusters refuses: an absence the
        tally reports, never an impulse silently absorbed.

        This ledger counts what was ACHIEVED (force actually given x tick),
        never what was asked. The craft's claim is fcs HK's business; the
        difference between the two is a bench finding, not an input here.
        """
        if self._burn_msg is None or self._thruster_dirs is None or self._thruster_umax is None:
            self._burn_stats["refused_undeclared"] += 1
            return
        force = float(cmd.force_n)
        axis = np.asarray(cmd.axis_b, dtype=float)
        norm = float(np.linalg.norm(axis))
        if not np.isfinite(force) or force <= 0.0 or not np.isfinite(norm) or norm < 1e-9:
            self._burn_stats["refused_bad"] += 1
            return
        alloc = allocate_on_times(
            self._thruster_dirs,
            self._thruster_umax,
            force / norm * axis,
            self.control_period_s,
        )
        if self._fuel_out is not None and self._fuel_per_ns_per_n is not None:
            # Price the tick in fuel, then let the tank's stock spend it.
            # The demand is the allocation's own plan: each device fires at
            # MaxThrust for its on-time, so its cost is umax*on/(Isp*g0).
            remaining = self._fuel_remaining()
            demand = float(np.dot(self._thruster_umax * alloc.on_times_s, self._fuel_per_ns_per_n))
            if remaining <= 0.0 or demand > remaining:
                if remaining <= 0.0:
                    # Dry: withhold the whole tick. The craft's quota clock
                    # keeps claiming while the world stays silent - the
                    # honest divergence, physical this time.
                    self._fuel_stats["starved"] += 1
                    return
                # Partly affordable: squeeze the ask to what the stock can
                # buy. Fuel cost is linear in on-time, so the affordability
                # ratio is exact on the second allocation, not an estimate.
                ratio = remaining / demand
                alloc = allocate_on_times(
                    self._thruster_dirs,
                    self._thruster_umax,
                    force / norm * axis * ratio,
                    self.control_period_s,
                )
                demand = float(
                    np.dot(self._thruster_umax * alloc.on_times_s, self._fuel_per_ns_per_n)
                )
                if demand > remaining:  # allocation's own clipping, one honest clamp
                    alloc = alloc.scaled(remaining / demand)
                self._fuel_stats["throttled"] += 1
        # SWIG ships this name as both module and class in the stub;
        # the class is what calls at runtime.
        payload = messaging.THRArrayOnTimeCmdMsgPayload()  # ty: ignore[not-callable]
        payload.OnTimeRequest = alloc.on_times_s.tolist()
        # The stamp is deliberate, not defaulted: the effector's ReadInputs
        # guards on the COMMAND TIMESTAMP (``prevCommandTime == timeWritten``
        # is "stale"), and a bare ``write()`` stamps 0 - the first quota
        # would fire and every later one would read as a retransmit. Basilisk's
        # own thruster unit test writes with an explicit time for the same
        # reason. The stamp doubles as the firing window's start: quota N,
        # written at boundary t, thrusts over [t, t + on-time], and a
        # message-less tick rewrites nothing - apply-once in the C++ too.
        self._burn_msg.write(payload, macros.sec2nano(self.sim_time_s))
        stats = self._burn_stats
        stats["applied"] += 1
        stats["impulse_applied_ns"] += (
            float(np.linalg.norm(alloc.achieved_force_B)) * self.control_period_s
        )
        if alloc.saturated:
            stats["saturated"] += 1
        stats["last_burn_id"] = int(cmd.burn_id)

    @property
    def has_thrusters(self) -> bool:
        """True when this vehicle's TOML declares a thruster effector."""
        return self._burn_msg is not None

    def truth(self) -> Truth:
        """The simulator's own solution: ideal, unbiased, never on the sensor feed."""
        payload = self._state_sub()
        stats = self._burn_stats
        fuel = self._fuel_stats
        if self._fuel_out is not None:
            # The tank keeps its own books natively; the world reports
            # them, never a Python-side accounting that could drift.
            remaining = self._fuel_remaining()
            fuel_remaining: float = remaining
            fuel_spent: float = max(self._fuel_load0_kg - remaining, 0.0)
        else:
            # No declared tank: the mass rows say so (NaN), the event counts
            # are true zeros - a tankless plant never starved a tick.
            fuel_remaining = float("nan")
            fuel_spent = float("nan")
        return Truth(
            sim_time_s=self.sim_time_s,
            r_bn_n=tuple(np.asarray(payload.r_BN_N, dtype=float).reshape(-1).tolist()),
            v_bn_n=tuple(np.asarray(payload.v_BN_N, dtype=float).reshape(-1).tolist()),
            sigma_bn=tuple(np.asarray(payload.sigma_BN, dtype=float).reshape(-1).tolist()),
            omega_bn_b=tuple(np.asarray(payload.omega_BN_B, dtype=float).reshape(-1).tolist()),
            burn_impulse_applied_ns=stats["impulse_applied_ns"],
            burn_ticks_applied=float(stats["applied"]),
            burn_ticks_saturated=float(stats["saturated"]),
            burn_id=float(stats["last_burn_id"]),
            fuel_remaining_kg=fuel_remaining,
            fuel_spent_kg=fuel_spent,
            fuel_ticks_starved=float(fuel["starved"]),
            fuel_ticks_throttled=float(fuel["throttled"]),
        )

    def gyro(self) -> np.ndarray:
        """What the satellite measures.

        With a plant IMU this is the sensor's own message: truth rate
        plus gyro bias plus random walk, all the plant's - the harness adds
        nothing. Without one, the Python error model stands: truth + fixed
        bias + white noise, same seed and draw order as ``DetumbleScenario``
        - parity between the build paths extends to the knowledge the law sees.
        """
        if self._imu_sub is not None:
            return np.asarray(self._imu_sub().AngVelPlatform, dtype=float).reshape(-1)
        truth = np.asarray(self.truth().omega_bn_b, dtype=float)
        return truth + self._bias + self._rng.normal(0.0, self._noise_sigma, 3)

    @property
    def wheel_speeds(self) -> np.ndarray | None:
        """Wheel speeds [rad/s] the plant reports, or None without a wheel array.

        The message payload is fixed-size (36 slots); trim to the fitted wheels.
        """
        if self._wheel_sub is None:
            return None
        speeds = np.asarray(self._wheel_sub().wheelSpeeds, dtype=float).reshape(-1)
        return speeds[: self._num_wheels]

    def onboard_state_bytes(self, sim_time_s: float, frame: int) -> bytes:
        """Pack one ``OnboardState`` from the plant's own outputs.

        Exactly what the OBC would read off its buses at this instant: wheel
        feedback, ST quaternion (scalar-first, CASE wrt inertial), CSS cosines
        in TOML order, measured field. Channels the plant did not fit stay
        zero - that is the OBC's view saying "nothing here", not the harness
        inventing something. The clock and frame number come from the rig:
        one world, one tick numbering, every vehicle's OBC sees the same.

        Field names arrive as dict keys and the generic container codec
        sizes the result against the registry row - this method reads
        hardware, it no longer knows wire layout.
        """
        wheels = [0.0] * 3
        speeds = self.wheel_speeds
        if speeds is not None:
            wheels[: len(speeds)] = [float(v) for v in speeds]
        quat = [0.0] * 4
        if self._st_sub is not None:
            q = np.asarray(self._st_sub().qInrtl2Case, dtype=float).reshape(-1)
            quat[: len(q)] = [float(v) for v in q]
        css = [0.0] * 4
        for index, sub in enumerate(self._css_subs):
            # Single-sensor cssDataOutMsg is CSSRawDataMsg: one cosine out.
            css[index] = float(sub().OutputData)
        mag = [0.0] * 3
        if self._mag_sub is not None:
            field = np.asarray(self._mag_sub().tam_S, dtype=float).reshape(-1)
            mag[: len(field)] = [float(v) for v in field]
        return encode_container(
            "BSK_LINK/OnboardState",
            {
                "SimTimeS": float(sim_time_s),
                "Frame": int(frame),
                "WheelSpeeds": (wheels[0], wheels[1], wheels[2]),
                "StQuatBn": (quat[0], quat[1], quat[2], quat[3]),
                "CssCos": (css[0], css[1], css[2], css[3]),
                "MagB": (mag[0], mag[1], mag[2]),
            },
        )

    def rate_norm(self) -> float:
        """Current *true* rate norm [rad/s] from the dynamics, not the gyro."""
        return float(np.linalg.norm(np.asarray(self.truth().omega_bn_b, dtype=float)))

    def nav_state_bytes(self, sim_time_s: float, frame: int) -> bytes:
        """Pack one ``NavState``: this vehicle's plant receiver fix.

        The fix is the plant ``SimpleNav`` device's own message - truth plus
        the Gauss-Markov error process the TOML declared - read from the
        device's output message, never recomputed here. What the ground sees
        as truth stays on the truth socket: two wires, one universe, and
        asserts both halves.
        """
        assert self._gps_sub is not None, "nav_state_bytes without a fitted receiver"
        msg = self._gps_sub()
        r_raw = np.asarray(msg.r_BN_N, dtype=float).ravel()
        v_raw = np.asarray(msg.v_BN_N, dtype=float).ravel()
        r_fix = (float(r_raw[0]), float(r_raw[1]), float(r_raw[2]))
        v_fix = (float(v_raw[0]), float(v_raw[1]), float(v_raw[2]))
        return encode_container(
            "BSK_LINK/NavState",
            {
                "SimTimeS": float(sim_time_s),
                "Frame": int(frame),
                "RBnN": r_fix,
                "VBnN": v_fix,
            },
        )

    def declared_fit(self, override: dict[str, bool] | None = None) -> int:
        """This vehicle's fit as its manifest declares it, as a bitmask.

        A registry row is claimed when the scenario declares ANY of the
        bskforge types that row covers (the onboard bundle row covers four
        instruments, so one CSS is enough to claim the frame). ``override``
        injects a deliberate lie for acceptance - claim a part that is not
        there, hide one that is - and ``main`` prints every override, because
        no run may whisper one.
        """
        return declared_mask(self.fitted_types, override, FIT_ROWS)

    def fit_manifest_bytes(
        self,
        sim_time_s: float,
        frame: int,
        override: dict[str, bool] | None = None,
        digest: str | None = None,
        declare_ids: Mapping[str, Iterable[int]] | None = None,
    ) -> bytes:
        """Pack ``BSK_LINK/FitManifest``: the declared fit, ids, and the catalog digest.

        An override lies about the BITS and ``declare_ids`` about the
        INSTANCES - claim an id no device carries - and only about those.
        The digest stays the world's true catalog digest, because that is
        what the handshake checks: a lie that also moved the digest would
        look like a stale binary, and a test that simulates a missing
        instance must not simulate a stale firmware.
        """
        return bytes(
            encode_fit_manifest(
                sim_time_s,
                frame,
                self.declared_fit(override),
                digest or FIT_HASH,
                declared_id_masks(self._instance_ids, declare_ids, FIT_ROWS),
            )
        )

    def attitude_state_bytes(self, sim_time_s: float, frame: int) -> bytes:
        """Pack one ``AttitudeState``: this vehicle's star tracker fix.

        The fix is the plant ST device's own ``STSensorMsgPayload`` - the
        quaternion with the white-noise error the TOML declared, read off the
        device message exactly as the GNSS fix is read off its receiver. The
        message carries quaternion ONLY, so the frame carries quaternion
        only: rate knowledge stays the gyro's (SENSOR). Scalar-first
        convention is Basilisk's C2EP (b[0] is the scalar), pinned by
        tests/test_attitude_landing.py against the device itself.
        """
        assert self._st_sub is not None, "attitude_state_bytes without a fitted star tracker"
        q_raw = np.asarray(self._st_sub().qInrtl2Case, dtype=float).ravel()
        quat = (float(q_raw[0]), float(q_raw[1]), float(q_raw[2]), float(q_raw[3]))
        return encode_container(
            "BSK_LINK/AttitudeState",
            {
                "SimTimeS": float(sim_time_s),
                "Frame": int(frame),
                "StqBn": quat,
            },
        )


class ForgeScenario:
    """bskforge-built rigid body, commanded externally, ridden by the rig.

    The single-vehicle world: same harness surface as ``DetumbleScenario``
    (attach_link/run_to/truth/gyro/...), so ``scripts/fly_world.sh``
    treats the two modules as interchangeable via ``SIM_MODULE``.
    """

    def __init__(
        self,
        scenario_path: str | Path = DEFAULT_SCENARIO,
        control_period_s: float | None = None,
        seed: int = 20261120,
        gyro_noise_sigma_rads: float = GYRO_NOISE_SIGMA_RADS,
        gyro_bias_rads: np.ndarray = GYRO_BIAS_RADS,
        kp_rads: np.ndarray = KP_RADS,
        lockstep: bool = False,
        ack_timeout_s: float = ACK_TIMEOUT_S,
        ack_retry_s: float = 0.5,
        pace: bool = False,
        fit_override: dict[str, bool] | None = None,
        fit_digest: str | None = None,
        fit_declare_ids: dict[str, set[int]] | None = None,
    ) -> None:
        """Load the TOML, build the graph, fix the schedule, initialize.

        Raises:
            FleetPolicyError: the TOML wires hardware in by hand, which the
                audited load refuses before anything is built.
            ForgeError: more than one spacecraft (that is a fleet - build a
                ``ForgeFleet``), a bskforge FSW section, or a
                ``--control-period`` that disagrees with ``[simulation.dt] fsw``.
        """
        config = load_fleet(scenario_path)
        if len(config.spacecraft) != 1:
            raise ForgeError(
                f"orrery plants carry exactly one spacecraft, "
                f"got {len(config.spacecraft)}; command a fleet with ForgeFleet"
            )
        sc_cfg = config.spacecraft[0]
        _reject_fsw(sc_cfg, scenario_path)
        control_period_s = _resolve_control_period(config, control_period_s, scenario_path)

        built = ScenarioRunner(config).build()
        ctx = built.context
        self.name = sc_cfg.name
        self.sim = ctx.sim
        self.body = ForgeBody(
            ctx,
            sc_cfg,
            seed=seed,
            gyro_noise_sigma_rads=gyro_noise_sigma_rads,
            gyro_bias_rads=gyro_bias_rads,
            control_period_s=control_period_s,
        )
        self.bundle = self.body.bundle

        # Rig wired BEFORE init: the loop's first tick IS the renderer's
        # greeting, and nothing may jump the queue - a stolen step left
        # that tick out of every rate history and MOVED the twin digests,
        # and a pre-first-scan WriteProtobuffer divides by a renderRate no
        # scan has copied yet (gdb: divq, greeter v2). The patience build
        # pinned in vizard.lock waits for the world as long as the world
        # needs, so nothing here choreographs the player's startup.
        rig = LockstepRig(
            ctx.sim,
            control_period_s=control_period_s,
            gyro=self.body.gyro,
            apply_torque=self.body.write_torque,
            truth_provider=lambda: truth_bytes(self.body.truth()),
            kp=np.asarray(kp_rads, dtype=float),
            lockstep=lockstep,
            ack_timeout_s=ack_timeout_s,
            ack_retry_s=ack_retry_s,
            pace=pace,
            observers=plant_observers(self.body, lambda: (rig.sim_time_s, rig.frame)),
            cam_sink=self.body.apply_cam_config if self.body.has_plant_cam else None,
            burn_sink=self.body.apply_burn if self.body.has_thrusters else None,
            fit_provider=fit_provider(
                self.body,
                lambda: (rig.sim_time_s, rig.frame),
                fit_override,
                fit_digest,
                fit_declare_ids,
            ),
        )
        self._rig = rig

        ctx.sim.InitializeSimulation()

    # ---- rig delegations: identical harness surface to DetumbleScenario ----

    def attach_link(self, sockets: dict) -> None:
        """Publish every link frame (SENSOR/CMD/TRUTH) on the given bound sockets."""
        self._rig.attach_link(sockets)
        if self.body.store_active:
            # The store outlet's voice is the sensor PUB: IMG is fire-and-
            # forget by socket nature (PUB drops, never blocks), which is the
            # v2 law - images answer to no check.
            self.body.attach_store_emitter(
                lambda payload: sockets["sensor"].send_multipart([ForgeBody.IMG_TOPIC, payload])
            )

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
        """Tripwire: answers from frames the check already passed."""
        return self._rig.stale_torques

    @property
    def sensor_retransmits(self) -> int:
        """Handshake shouts the check had to repeat while waiting."""
        return self._rig.sensor_retransmits

    @property
    def foreign_frames(self) -> int:
        """Tripwire: answers from frames ahead of the check."""
        return self._rig.foreign_frames

    @property
    def bad_frames(self) -> int:
        """Tripwire: undecodable or malformed multipart frames."""
        return self._rig.bad_frames

    # ---- plant side: what the satellite measures, what the effector feels --

    def _write_torque(self, torque: np.ndarray) -> None:
        """Write this tick's effector command (per-body, see ForgeBody)."""
        self.body.write_torque(torque)

    def truth(self) -> Truth:
        """The simulator's own solution (per-body, see ForgeBody)."""
        return self.body.truth()

    def gyro(self) -> np.ndarray:
        """What the satellite measures (per-body, see ForgeBody)."""
        return self.body.gyro()

    @property
    def wheel_speeds(self) -> np.ndarray | None:
        """Wheel speeds [rad/s] the plant reports, or None without wheels."""
        return self.body.wheel_speeds

    def _onboard_state_bytes(self) -> bytes:
        """One OnboardState frame for THIS tick (the rig supplies clock/frame)."""
        return self.body.onboard_state_bytes(self._rig.sim_time_s, self._rig.frame)

    def rate_norm(self) -> float:
        """Current *true* rate norm [rad/s] from the dynamics, not the gyro."""
        return self.body.rate_norm()


class ForgeFleet:
    """One bskforge world, N vehicles, N checks on one clock.

    Every ``[[spacecraft]]`` block becomes a :class:`ForgeBody` with its own
    rig channel and its own socket set, addressed by its block index through
    ``sim.fleet.ports`` - the file IS the fleet and nothing is declared
    twice. One tick: measure every body, publish every frame, check every
    container, apply every torque, then step the world once. A container
    that stops acking raises ``LockStalled`` naming its vehicle: one dead
    mission stalls the world loudly, it never advances on stale knowledge.
    """

    def __init__(
        self,
        scenario_path: str | Path,
        control_period_s: float | None = None,
        seed: int = 20261120,
        gyro_noise_sigma_rads: float = GYRO_NOISE_SIGMA_RADS,
        gyro_bias_rads: np.ndarray = GYRO_BIAS_RADS,
        kp_rads: np.ndarray = KP_RADS,
        lockstep: bool = False,
        ack_timeout_s: float = ACK_TIMEOUT_S,
        ack_retry_s: float = 0.5,
        pace: bool = False,
        port_base: int = 0,
        fit_override: dict[str, bool] | None = None,
        fit_digest: str | None = None,
        fit_declare_ids: dict[str, set[int]] | None = None,
    ) -> None:
        """Load the multi-vehicle TOML, build every body, initialize once.

        Raises:
            FleetPolicyError: the fleet file wires hardware in by hand, which
                the audited load refuses before anything is built.
            ForgeError: an empty fleet, a bskforge FSW section (any
                vehicle), a ``--control-period`` that disagrees with
                ``[simulation.dt] fsw``, or a derived-identity disagreement
                (belt & braces: raw-TOML order vs the validated config).
        """
        config = load_fleet(scenario_path)
        if not config.spacecraft:
            raise ForgeError(f"{scenario_path}: a fleet needs at least one spacecraft")
        for sc_cfg in config.spacecraft:
            _reject_fsw(sc_cfg, scenario_path)
        control_period_s = _resolve_control_period(config, control_period_s, scenario_path)

        built = ScenarioRunner(config).build()
        ctx = built.context
        self.scenario_path = str(scenario_path)
        self.vehicles = scenario_vehicles(scenario_path, port_base=port_base)
        if [v.name for v in self.vehicles] != [sc.name for sc in config.spacecraft]:
            raise ForgeError(
                "derived vehicle order disagrees with the built config - "
                "the raw TOML and the validator must agree on block order"
            )
        self.bodies = [
            ForgeBody(
                ctx,
                sc_cfg,
                seed=seed + index,  # per-vehicle RNG stream, deterministic
                gyro_noise_sigma_rads=gyro_noise_sigma_rads,
                gyro_bias_rads=gyro_bias_rads,
                control_period_s=control_period_s,
            )
            for index, sc_cfg in enumerate(config.spacecraft)
        ]

        # Rig before init, same law as the single-vehicle path: the loop's
        # first tick is the renderer's greeting (see ForgeScenario for both
        # falsified greeters).
        rig = LockstepRig(
            ctx.sim,
            control_period_s=control_period_s,
            channels=[
                RigChannel(
                    name=body.name,
                    gyro=body.gyro,
                    apply_torque=body.write_torque,
                    truth_provider=(lambda b=body: truth_bytes(b.truth())),
                    kp=np.asarray(kp_rads, dtype=float),
                    observers=plant_observers(body, lambda: (rig.sim_time_s, rig.frame)),
                    cam_sink=body.apply_cam_config if body.has_plant_cam else None,
                    burn_sink=body.apply_burn if body.has_thrusters else None,
                    fit_provider=fit_provider(
                        body,
                        lambda: (rig.sim_time_s, rig.frame),
                        fit_override,
                        fit_digest,
                        fit_declare_ids,
                    ),
                )
                for body in self.bodies
            ],
            lockstep=lockstep,
            ack_timeout_s=ack_timeout_s,
            ack_retry_s=ack_retry_s,
            pace=pace,
        )
        self._rig = rig

        ctx.sim.InitializeSimulation()

    # ---- harness surface ----------------------------------------------------

    def attach_links(self, socket_sets: list[dict]) -> None:
        """Give channel i its socket set; one set per vehicle, same order."""
        if len(socket_sets) != len(self.bodies):
            raise ForgeError(
                f"fleet has {len(self.bodies)} vehicles, got {len(socket_sets)} socket sets"
            )
        for index, sockets in enumerate(socket_sets):
            self._rig.attach_channel(index, sockets)
            body = self.bodies[index]
            if body.store_active:
                body.attach_store_emitter(
                    lambda payload, sock=sockets["sensor"]: sock.send_multipart(
                        [ForgeBody.IMG_TOPIC, payload]
                    )
                )

    def flush_link(self) -> None:
        """Discard stale queue contents (every channel) before the first tick."""
        self._rig.flush_link()

    def close(self) -> None:
        """Release every channel's link sockets."""
        self._rig.close()

    def run_to(self, sim_time_s: float) -> None:
        """Advance the shared world, one check-everyone tick per control period."""
        self._rig.run_to(sim_time_s)

    @property
    def rig(self) -> LockstepRig:
        """The harness itself: perpetual stop wiring, per-channel detail."""
        return self._rig

    @property
    def frame(self) -> int:
        """The world's frame counter (every container acks the same number)."""
        return self._rig.frame

    @property
    def sim_time_s(self) -> float:
        """The shared world's instant."""
        return self._rig.sim_time_s

    @property
    def stale_torques(self) -> int:
        """Tripwire summed over checks (per-channel detail lives in channels)."""
        return self._rig.stale_torques

    @property
    def sensor_retransmits(self) -> int:
        """Handshake shouts summed over checks."""
        return self._rig.sensor_retransmits

    @property
    def foreign_frames(self) -> int:
        """Tripwire summed over checks."""
        return self._rig.foreign_frames

    @property
    def bad_frames(self) -> int:
        """Tripwire summed over checks."""
        return self._rig.bad_frames

    def rate_history(self, vehicle: str) -> list[tuple[float, float]]:
        """One vehicle's digest stream: (sim_time_s, |omega|) per tick."""
        for ch in self._rig.channels:
            if ch.name == vehicle:
                return ch.rate_history
        raise KeyError(f"no vehicle {vehicle!r} in this fleet")

    def rate_norms(self) -> dict[str, float]:
        """True |omega| per vehicle right now."""
        return {body.name: body.rate_norm() for body in self.bodies}

    def wheel_speeds(self, vehicle: str) -> np.ndarray | None:
        """One vehicle's wheel speeds, or None if its plant has no wheels."""
        for body in self.bodies:
            if body.name == vehicle:
                return body.wheel_speeds
        raise KeyError(f"no vehicle {vehicle!r} in this fleet")


def main(argv: list[str] | None = None) -> int:
    """Run the bskforge-built scenario(s) and report whether they damped out."""
    ap = argparse.ArgumentParser(prog="forge_scenario", description=__doc__.splitlines()[0])
    ap.add_argument("--scenario", default=str(DEFAULT_SCENARIO), help="bskforge TOML path")
    ap.add_argument(
        "--duration",
        type=float,
        default=120.0,
        help="sim seconds to run; inf = perpetual (stop with Ctrl-C/SIGTERM at a boundary)",
    )
    ap.add_argument(
        "--control-period",
        type=float,
        default=None,
        help="override; must match the scenario's [simulation.dt] fsw (default: the TOML)",
    )
    ap.add_argument(
        "--port-base",
        type=int,
        default=0,
        help="fleet only: shift every derived port slot (two scenario fleets "
        "on one box; block ORDER and names are untouched)",
    )
    ap.add_argument(
        "--zmq",
        action="store_true",
        help="lockstep link: publish EDS frames, own the torque to fcs, check on step acks",
    )
    ap.add_argument(
        "--pace",
        action="store_true",
        help="align sim seconds to wall seconds (human watch mode; the check stays in force)",
    )
    ap.add_argument("--ack-timeout", type=float, default=ACK_TIMEOUT_S, help="check timeout [s]")
    ap.add_argument(
        "--fit-declare",
        action="append",
        default=[],
        metavar="PART",
        help="claim a part this vehicle does NOT declare (acceptance injection)",
    )
    ap.add_argument(
        "--fit-hide",
        action="append",
        default=[],
        metavar="PART",
        help="conceal a part this vehicle DOES declare (acceptance injection)",
    )
    ap.add_argument(
        "--fit-declare-id",
        action="append",
        default=[],
        metavar="ROW=ID",
        help="claim an instance id the vehicle does not carry (acceptance injection)",
    )
    ap.add_argument(
        "--fit-digest",
        metavar="HEX8",
        help="claim this registry digest instead of the world's own (stale-firmware injection)",
    )
    add_link_args(ap)
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args(argv)
    try:
        args.fit_override, args.fit_id_inject, args.fit_digest = resolve_fit_knobs(
            args.fit_declare, args.fit_hide, args.fit_digest, args.fit_declare_id
        )
    except ValueError as exc:
        ap.error(str(exc))

    names = scenario_spacecraft(args.scenario)
    if len(names) > 1:
        return _main_fleet(args, names)
    if args.port_base:
        ap.error("--port-base is a fleet option; a single vehicle uses --instance")
    return _main_single(args)


def _main_fleet(args: argparse.Namespace, names: list[str]) -> int:
    """Multi-vehicle mode: derived per-block addressing, one process."""
    if args.instance != 0 or args.sensor_endpoint or args.cmd_endpoint or args.truth_endpoint:
        ap_err = argparse.ArgumentParser(prog="forge_scenario", add_help=False)
        ap_err.error(
            "--instance/--*-endpoint apply to single-vehicle scenarios only; "
            f"a fleet derives every block from the scenario ({', '.join(names)})"
        )
    vehicles = scenario_vehicles(args.scenario, port_base=args.port_base)

    fleet = ForgeFleet(
        args.scenario,
        control_period_s=args.control_period,
        fit_override=args.fit_override or None,
        fit_digest=args.fit_digest,
        fit_declare_ids=args.fit_id_inject or None,
        lockstep=args.zmq,
        ack_timeout_s=args.ack_timeout,
        pace=args.pace,
        port_base=args.port_base,
    )
    if args.zmq:
        socket_sets = [
            bind_link(
                v.ports.sensor_endpoint,
                v.ports.link_cmd_endpoint,
                v.ports.truth_endpoint,
                v.ports.echo_endpoint,
            )
            for v in vehicles
        ]
        fleet.attach_links(socket_sets)
        fleet.flush_link()

    start_rates = fleet.rate_norms()
    if not args.quiet:
        mode = "lockstep fleet" if args.zmq else "standalone fleet"
        print(f"scenario = {args.scenario}, world = {len(vehicles)} vehicles, {mode}")
        for v in vehicles:
            print(
                f"vehicle {v.name}: instance {v.instance} beacon :{v.ports.beacon}/"
                f":{v.ports.uplink} sensor={v.ports.sensor} link_cmd={v.ports.link_cmd} "
                f"truth={v.ports.truth} echo={v.ports.echo} target={v.cosmos_target}"
            )
        print(
            "initial |omega| = "
            + ", ".join(f"{n}={r:.6f}" for n, r in start_rates.items())
            + f", running {args.duration:.0f} s"
        )

    announce_fit_injections(args, args.quiet)
    install_stop_handlers(fleet.rig)
    try:
        fleet.run_to(args.duration)
    except LockStalled as exc:
        print(f"LOCKSTEP STALL: {exc}", flush=True)
        fleet.close()
        return 2
    if fleet.rig.stop_requested and not args.quiet:
        print(f"perpetual: stopped on request at sim time {fleet.sim_time_s:.0f} s", flush=True)

    final_rates = fleet.rate_norms()
    fleet.close()
    if not args.quiet:
        for name, start in start_rates.items():
            final = final_rates[name]
            ratio = final / start if start else 0.0
            print(f"final   |omega| {name} = {final:.6f} rad/s ({ratio:.3%} of initial)")
    for name in start_rates:
        # Per-vehicle digests, named: a fleet prints one trajectory per body.
        print(f"{name} trajectory sha256 {trajectory_sha256(fleet.rate_history(name))}")
    if args.zmq:
        print(
            f"link: stale={fleet.stale_torques} foreign={fleet.foreign_frames} "
            f"bad={fleet.bad_frames} retransmits={fleet.sensor_retransmits} "
            "(stale/foreign/bad must be 0 in a healthy lockstep run)"
        )
    print("FLEET RUN OK", flush=True)
    return 0


def _main_single(args: argparse.Namespace) -> int:
    """Single-vehicle mode: the pre-fleet surface, byte-for-byte behavior."""
    block = resolve_endpoints(args)

    scenario = ForgeScenario(
        args.scenario,
        control_period_s=args.control_period,
        fit_override=args.fit_override or None,
        fit_digest=args.fit_digest,
        fit_declare_ids=args.fit_id_inject or None,
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
        print(f"plant = {args.scenario}, spacecraft = {scenario.name}, {mode}")
        print(
            f"ports: instance {args.instance} "
            f"sensor={block.sensor} cmd={block.link_cmd} truth={block.truth} echo={block.echo}"
        )
        print(f"initial |omega| = {start_rate:.6f} rad/s, running {args.duration:.0f} s")

    announce_fit_injections(args, args.quiet)
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
        if scenario.wheel_speeds is not None:
            print(f"wheel speeds = {np.array2string(scenario.wheel_speeds, precision=1)} rad/s")
        if args.zmq:
            print(
                f"link: stale={scenario.stale_torques} foreign={scenario.foreign_frames} "
                f"bad={scenario.bad_frames} retransmits={scenario.sensor_retransmits} "
                "(stale/foreign/bad must be 0 in a healthy lockstep run)"
            )
    # Same machine-readable acceptance line as the hand-written scenario:
    # parity and determinism checks both read it, and they demand equality.
    print(f"trajectory sha256 {trajectory_sha256(scenario.rate_history)}")
    print("DETUMBLE OK" if ratio < CONVERGENCE_RATIO else "DETUMBLE FAILED", flush=True)
    return 0 if ratio < CONVERGENCE_RATIO else 1


if __name__ == "__main__":
    raise SystemExit(main())
