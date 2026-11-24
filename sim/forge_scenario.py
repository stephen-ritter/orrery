"""A bskforge TOML plant ridden by the orrery lockstep rig (ROADMAP W3, D7).

The plant is no longer written in Python: ``bskforge`` assembles the Basilisk
graph from a scenario TOML (``sim/scenarios/*.toml``), and this module wraps
the build with everything bskforge deliberately does not do - the D5 lockstep
barrier, the EDS link codecs and the onboard knowledge model. The contract of
D7 in one sentence: **the forge builds the satellite, orrery flies it**.

Two entries, one per plurality (D10): ``ForgeScenario`` flies a one-vehicle
world (the pre-fleet surface every frozen digest was flown on) and
``ForgeFleet`` flies an N-vehicle world - one Basilisk process, one clock,
one gate per vehicle: the world steps only when EVERY mission container has
acked the frame. A scenario with ``[[spacecraft]]`` blocks is a fleet; a
second vehicle is a block, never a second process fighting for the world.

Two pieces of surgery make the forge plant behave exactly like the
hand-written one (the W3 parity gate demands a bit-identical trajectory):

* ``restore_effector_order`` - bskforge registers the spacecraft BEFORE its
  effectors (all priority -1), so the spacecraft update pulls the effector's
  buffer one scan stale: the P3f actuation-lag trap, re-armed by a different
  author. It applies to ``extForceTorque`` and to a reaction-wheel array
  alike (P4a). Basilisk exposes no reordering API, but ``TaskData.AddNewObject``
  re-inserts into the live C++ list, so the effector is registered a second
  time at higher priority. The trailing duplicate update is provably a no-op:
  ``ExtForceTorque`` is stateless (its UpdateState only copies
  ``cmdTorqueInMsg`` into buffers), and the wheel effector's UpdateState only
  reads messages and assigns (``u_current``, clamps, wheel speed outputs) -
  the wheel integration itself belongs to the hub's coupled integrator, which
  the duplicate registration does not touch. THIS FIX IS PER VEHICLE: a fleet
  that misses it on one body flies one body on stale actuation, and the
  digest of nobody else notices.
* The FSW section is forbidden (``D7``): a bskforge control law would put the
  "flight software" in the simulator, and the link would command a body that
  is already being commanded from inside.

What the plant *does* with a tick is the TOML's decision, not the harness's
(P4a): a ``reactionWheels`` effector makes the scenario a wheel commander
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
from collections.abc import Callable
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
from sim.eds_link import CamConfig, encode_container, encode_fit_manifest
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
from sim.parts import declared_mask, frame_rows, load_registry, registry_digest

DEFAULT_SCENARIO = Path(__file__).parent / "scenarios" / "detumble_forge.toml"

# Registry rows by part name (REG): the world-side provider table looks its
# topics up here, so the sim publishes exactly what the catalog lands and a
# topic string is never written down twice.
PART_ROWS = {p.name: p for p in load_registry()}

# The fit vocabulary (OB): the rows the firmware carries, in the order that
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
) -> dict[str, Callable[[], bytes]]:
    """Wire the registered observer frames this plant can actually feed (REG).

    The topic a provider rides comes from the registry row; WHETHER the
    provider exists at all comes from the manifest - a plant without the
    device ships no frame (absence declared, not zero-filled). Publish order
    is insertion order and fixed right here, once: the digest ledger hashes
    the stream, so the world must never shuffle its own publishes.

    An exit demo (REG/D13): fitting a registered part on a vehicle costs one
    ``if`` line in this function plus its provider method. The rig, the
    codec, the spacecraft, and the gate learn the part from the registry.
    """
    observers: dict[str, Callable[[], bytes]] = {}
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
        # Same hardware, second honest view (CAM v2a): what the device IS,
        # beside what the pixels SAY. Same rig-side refusal if the row is
        # not landed - presence is only ever declared through the registry.
        observers[PART_ROWS["camstate"].zmq_topic] = lambda: body.cam_state_bytes(*clock())
    return observers


def resolve_fit_knobs(
    declare: list[str], hide: list[str], digest: str | None
) -> tuple[dict[str, bool], str | None]:
    """Validate the OB injection knobs into (override map, digest or None).

    Every lie this tool can tell is checked here, in one place, before a plant
    is built: a part name no row owns, a part both claimed and concealed, a
    digest that is not eight hex digits. Rejecting early is what lets the
    acceptance script trust that the verdict it read came from the lie it meant
    to plant.
    """
    overlap = sorted(set(declare) & set(hide))
    if overlap:
        raise ValueError(f"--fit-declare and --fit-hide both name {overlap}: pick one per part")
    override = {name: True for name in declare} | {name: False for name in hide}
    unknown = sorted(set(override) - {row.name for row in FIT_ROWS})
    if unknown:
        rows = [row.name for row in FIT_ROWS]
        raise ValueError(f"no such registered part(s): {unknown} (fit rows: {rows})")
    if digest is not None:
        try:
            digest = f"{int(digest, 16):08x}"
        except ValueError:
            raise ValueError(f"--fit-digest must be 8 hex digits, got {digest!r}") from None
    return override, digest


def announce_fit_injections(args: argparse.Namespace, quiet: bool) -> None:
    """Print every handshake lie this run tells (OB).

    The acceptance gates inject a wrong manifest on purpose. A knob that can
    make the spacecraft abort or degrade must never operate silently: whoever
    reads the log later must be able to tell a planted lie from a real one.
    """
    if quiet:
        return
    if getattr(args, "fit_override", None):
        print(f"FIT OVERRIDE (acceptance injection): {args.fit_override}", flush=True)
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
) -> Callable[[], bytes]:
    """The boot handshake provider (OB): this vehicle's declared fit, per tick.

    Published every tick rather than once, for the same reason the gate
    re-shouts its first sensor frame: ZeroMQ eats messages published before a
    subscription lands, and a mission that RESTARTS mid-run heals its verdict
    within a tick instead of flying on a stale handshake forever.
    """
    return lambda: body.fit_manifest_bytes(*clock(), override=override, digest=digest)


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
    """D7, per vehicle: the control law is flight software, never the plant."""
    if sc_cfg.fsw is not None:
        raise ForgeError(
            f"[spacecraft.fsw] in {sc_cfg.name!r} ({scenario_path}): the control "
            "law lives in cFS fcs_app, not in the simulator (decision D7); "
            "delete the [spacecraft.fsw] section"
        )


def _resolve_control_period(
    config: Any,  # noqa: ANN401 - bskforge model
    override: float | None,
    scenario_path: str | Path,
) -> float:
    """The world tick comes from the scenario's ``[simulation.dt] fsw``.

    One knob, one source of truth: the number that sets the Basilisk fswTask
    rate is the number the lockstep world gates on. A matching override is
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
    OnboardState packing. A fleet (D10) builds one of these per
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
    ) -> None:
        """Fix the schedule, subscribe to the plant, guard the wire schema.

        The plant must be built but NOT yet initialized; the fleet scenario
        initializes the one shared sim once, after every body exists.
        """
        bundle = ctx.bundles[sc_cfg.name]
        self.name = sc_cfg.name
        self.bundle = bundle
        # The manifest's own claim (OB), kept as data: WHICH hardware this
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
        self._sim = ctx.sim

        restore_effector_order(ctx.sim, bundle.ext_ft)
        self._torque_msg = messaging.CmdTorqueBodyMsg()
        # No FSW was built, so cmdTorqueInMsg has no writer: this body is
        # the writer, in both modes (standalone: internal law; lockstep: fcs).
        bundle.ext_ft.cmdTorqueInMsg.subscribeTo(self._torque_msg)
        self._state_sub = bundle.sc.scStateOutMsg.addSubscriber()

        # Actuator: the TOML decides (P4a). A reactionWheels effector takes
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

        # Onboard state providers (P4b): every sensor the plant fitted, plus
        # the wheel array, sampled at tick time into OnboardState frames.
        # What the plant lacks stays zero - the frame is the OBC's view, and
        # an empty channel is that view saying "nothing here".
        self._st_sub = None
        self._css_subs: list[Any] = []
        self._mag_sub = None
        for sensor in sc_cfg.sensors:
            out_msg, _ = bundle.sensor_msgs[sensor.name]
            if sensor.type == "starTracker" and self._st_sub is None:
                self._st_sub = out_msg.addSubscriber()
            elif sensor.type == "coarseSunSensor":
                self._css_subs.append(out_msg.addSubscriber())
            elif sensor.type == "magnetometer" and self._mag_sub is None:
                self._mag_sub = out_msg.addSubscriber()
        if self._num_wheels > 3:
            raise ForgeError(
                f"{self.name}: the OnboardState wire fixes three wheel axes, "
                f"this plant has {self._num_wheels}"
            )
        if len(self._css_subs) > 4:
            raise ForgeError(
                f"{self.name}: the OnboardState wire fixes four CSS units, "
                f"this plant has {len(self._css_subs)}"
            )

        self._bias = np.asarray(gyro_bias_rads, dtype=float)
        self._noise_sigma = gyro_noise_sigma_rads
        self._rng = np.random.default_rng(seed)
        # KN2: the GNSS receiver is plant hardware now (D13). A TOML
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
        # the onboard knowledge model riding it - the harness's half of D7 -
        # so they join here, between the build and InitializeSimulation, on a
        # task created AFTER the forge's camera task: each tick reads the
        # frame the camera just received.
        self._opnav: tuple[Any, ...] | None = None
        self._cam_stats = {"applied": 0, "refused_id": 0, "refused_rate": 0}
        if cams:
            self._opnav = self._build_opnav(ctx, cams)

    def _build_opnav(
        self,
        ctx: Any,  # noqa: ANN401 - bskforge SimContext (SWIG types inside)
        cams: list[Any],
    ) -> tuple[Any, ...]:
        """Wire limbFinding -> horizonOpNav onto this body's rendered camera.

        The trimmed wheel's spellings are the API (``limbFinding.LimbFinding``,
        ``horizonOpNav.horizonOpNav`` - proven on the matched wheel), and
        ``attInMsg`` wants a ``NavAttMsg`` the harness writes per tick from
        the plant state: this wheel ships no relative-navigation module to
        build one, and the lockstep harness is the only party holding tick
        truth anyway. The estimators ride their own task at the render rate,
        created after the forge's camera task so the frame they read was
        received in the same tick.

        Returns the tuple ``(camera_id, camera module, horizon module,
        attitude message, knowledge recorder, image recorder)``.

        Raises:
            ForgeError: The plant mounts more than one camera for this body
                (v1 knowledge channels are single-eyed), or the forge built
                no module for a ``[[camera]]`` line naming this vehicle.
        """
        if len(cams) > 1:
            raise ForgeError(
                f"{self.name}: the OPNAV frame carries one camera's knowledge; "
                f"this plant mounts {len(cams)}"
            )
        cam = cams[0]
        cam_module = ctx.modules_by_tag.get(f"{cam.parent}_camera{cam.camera_id}")
        if cam_module is None:
            raise ForgeError(
                f"{self.name}: the forge built no camera module "
                f"'{cam.parent}_camera{cam.camera_id}' for this body"
            )
        limb = limbFinding.LimbFinding()
        limb.ModelTag = f"{self.name}_limbFinder"
        horizon = horizonOpNav.horizonOpNav()
        horizon.ModelTag = f"{self.name}_horizonNav"
        limb.imageInMsg.subscribeTo(cam_module.imageOutMsg)
        horizon.limbInMsg.subscribeTo(limb.opnavLimbOutMsg)
        horizon.cameraConfigInMsg.subscribeTo(cam_module.cameraConfigOutMsg)
        att_msg = messaging.NavAttMsg()
        horizon.attInMsg.subscribeTo(att_msg)
        horizon.planetTarget = OPNAV_PLANET_TARGET
        task = f"{self.name}OpNavTask"
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
        # CAM v2a: a recorder on the IMAGE port, same task, is the captured
        # witness. Recorder granularity is this task's tick: images that
        # complete between two ticks share one record, so Captured is a
        # floor, and its docstring says so rather than pretending otherwise.
        image_recorder = cam_module.imageOutMsg.recorder()
        ctx.sim.AddModelToTask(task, image_recorder)
        return (cam.camera_id, cam_module, horizon, att_msg, recorder, image_recorder)

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

        Same device the P4b bundle already samples: the exit demo fits the
        part to a vehicle whose manifest already had the hardware - the
        TOML line predates the registry, and REG's law is that presence of
        a REGISTERED part adds a FRAME, not a rewrite.
        """
        return self._st_sub is not None

    @property
    def has_plant_opnav(self) -> bool:
        """Whether this plant mounts the rendered camera that feeds OPNAV."""
        return self._opnav is not None

    def opnav_state_bytes(self, sim_time_s: float, frame: int) -> bytes:
        """Pack one ``OpNavState``: what this body's camera pixels say (CAM).

        Knowledge-channel content, nothing else - the body-frame estimate,
        its target and eye, and the estimator's own verdict flags. The image
        never travels: pixels that crossed the airgap would stop being a
        knowledge channel and start being a bandwidth problem.
        """
        assert self._opnav is not None, "opnav_state_bytes without a rendered eye"
        camera_id, _, _, _, recorder, _ = self._opnav
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
        """Whether this plant mounts a commandable eye (CAM v2a).

        The same [[camera]] line that feeds OPNAV: one hardware, two honest
        views - the estimator's opinion (opnav row) and the device's own
        confirmed state (camstate row).
        """
        return self._opnav is not None

    def cam_state_bytes(self, sim_time_s: float, frame: int) -> bytes:
        """Pack one ``CamState``: what this body's camera IS (CAM v2a).

        The device side of the commanded-vs-confirmed pair: read from the
        module's live fields, which Basilisk re-reads every update
        (camera.cpp) - the Phase-0 spike flew this conclusion before the
        schema was written. ``captured`` counts distinct image timeTags at
        estimator-task granularity, boot stream included; two images
        completing between consecutive recorder ticks would share one count,
        so this is a floor, and v2b's storage ledger is where the exact
        number lives.
        """
        assert self._opnav is not None, "cam_state_bytes without a rendered eye"
        camera_id, module, _, _, _, image_recorder = self._opnav
        # timeTag is the camera's own stamp on each received frame; distinct
        # positive values at recorder granularity is the honest capture door
        # (CameraImageMsg carries no writeTime - the recorder's payload fields
        # are exactly the message's, proven the hard way by a live flight).
        captured = len({t for t in image_recorder.timeTag if t > 0.0})
        return encode_container(
            "BSK_LINK/CamState",
            {
                "SimTimeS": float(sim_time_s),
                "Frame": int(frame),
                "CameraId": int(camera_id),
                "Shutter": int(module.cameraIsOn),
                "RenderRateS": float(module.renderRate) / 1e9,
                "Captured": captured,
            },
        )

    def apply_cam_config(self, cfg: CamConfig) -> None:
        """The sim-side config door (CAM v2a): validate, write, tally.

        Second guard on the same law the cam app enforces - zero render
        rate is a divide-by-zero inside vizInterface's request modulo (B1),
        and both sides of the airgap refuse it because a stale firmware on
        either side must not be able to kill the vehicle. Writes land on
        the module handle; the next Basilisk update sees them (spike-
        proven) and every digest stays byte-equal either way. Refusals
        tally instead of acking: this door answers nobody, and the ground
        reads the divergence (CAM_HK vs CamState), not a receipt.
        """
        assert self._opnav is not None, "apply_cam_config without a rendered eye"
        camera_id, module, _, _, _, _ = self._opnav
        if cfg.camera_id != camera_id:
            self._cam_stats["refused_id"] += 1
            print(
                f"[{self.name}] cam config refused: eye {cfg.camera_id} "
                f"not fitted (bird has {camera_id})"
            )
            return
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
        if self._opnav is not None:
            # The knowledge channel's attitude truth rides the same door:
            # horizonOpNav reads the LAST-written NavAttMsg on its own task,
            # and this is the only per-tick write point the rig gives a body.
            # The estimate was 0.73 degrees from truth with exactly this
            # feed - one tick old, which at these rates is this tick.
            _, _, _, att_msg, _, _ = self._opnav
            state = self._state_sub()
            payload = messaging.NavAttMsgPayload()
            payload.timeTag = self.sim_time_s
            payload.sigma_BN = np.asarray(state.sigma_BN, dtype=float).reshape(-1).tolist()
            payload.omega_BN_B = np.asarray(state.omega_BN_B, dtype=float).reshape(-1).tolist()
            att_msg.write(payload)

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
        """What the satellite measures.

        With a plant IMU (P4a) this is the sensor's own message: truth rate
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
        """Pack one ``OnboardState`` from the plant's own outputs (P4b).

        Exactly what the OBC would read off its buses at this instant: wheel
        feedback, ST quaternion (scalar-first, CASE wrt inertial), CSS cosines
        in TOML order, measured field. Channels the plant did not fit stay
        zero - that is the OBC's view saying "nothing here", not the harness
        inventing something. The clock and frame number come from the rig:
        one world, one tick numbering, every vehicle's OBC sees the same.

        REG: field names arrive as dict keys and the generic container codec
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
        """Pack one ``NavState``: this vehicle's plant receiver fix (KN2).

        The fix is the plant ``SimpleNav`` device's own message - truth plus
        the Gauss-Markov error process the TOML declared - read from the
        device's output message, never recomputed here. What the ground sees
        as truth stays on the truth socket: two wires, one universe, and P5
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
        """This vehicle's fit as its manifest declares it, as a bitmask (OB).

        A registry row is claimed when the scenario declares ANY of the
        bskforge types that row covers (the P4b bundle row covers four
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
    ) -> bytes:
        """Pack ``BSK_LINK/FitManifest``: the declared fit plus the catalog digest.

        An override lies about the BITS only. The digest stays the world's true
        catalog digest, because that is what the handshake checks: a lie that
        also moved the digest would look like a stale binary, and a test that
        simulates a missing sensor must not simulate a stale firmware.
        """
        return bytes(
            encode_fit_manifest(sim_time_s, frame, self.declared_fit(override), digest or FIT_HASH)
        )

    def attitude_state_bytes(self, sim_time_s: float, frame: int) -> bytes:
        """Pack one ``AttitudeState``: this vehicle's star tracker fix (REG exit part).

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
    ) -> None:
        """Load the TOML, build the graph, fix the schedule, initialize.

        Raises:
            FleetPolicyError: the TOML wires hardware in by hand, which the
                audited load refuses before anything is built.
            ForgeError: more than one spacecraft (that is a fleet - build a
                ``ForgeFleet``), a bskforge FSW section (D7), or a
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
            fit_provider=fit_provider(
                self.body, lambda: (rig.sim_time_s, rig.frame), fit_override, fit_digest
            ),
        )
        self._rig = rig

        ctx.sim.InitializeSimulation()

    # ---- rig delegations: identical harness surface to DetumbleScenario ----

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
    """One bskforge world, N vehicles, N gates on one clock (decision D10).

    Every ``[[spacecraft]]`` block becomes a :class:`ForgeBody` with its own
    rig channel and its own socket set, addressed by its block index through
    ``sim.fleet.ports`` - the file IS the fleet and nothing is declared
    twice. One tick: measure every body, publish every frame, gate every
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
    ) -> None:
        """Load the multi-vehicle TOML, build every body, initialize once.

        Raises:
            FleetPolicyError: the fleet file wires hardware in by hand, which
                the audited load refuses before anything is built.
            ForgeError: an empty fleet, a bskforge FSW section (D7, any
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
                    fit_provider=fit_provider(
                        body, lambda: (rig.sim_time_s, rig.frame), fit_override, fit_digest
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

    def flush_link(self) -> None:
        """Discard stale queue contents (every channel) before the first tick."""
        self._rig.flush_link()

    def close(self) -> None:
        """Release every channel's link sockets."""
        self._rig.close()

    def run_to(self, sim_time_s: float) -> None:
        """Advance the shared world, one gate-everyone tick per control period."""
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
        """Tripwire summed over gates (per-channel detail lives in channels)."""
        return self._rig.stale_torques

    @property
    def sensor_retransmits(self) -> int:
        """Handshake shouts summed over gates."""
        return self._rig.sensor_retransmits

    @property
    def foreign_frames(self) -> int:
        """Tripwire summed over gates."""
        return self._rig.foreign_frames

    @property
    def bad_frames(self) -> int:
        """Tripwire summed over gates."""
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
        help="lockstep link: publish EDS frames, own the torque to fcs, gate on step acks",
    )
    ap.add_argument(
        "--pace",
        action="store_true",
        help="align sim seconds to wall seconds (human watch mode; the gate stays in force)",
    )
    ap.add_argument("--ack-timeout", type=float, default=ACK_TIMEOUT_S, help="gate timeout [s]")
    ap.add_argument(
        "--fit-declare",
        action="append",
        default=[],
        metavar="PART",
        help="claim a part this vehicle does NOT declare (acceptance injection; OB)",
    )
    ap.add_argument(
        "--fit-hide",
        action="append",
        default=[],
        metavar="PART",
        help="conceal a part this vehicle DOES declare (acceptance injection; OB)",
    )
    ap.add_argument(
        "--fit-digest",
        metavar="HEX8",
        help="claim this registry digest instead of the world's own (stale-firmware injection; OB)",
    )
    add_link_args(ap)
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args(argv)
    try:
        args.fit_override, args.fit_digest = resolve_fit_knobs(
            args.fit_declare, args.fit_hide, args.fit_digest
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
    # parity and determinism gates both read it, and they demand equality.
    print(f"trajectory sha256 {trajectory_sha256(scenario.rate_history)}")
    print("DETUMBLE OK" if ratio < CONVERGENCE_RATIO else "DETUMBLE FAILED", flush=True)
    return 0 if ratio < CONVERGENCE_RATIO else 1


if __name__ == "__main__":
    raise SystemExit(main())
