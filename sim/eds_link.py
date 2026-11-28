"""EdsLib codecs for the Basilisk side of the link.

The mission's EDS XML is the only description of what crosses the Basilisk <-> cFS
boundary. ``edslink/build.sh`` compiles it into a runtime database; this module loads
that database once and hands out encode/decode helpers, so no message layout is ever
written down twice in Python.

    from sim.eds_link import encode_sensor_frame, decode_torque_command
    sock.send(bytes(encode_sensor_frame(sim_time_s, omega, frame)))
    torque = decode_torque_command(body).torque_sum_b

The database and the CPython binding are built by ``edslink/build.sh`` into
``edslink/dist`` and travel together: the binding resolves the database through its
own RUNPATH, so importing this module only needs ``edslink/dist`` on ``sys.path``.
"""

from __future__ import annotations

import os
import sys
from collections.abc import Iterable
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path
from typing import Any

REPO = Path(__file__).resolve().parents[1]
# EDSLINK_DIST is already how edslink/build.sh is told where artifacts go; the
# importer honors the same knob, because the binding must match the importing
# interpreter and one checkout now serves two: a cp314 world on the host and a
# cp312 world in orrery-bsk:dev. The container sets this to its own build -
# host dist in a container python is not an old artifact, it's a broken one
# (ImportError: undefined symbol: PyWeakref_GetRef was the W3 spike's first
# container-world verdict: a C-extension mismatch measured as physics).
DIST = Path(os.environ.get("EDSLINK_DIST") or REPO / "edslink" / "dist")
DB_NAME = "orrery"

__all__ = [
    "BurnCommand",
    "FitManifest",
    "NavState",
    "OnboardState",
    "OpNavState",
    "StepAck",
    "TorqueCommand",
    "available",
    "decode_burn_command",
    "decode_fit_manifest",
    "decode_nav_state",
    "decode_onboard_state",
    "decode_opnav_state",
    "decode_step_ack",
    "decode_torque_command",
    "encode_burn_command",
    "encode_container",
    "encode_fit_manifest",
    "encode_sensor_frame",
    "encode_truth_state",
    "entry",
]


class LinkUnavailable(RuntimeError):
    """Raised when the EdsLib database has not been built yet."""


def available() -> bool:
    """True when ``edslink/build.sh`` has produced the database and binding."""
    return (DIST / "EdsLib.so").is_file() and (DIST / f"{DB_NAME}_eds_db.so").is_file()


def _require() -> None:
    if not available():
        raise LinkUnavailable("run edslink/build.sh to build the EDS runtime database")
    if str(DIST) not in sys.path:
        sys.path.insert(0, str(DIST))


@lru_cache(maxsize=1)
def _lib() -> Any:  # noqa: ANN401 - ty: ignore[missing-return]  # stub-less C extension
    _require()
    import EdsLib  # ty: ignore[import-not-found]

    return EdsLib


@lru_cache(maxsize=64)
def entry(name: str) -> Any:  # noqa: ANN401 - ty: ignore[missing-return]  # stub-less C extension
    """Cached EdsLib type object for ``PKG/Container`` (the binding is slow to look up)."""
    lib = _lib()
    return lib.Database(DB_NAME).Entry(name)


@dataclass(frozen=True)
class TorqueCommand:
    """A decoded ``BSK_LINK/TorqueCommand`` frame."""

    sim_time_s: float
    torque_sum_b: tuple[float, float, float]
    frame: int


@dataclass(frozen=True)
class StepAck:
    """A decoded ``BSK_LINK/StepAck`` frame (the lockstep barrier).

    ``mode`` is the fcs mode that answered the frame (0 disabled, 1 enabled,
    2 safe - the supervisor latch); ``torques_issued`` is the issuing
    counter at ack time, so a torque packet can still be matched by ``frame``
    alone while this keeps the audit honest.
    """

    sim_time_s: float
    frame: int
    mode: int
    torques_issued: int


@dataclass(frozen=True)
class OnboardState:
    """A decoded ``BSK_LINK/OnboardState`` frame (observability companion).

    Everything here is what the OBC's own instruments and actuators report:
    wheel feedback, ST quaternion (scalar-first), CSS cosines in TOML order,
    measured magnetic field. Navigator truth never rides this frame.
    """

    sim_time_s: float
    frame: int
    wheel_speeds: tuple[float, float, float]
    st_quat_bn: tuple[float, float, float, float]
    css_cos: tuple[float, float, float, float]
    mag_b: tuple[float, float, float]


def encode_sensor_frame(
    sim_time_s: float, omega_bn_b: tuple[float, float, float], frame: int
) -> Any:  # noqa: ANN401 - stub-less C extension
    """Build a packed ``BSK_LINK/SensorFrame`` (returns an object for ``bytes()``)."""
    lib = _lib()
    obj = entry("BSK_LINK/SensorFrame")()
    obj["SimTimeS"] = sim_time_s
    for index, value in enumerate(omega_bn_b):
        obj["OmegaBnB"][index] = value
    obj["Frame"] = frame
    return lib.PackedObject(obj)


@dataclass(frozen=True)
class FitManifest:
    """A decoded ``BSK_LINK/FitManifest``: what the world claims is fitted.

    ``declared_bits`` are the bits of ``frame_rows`` order (sim.parts), and
    ``registry_hash`` is the registry's semantics digest - the two numbers the
    spacecraft compares against what it was BUILT with and what its bus
    actually sees. Nothing else crosses on this frame: a handshake that needs
    the whole catalog would be a config file in a packet costume.
    """

    sim_time_s: float
    frame: int
    declared_bits: int
    registry_hash: str
    # one mask per manifest id slot (bit n is id n+1); slot order is
    # sim.parts.id_rows, and an all-zero slot claims no instance - either no
    # id row occupies it or nothing is fitted there
    declared_ids: tuple[int, ...] = (0, 0, 0, 0)


def encode_fit_manifest(
    sim_time_s: float,
    frame: int,
    declared_bits: int,
    registry_hash: str,
    declared_ids: Iterable[int] = (),
) -> Any:  # noqa: ANN401 - stub-less C extension
    """Build a packed ``BSK_LINK/FitManifest`` (the boot handshake).

    Hand-written like the barrier's own frames on purpose: the generic
    ``encode_container`` refuses containers that are not landed registry parts,
    and that refusal is the point - a part's frame must be registered to reach
    the bus. A control frame is not a part, so it encodes by name.

    ``declared_ids`` fills the instance-attribution slots in order; a longer
    sequence than the container's four slots is refused here, at the edge of
    the world, rather than as a decode error inside the spacecraft.
    """
    lib = _lib()
    ids = list(declared_ids)
    if len(ids) > 4:
        raise ValueError(f"{len(ids)} declared-id masks exceed the manifest's 4 slots")
    obj = entry("BSK_LINK/FitManifest")()
    obj["SimTimeS"] = float(sim_time_s)
    obj["Frame"] = int(frame)
    obj["DeclaredBits"] = int(declared_bits)
    obj["RegistryHash"] = int(registry_hash, 16)
    for slot, mask in enumerate(ids):
        obj[f"DeclaredIds{slot}"] = int(mask)
    return lib.PackedObject(obj)


def decode_fit_manifest(payload: bytes) -> FitManifest:
    """Decode a fit manifest (tests and the operator's audit tool read it back)."""
    lib = _lib()
    obj = entry("BSK_LINK/FitManifest")(lib.PackedObject(payload))
    return FitManifest(
        sim_time_s=float(obj["SimTimeS"]()),
        frame=int(obj["Frame"]()),
        declared_bits=int(obj["DeclaredBits"]()),
        registry_hash=f"{int(obj['RegistryHash']()):08x}",
        declared_ids=tuple(int(obj[f"DeclaredIds{slot}"]()) for slot in range(4)),
    )


def encode_truth_state(
    sim_time_s: float,
    r_bn_n: tuple[float, float, float],
    v_bn_n: tuple[float, float, float],
    sigma_bn: tuple[float, float, float, float],
    omega_bn_b: tuple[float, float, float],
    *,
    burn_impulse_applied_ns: float = 0.0,
    burn_ticks_applied: float = 0.0,
    burn_ticks_saturated: float = 0.0,
    burn_id: float = 0.0,
    fuel_remaining_kg: float = float("nan"),
    fuel_spent_kg: float = float("nan"),
    fuel_ticks_starved: float = 0.0,
    fuel_ticks_throttled: float = 0.0,
) -> Any:  # noqa: ANN401 - stub-less C extension
    """Build a packed ``BSK_LINK/TruthState`` (ground truth stream, never on cFS).

    The burn ledger defaults to silence: a plant that never thrusts says so
    with zeros, and every pre-burn caller stays untouched. The fuel rows
    default to the other idiom: NaN declares "this vehicle has no tank"
    where zero would lie about stock; the event counts are true zeros.
    """
    lib = _lib()
    obj = entry("BSK_LINK/TruthState")()
    obj["SimTimeS"] = sim_time_s
    for name, values in (
        ("r_BN_N", r_bn_n),
        ("v_BN_N", v_bn_n),
        ("sigma_BN", sigma_bn),
        ("omega_BN_B", omega_bn_b),
    ):
        for index, value in enumerate(values):
            obj[name][index] = value
    obj["BurnImpulseAppliedNs"] = burn_impulse_applied_ns
    obj["BurnTicksApplied"] = burn_ticks_applied
    obj["BurnTicksSaturated"] = burn_ticks_saturated
    obj["BurnId"] = burn_id
    obj["FuelRemainingKg"] = fuel_remaining_kg
    obj["FuelSpentKg"] = fuel_spent_kg
    obj["FuelTicksStarved"] = fuel_ticks_starved
    obj["FuelTicksThrottled"] = fuel_ticks_throttled
    return lib.PackedObject(obj)


@lru_cache(maxsize=1)
def _registry_sizes() -> dict[str, int]:
    """EDS container -> declared payload bytes, from the parts registry."""
    from sim.parts import load_registry  # deferred: parts never imports this module

    return {p.eds_container: p.container_bytes for p in load_registry() if p.landed}


def encode_container(type_name: str, values: dict[str, float | int | tuple[float, ...]]) -> bytes:
    """Pack one observability container by EDS name (the sim's generic provider).

    The field names are the contract: they come from the schema and arrive as
    dict keys, so no per-container encode function is ever written again -
    a registry row plus its XML container is the whole codec. The packed
    length is checked against the registry's declared bytes, so a provider
    packing a shape that disagrees with the row fails on the SIM side, at the
    edge of the world, instead of as a decode error inside the spacecraft.
    """
    sizes = _registry_sizes()
    if type_name not in sizes:
        raise LinkUnavailable(
            f"{type_name} is not a landed registry container - observability codecs "
            "come from registry/parts.toml"
        )
    lib = _lib()
    obj = entry(type_name)()
    for name, value in values.items():
        if isinstance(value, (tuple, list)):
            for index, element in enumerate(value):
                obj[name][index] = float(element)  # EdsLib rejects numpy scalars
        elif isinstance(value, int) and not isinstance(value, bool):
            obj[name] = int(value)
        else:
            obj[name] = float(value)
    payload = bytes(lib.PackedObject(obj))
    if len(payload) != sizes[type_name]:
        raise LinkUnavailable(
            f"{type_name} packed to {len(payload)} B but the registry declares "
            f"{sizes[type_name]} B - row and schema disagree (re-run the checks)"
        )
    return payload


def decode_onboard_state(payload: bytes) -> OnboardState:
    """Decode the body of a ``BSK_LINK/OnboardState`` frame."""
    lib = _lib()
    obj = entry("BSK_LINK/OnboardState")(lib.PackedObject(payload))
    return OnboardState(
        sim_time_s=obj["SimTimeS"](),
        frame=obj["Frame"](),
        wheel_speeds=tuple(obj["WheelSpeeds"][i]() for i in range(3)),
        st_quat_bn=tuple(obj["StQuatBn"][i]() for i in range(4)),
        css_cos=tuple(obj["CssCos"][i]() for i in range(4)),
        mag_b=tuple(obj["MagB"][i]() for i in range(3)),
    )


@dataclass(frozen=True)
class NavState:
    """A decoded ``BSK_LINK/NavState`` frame (a GNSS opinion, never truth)."""

    sim_time_s: float
    frame: int
    r_bn_n: tuple[float, float, float]
    v_bn_n: tuple[float, float, float]


def decode_nav_state(payload: bytes) -> NavState:
    """Decode the body of a ``BSK_LINK/NavState`` frame."""
    lib = _lib()
    obj = entry("BSK_LINK/NavState")(lib.PackedObject(payload))
    return NavState(
        sim_time_s=obj["SimTimeS"](),
        frame=obj["Frame"](),
        r_bn_n=tuple(obj["RBnN"][i]() for i in range(3)),
        v_bn_n=tuple(obj["VBnN"][i]() for i in range(3)),
    )


@dataclass(frozen=True)
class OpNavState:
    """A decoded ``BSK_LINK/OpNavState`` frame (CAM: planet knowledge from pixels)."""

    sim_time_s: float
    frame: int
    r_bn_b: tuple[float, float, float]
    planet_id: int
    camera_id: int
    valid: bool
    fault_detected: bool


def decode_opnav_state(payload: bytes) -> OpNavState:
    """Decode the body of a ``BSK_LINK/OpNavState`` frame."""
    lib = _lib()
    obj = entry("BSK_LINK/OpNavState")(lib.PackedObject(payload))
    return OpNavState(
        sim_time_s=obj["SimTimeS"](),
        frame=obj["Frame"](),
        r_bn_b=tuple(obj["RBnB"][i]() for i in range(3)),
        planet_id=int(obj["PlanetId"]()),
        camera_id=int(obj["CameraId"]()),
        valid=bool(obj["Valid"]()),
        fault_detected=bool(obj["FaultDetected"]()),
    )


@dataclass(frozen=True)
class CamState:
    """A decoded ``BSK_LINK/CamState`` frame (confirmed device state).

    The sim's side of the commanded-vs-confirmed pair: CAM_HK carries what the
    ground asked for, this carries what the camera module is actually running.
    ``captured`` counts distinct image writes seen since boot at recorder
    granularity - the firstPass boot stream included, because it is what the
    camera captured, as honesty demands. ``frames_recorded`` counts the
    frames the STORE OUTLET has sent: every ``record_rate``-th capture whose
    PNG bytes were chunked onto the wire. Zero means the camera is nav-only -
    an absence, not a failure. ``snapshots`` counts shutter-pulse deliveries:
    frames that left the shutter because the ground pulsed it, outside the
    record cadence - the delivery half of the pair with CAM_HK.SnapshotCmds.
    """

    sim_time_s: float
    frame: int
    camera_id: int
    shutter: int
    render_rate_s: float
    captured: int
    frames_recorded: int
    snapshots: int = 0


def decode_cam_state(payload: bytes) -> CamState:
    """Decode the body of a ``BSK_LINK/CamState`` frame (audit tools, tests)."""
    lib = _lib()
    obj = entry("BSK_LINK/CamState")(lib.PackedObject(payload))
    return CamState(
        sim_time_s=obj["SimTimeS"](),
        frame=int(obj["Frame"]()),
        camera_id=int(obj["CameraId"]()),
        shutter=int(obj["Shutter"]()),
        render_rate_s=float(obj["RenderRateS"]()),
        captured=int(obj["Captured"]()),
        frames_recorded=int(obj["FramesRecorded"]()),
        snapshots=int(obj["Snapshots"]()),
    )


@dataclass(frozen=True)
class CamConfig:
    """A decoded ``BSK_LINK/CamConfig`` frame (cFS -> sim).

    One accepted ``CAM_SET_CONFIG`` leaving the bus. Fire-and-forget: nothing
    on this frame is a barrier, and a sim that cannot honour it (no such eye,
    zero rate) refuses by counter, never by an ack the physics would wait on.

    ``pulse`` is the shutter-pulse edge (FC 2 SNAP sets it for one frame):
    the sim opens the shutter if closed, delivers the NEXT capture to the
    store outside the record cadence, and restores. It rides the same frame
    as the steady config because it IS a config - "look now, at the settings
    you have" - and a one-frame edge needs no second MID.
    """

    sim_time_s: float
    frame: int
    camera_id: int
    shutter: int
    render_rate_s: float
    pulse: int = 0


def decode_cam_config(payload: bytes) -> CamConfig:
    """Decode the body of a ``BSK_LINK/CamConfig`` frame (the sim's config door)."""
    lib = _lib()
    obj = entry("BSK_LINK/CamConfig")(lib.PackedObject(payload))
    return CamConfig(
        sim_time_s=obj["SimTimeS"](),
        frame=int(obj["Frame"]()),
        camera_id=int(obj["CameraId"]()),
        shutter=int(obj["Shutter"]()),
        render_rate_s=float(obj["RenderRateS"]()),
        pulse=int(obj["Pulse"]()),
    )


def encode_cam_config(
    sim_time_s: float,
    frame: int,
    camera_id: int,
    shutter: int,
    render_rate_s: float,
    pulse: int = 0,
) -> Any:  # noqa: ANN401 - stub-less C extension
    """Build a packed ``BSK_LINK/CamConfig``.

    Production frames are egressed by ``bsk_link`` from the bus; this encoder
    exists for tests that drive the sim's config door without a container -
    the same fake-FSW role ``encode_step_ack`` plays at the barrier.
    """
    lib = _lib()
    obj = entry("BSK_LINK/CamConfig")()
    obj["SimTimeS"] = sim_time_s
    obj["Frame"] = frame
    obj["CameraId"] = camera_id
    obj["Shutter"] = shutter
    obj["RenderRateS"] = render_rate_s
    obj["Pulse"] = pulse
    return lib.PackedObject(obj)


@dataclass(frozen=True)
class BurnCommand:
    """A decoded ``BSK_LINK/BurnCommand`` frame: one tick of a commanded burn.

    The craft's claim, delivered apply-once: this tick, this force along this
    body axis. ``burn_id`` is the craft's burn ordinal so the world's applied
    ledger and the craft's forwarded ledger can be joined per burn by the
    bench - neither side reads the other; only the bench does.
    """

    sim_time_s: float
    force_n: float
    axis_b: tuple[float, float, float]
    frame: int
    burn_id: int


def decode_burn_command(payload: bytes) -> BurnCommand:
    """Decode the body of a ``BSK_LINK/BurnCommand`` frame (the sim's burn door)."""
    lib = _lib()
    obj = entry("BSK_LINK/BurnCommand")(lib.PackedObject(payload))
    return BurnCommand(
        sim_time_s=float(obj["SimTimeS"]()),
        force_n=float(obj["ForceN"]()),
        axis_b=tuple(obj["AxisB"][i]() for i in range(3)),
        frame=int(obj["Frame"]()),
        burn_id=int(obj["BurnId"]()),
    )


def encode_burn_command(
    sim_time_s: float,
    force_n: float,
    axis_b: tuple[float, float, float],
    frame: int,
    burn_id: int,
) -> Any:  # noqa: ANN401 - stub-less C extension
    """Build a packed ``BSK_LINK/BurnCommand``.

    Production frames are egressed by ``bsk_link`` from the bus; this encoder
    exists for tests that drive the sim's burn door without a container -
    ``encode_cam_config``'s fake-FSW role again.
    """
    lib = _lib()
    obj = entry("BSK_LINK/BurnCommand")()
    obj["SimTimeS"] = sim_time_s
    obj["ForceN"] = force_n
    for index, value in enumerate(axis_b):
        obj["AxisB"][index] = value
    obj["Frame"] = frame
    obj["BurnId"] = burn_id
    return lib.PackedObject(obj)


def encode_torque_command(
    sim_time_s: float, torque_sum_b: tuple[float, float, float], frame: int
) -> Any:  # noqa: ANN401 - stub-less C extension
    """Build a packed ``BSK_LINK/TorqueCommand`` (what the FSW sends)."""
    lib = _lib()
    obj = entry("BSK_LINK/TorqueCommand")()
    obj["SimTimeS"] = sim_time_s
    for index, value in enumerate(torque_sum_b):
        obj["TorqueSumB"][index] = value
    obj["Frame"] = frame
    return lib.PackedObject(obj)


def decode_torque_command(payload: bytes) -> TorqueCommand:
    """Decode the body of a ``BSK_LINK/TorqueCommand`` frame."""
    lib = _lib()
    obj = entry("BSK_LINK/TorqueCommand")(lib.PackedObject(payload))
    return TorqueCommand(
        sim_time_s=obj["SimTimeS"](),
        torque_sum_b=tuple(obj["TorqueSumB"][i]() for i in range(3)),
        frame=obj["Frame"](),
    )


def decode_step_ack(payload: bytes) -> StepAck:
    """Decode the body of a ``BSK_LINK/StepAck`` frame (fcs -> sim barrier)."""
    lib = _lib()
    obj = entry("BSK_LINK/StepAck")(lib.PackedObject(payload))
    return StepAck(
        sim_time_s=obj["SimTimeS"](),
        frame=obj["Frame"](),
        mode=obj["Mode"](),
        torques_issued=obj["TorquesIssued"](),
    )


def encode_step_ack(sim_time_s: float, frame: int, mode: int, torques_issued: int) -> Any:  # noqa: ANN401 - stub-less C extension
    """Build a packed ``BSK_LINK/StepAck``.

    Production acks come from ``fcs`` over the bus; this encoder exists for
    the ground-side fake-FSW tests that must answer the check without a
    container. verify.py pins that the bytes match ``<dQb3xI``.
    """
    lib = _lib()
    obj = entry("BSK_LINK/StepAck")()
    obj["SimTimeS"] = sim_time_s
    obj["Frame"] = frame
    obj["Mode"] = mode
    obj["TorquesIssued"] = torques_issued
    return lib.PackedObject(obj)
