"""EdsLib codecs for the Basilisk side of the link (ROADMAP D6).

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
    "FitManifest",
    "NavState",
    "OnboardState",
    "StepAck",
    "TorqueCommand",
    "available",
    "decode_fit_manifest",
    "decode_nav_state",
    "decode_onboard_state",
    "decode_step_ack",
    "decode_torque_command",
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
    """A decoded ``BSK_LINK/StepAck`` frame (the lockstep barrier, D5 amendment).

    ``mode`` is the fcs mode that answered the frame (0 disabled, 1 enabled,
    2 safe - the P4b-2 supervisor latch); ``torques_issued`` is the issuing
    counter at ack time, so a torque packet can still be matched by ``frame``
    alone while this keeps the audit honest.
    """

    sim_time_s: float
    frame: int
    mode: int
    torques_issued: int


@dataclass(frozen=True)
class OnboardState:
    """A decoded ``BSK_LINK/OnboardState`` frame (P4b observability companion).

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
    """A decoded ``BSK_LINK/FitManifest``: what the world claims is fitted (OB).

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


def encode_fit_manifest(
    sim_time_s: float, frame: int, declared_bits: int, registry_hash: str
) -> Any:  # noqa: ANN401 - stub-less C extension
    """Build a packed ``BSK_LINK/FitManifest`` (the boot handshake, OB).

    Hand-written like the barrier's own frames on purpose: the generic
    ``encode_container`` refuses containers that are not landed registry parts,
    and that refusal is the point - a part's frame must be registered to reach
    the bus. A control frame is not a part, so it encodes by name.
    """
    lib = _lib()
    obj = entry("BSK_LINK/FitManifest")()
    obj["SimTimeS"] = float(sim_time_s)
    obj["Frame"] = int(frame)
    obj["DeclaredBits"] = int(declared_bits)
    obj["RegistryHash"] = int(registry_hash, 16)
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
    )


def encode_truth_state(
    sim_time_s: float,
    r_bn_n: tuple[float, float, float],
    v_bn_n: tuple[float, float, float],
    sigma_bn: tuple[float, float, float, float],
    omega_bn_b: tuple[float, float, float],
) -> Any:  # noqa: ANN401 - stub-less C extension
    """Build a packed ``BSK_LINK/TruthState`` (ground truth stream, never on cFS)."""
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
    return lib.PackedObject(obj)


@lru_cache(maxsize=1)
def _registry_sizes() -> dict[str, int]:
    """EDS container -> declared payload bytes, from the parts registry."""
    from sim.parts import load_registry  # deferred: parts never imports this module

    return {p.eds_container: p.container_bytes for p in load_registry() if p.landed}


def encode_container(type_name: str, values: dict[str, float | int | tuple[float, ...]]) -> bytes:
    """Pack one observability container by EDS name (REG: the sim's generic provider).

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
            f"{sizes[type_name]} B - row and schema disagree (re-run the gates)"
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
    """A decoded ``BSK_LINK/NavState`` frame (KN: a GNSS opinion, never truth)."""

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


def encode_torque_command(
    sim_time_s: float, torque_sum_b: tuple[float, float, float], frame: int
) -> Any:  # noqa: ANN401 - stub-less C extension
    """Build a packed ``BSK_LINK/TorqueCommand`` (what the FSW will send in P3c)."""
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
    the ground-side fake-FSW tests that must answer the gate without a
    container. verify.py pins that the bytes match ``<dQb3xI``.
    """
    lib = _lib()
    obj = entry("BSK_LINK/StepAck")()
    obj["SimTimeS"] = sim_time_s
    obj["Frame"] = frame
    obj["Mode"] = mode
    obj["TorquesIssued"] = torques_issued
    return lib.PackedObject(obj)
