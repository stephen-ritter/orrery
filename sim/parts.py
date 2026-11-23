"""Parts registry loader (REG, D13): the sensor circuit as data.

Pure ``tomllib`` like ``sim/fleet.py`` - codegen, gates, and CI read the
registry without importing Basilisk. The registry is authoritative for the
WIRE MEANING of a part (topic, MsgId, container, consumer, exhibit); the
bskforge schema stays authoritative for the PHYSICS type, and
``registered_bskforge_types`` is the seam: a row may only name a type the
catalog actually has.

The registry drives generators in stages (REG increments); until a surface
is generated, ``tests/test_parts.py`` pins that the row AGREES with the
hand-maintained source - a registry that lies fails CI like a wrong struct.
"""

from __future__ import annotations

import hashlib
import tomllib
from collections.abc import Iterable
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

REPO = Path(__file__).resolve().parent.parent
REGISTRY = REPO / "registry" / "parts.toml"

# A fit mask is a uint32 on the wire and in the generated header (OB): enough
# for a decade of parts here, and a registry that outgrows it fails the loader
# rather than silently aliasing two parts onto one bit.
FIT_BIT_CAPACITY = 32


@dataclass(frozen=True)
class Exhibit:
    """The acceptance promise a part arrives with (p5's two-sided rule)."""

    kind: str  # "truth_pair" today; camera's truth-vs-image joins at CAM
    flight_packet: str
    truth_packet: str
    flight_pos: str
    truth_pos: str
    floor: float  # multiplier on sigma: under floor*sigma the wire is an oracle
    ceiling: str  # "3sqrt3" | "declared": over it the wires describe two worlds
    epoch: str  # "strict": raw pairs may not cross any epoch skew
    flight_vel: str = ""  # rate pair: empty when the part has no rate opinion
    truth_vel: str = ""
    # how the pair is compared: "vector3" = componentwise R/V-style vectors
    # bounded by (pos_sigma, vel_sigma); "rotation_mrp" = flight quaternion
    # (scalar-first) against truth MRP, one angle gap bounded by angle_sigma
    compare: str = "vector3"


@dataclass(frozen=True)
class Part:
    """One registry row: the part's physics type, wire identity, consumers."""

    name: str
    display: str
    kind: str  # "opinion" (one part, one frame) | "bundle" (legacy multi-part frame)
    landed: bool  # the circuit exists end to end; rows may register ahead of it
    bskforge_types: tuple[str, ...]
    plant_output: str
    zmq_topic: str
    eds_container: str
    container_bytes: int
    topic_offset: int
    topic_name: str
    mid_name: str
    evs_info: str
    evs_info_id: int
    evs_label: str
    consumer: str
    subscribers: tuple[str, ...] = ()
    declared: dict[str, str] = field(default_factory=dict)
    exhibit: Exhibit | None = None


def _row_to_part(row: dict[str, Any]) -> Part:
    exhibit_raw = row.get("exhibit")
    exhibit = Exhibit(**exhibit_raw) if exhibit_raw else None
    types = row.get("bskforge_types")
    if types is None:
        types = (row["bskforge_type"],) if "bskforge_type" in row else ()
    return Part(
        name=row["name"],
        display=row["display"],
        kind=row.get("kind", "opinion"),
        landed=bool(row.get("landed", False)),
        bskforge_types=tuple(types),
        plant_output=row.get("plant_output", ""),
        zmq_topic=row["zmq_topic"],
        eds_container=row["eds_container"],
        container_bytes=int(row["container_bytes"]),
        topic_offset=int(row["topic_offset"]),
        topic_name=row["topic_name"],
        mid_name=row["mid_name"],
        evs_info=row.get("evs_info", ""),
        evs_info_id=int(row.get("evs_info_id", 0)),
        evs_label=row.get("evs_label", ""),
        consumer=row["consumer"],
        subscribers=tuple(row.get("subscribers", ())),
        declared=dict(row.get("declared", {})),
        exhibit=exhibit,
    )


def load_registry(path: str | Path = REGISTRY) -> list[Part]:
    """Every row, in file order, validated only for internal consistency.

    (Cross-checks against the real surfaces live in the test gate.)
    """
    with Path(path).open("rb") as fh:
        doc = tomllib.load(fh)
    parts = [_row_to_part(row) for row in doc.get("part", [])]
    names = [p.name for p in parts]
    if len(names) != len(set(names)):
        raise ValueError(f"duplicate part names in {path}: {names}")
    offsets = [p.topic_offset for p in parts]
    if len(offsets) != len(set(offsets)):
        raise ValueError(f"duplicate topic_offset in {path}: {offsets}")
    mids = [p.mid_name for p in parts]
    if len(mids) != len(set(mids)):
        raise ValueError(f"duplicate mid_name in {path}: {mids}")
    topics = [p.zmq_topic for p in parts]
    if len(topics) != len(set(topics)):
        raise ValueError(f"duplicate zmq_topic in {path}: {topics}")
    return parts


def registered_bskforge_types() -> set[str]:
    """Every physics type the registry claims.

    Checked against bskforge's schema union by the test gate (catalog
    before factory, D13).
    """
    return {t for part in load_registry() for t in part.bskforge_types}


# --- the fit vocabulary (OB, D13) -------------------------------------------
#
# The boot handshake needs one agreed way to say WHICH parts a vehicle has,
# small enough to cross the airgap in eight bytes and stable enough that a
# flight binary and a running world can compare it without a human in the
# middle. That way is a bitmask over ``frame_rows`` order plus a digest of the
# rows' SEMANTICS: the binary bakes both in (codegen), the world recomputes
# both at runtime, and a disagreement means one of them is stale - the class of
# lie no size check can catch (a stale binary has a perfectly self-consistent
# size).


def frame_rows(parts: list[Part]) -> list[Part]:
    """The rows that exist in firmware: landed, consumed by ``bsk_link``.

    Registry order, unsorted: the bit index of a part IS its index here, the
    generated C table is rendered in the same order, and the test gate pins
    that both sides agree - so the same number means the same part in Python,
    in the header, and in the operator's event log.
    """
    return [p for p in parts if p.landed and p.consumer == "bsk_link"]


def fit_bit(name: str) -> int:
    """The single bit a part occupies in a fit mask (``frame_rows`` order)."""
    names = [row.name for row in frame_rows(load_registry())]
    if len(names) > FIT_BIT_CAPACITY:
        raise ValueError(f"{len(names)} frame rows exceed the {FIT_BIT_CAPACITY}-bit fit mask")
    if name not in names:
        raise ValueError(f"{name}: no fit bit (a row must be landed with consumer=bsk_link)")
    return 1 << names.index(name)


def fit_mask(names: Iterable[str]) -> int:
    """A fit mask from part names - the eight bytes a vehicle's fit becomes."""
    mask = 0
    for name in names:
        mask |= fit_bit(name)
    return mask


def declared_mask(
    bskforge_types: Iterable[str],
    override: dict[str, bool] | None = None,
    rows: list[Part] | None = None,
) -> int:
    """What a vehicle's manifest CLAIMS, as a fit mask (OB).

    A row is claimed when the scenario declares ANY of the bskforge types that
    row covers (the P4b bundle row covers four instruments, so one CSS is
    enough to claim the frame). ``override`` injects a deliberate lie for
    acceptance - claim a part that is not there, hide one that is - and the
    driver prints every override, because no run may whisper one.

    Kept free of Basilisk and of ``self`` so the acceptance logic is unit-testable
    without a plant: the handshake's whole job is to catch two sources of truth
    disagreeing, so its own arithmetic must be checkable on a bare list.
    """
    declared_types = set(bskforge_types)
    mask = 0
    for row in frame_rows(load_registry()) if rows is None else rows:
        claimed = bool(declared_types & set(row.bskforge_types))
        if override and row.name in override:
            claimed = override[row.name]
        if claimed:
            mask |= fit_bit(row.name)
    return mask


def registry_digest(parts: list[Part]) -> str:
    """8 hex chars over the rows' SEMANTICS - what a firmware is built from.

    Deliberately blind to ``display`` text, comments and ordering-only edits of
    unrelated fields: a reworded comment must never demand a mission rebuild,
    while a new topic, container, size or EVS id must. The generator bakes this
    into ``orrery_frames.h`` and the sim recomputes it, so a binary older than
    the catalog says so at boot instead of mis-decoding the new frame.
    """
    payload = "\n".join(
        "|".join(
            (
                row.name,
                row.zmq_topic,
                row.eds_container,
                str(row.container_bytes),
                str(row.topic_offset),
                row.mid_name,
                str(row.evs_info_id),
            )
        )
        for row in frame_rows(parts)
    )
    return hashlib.sha256(payload.encode()).hexdigest()[:8]
