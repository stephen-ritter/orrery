"""Parts registry loader: the sensor circuit as data.

Pure ``tomllib`` like ``sim/fleet.py`` - codegen, checks, and CI read the
registry without importing Basilisk. The registry is authoritative for the
WIRE MEANING of a part (topic, MsgId, container, consumer, exhibit); the
bskforge schema stays authoritative for the PHYSICS type, and
``registered_bskforge_types`` is the seam: a row may only name a type the
catalog actually has.

The registry drives generators in stages of adoption; until a surface
is generated, ``tests/test_parts.py`` pins that the row AGREES with the
hand-maintained source - a registry that lies fails CI like a wrong struct.
"""

from __future__ import annotations

import hashlib
import tomllib
from collections.abc import Iterable, Mapping
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

REPO = Path(__file__).resolve().parent.parent
REGISTRY = REPO / "registry" / "parts.toml"

# A fit mask is a uint32 on the wire and in the generated header: enough
# for a decade of parts here, and a registry that outgrows it fails the loader
# rather than silently aliasing two parts onto one bit.
FIT_BIT_CAPACITY = 32

# The manifest carries this many declared-id slots (BSK_LINK/FitManifest):
# each id-capable frame row takes one, in frame order, and an id mask is a
# uint32 over ids 1..32. Rows outgrowing the slots fail the loader, never
# the boot - the same law as the fit bits, one tier down.
FIT_ID_SLOT_CAPACITY = 4


@dataclass(frozen=True)
class Exhibit:
    """The acceptance promise a part arrives with (the coverage rule's two sides)."""

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
    # payload apps this part brings onto a vehicle that mounts it; empty for
    # pure-row parts, whose frames ride the generic consumer. The startup
    # generator (codegen/emit_startup.py) turns these into wake-up lists.
    fsw_apps: tuple[str, ...] = ()
    # the mounts law: how many mount blocks of a named hardware type ONE
    # vehicle may carry (absent = single). A row may legislate only for the
    # types it names; rows naming one type must agree. The forge enforces
    # the law at world build - silently ignoring a duplicate mount was the
    # old silence, and silence is the one verdict this bench does not print.
    instances: dict[str, int] = field(default_factory=dict)
    # the instance id: the FIELD NAME in this row's container that says
    # WHICH device the frame carries (empty = the frame is one opinion with
    # no which). The boot handshake counts these per row, so a verdict can
    # name the instance that stalled, not just the part. The name resolves
    # against the EDS container at runtime: a typo streams no ids and the
    # handshake's own SILENT verdict names the claim that never arrived.
    instance_field: str = ""


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
        fsw_apps=tuple(row.get("fsw_apps", ())),
        instances={str(t): int(n) for t, n in row.get("instances", {}).items()},
        instance_field=str(row.get("instance_field", "")),
    )


def load_registry(path: str | Path = REGISTRY) -> list[Part]:
    """Every row, in file order, validated only for internal consistency.

    (Cross-checks against the real surfaces live in the test check.)
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
    _validate_mounts_laws(parts, path)
    _validate_instance_fields(parts, path)
    return parts


def _validate_instance_fields(parts: list[Part], path: str | Path) -> None:
    """An id is only countable on a frame that reaches firmware, in a slot that exists.

    Counting instances is the boot handshake's job, and the handshake only
    sees landed ``bsk_link`` rows: an id field on any other row is a promise
    no verdict can ever keep. And a fleet of id rows larger than the
    manifest's slots fails here, at the catalog, rather than at a boot where
    one row's ids would silently share another's mask.
    """
    for p in parts:
        if p.instance_field and not (p.landed and p.consumer == "bsk_link"):
            raise ValueError(
                f"{p.name}: instance_field counts ids on a frame firmware never carries ({path})"
            )
    cap = FIT_ID_SLOT_CAPACITY
    if len(id_rows(parts)) > cap:
        raise ValueError(f"{path}: id rows exceed the manifest's {cap} slots")


def _validate_mounts_laws(parts: list[Part], path: str | Path) -> None:
    """The mounts law grammar: legislate only for hardware you name, and never twice.

    A row fixes how many of a type one vehicle may carry only if the row
    carries that type's readings - a typo must not install a law for some
    other part's hardware. And where several rows name one type, they are
    several witnesses to one fact: a disagreement is refused like a row that
    lies about a topic, because the forge enforces whatever they agree on.
    """
    for p in parts:
        for t, limit in p.instances.items():
            if t not in p.bskforge_types:
                raise ValueError(
                    f"{p.name}: instances names {t}, which the row does not "
                    f"name as hardware ({path})"
                )
            if limit < 1:
                raise ValueError(
                    f"{p.name}: instances {t} = {limit}; a law of zero is a "
                    f"refusal to carry the part"
                )
    raised: dict[str, list[str]] = {}
    for p in parts:
        for t, limit in p.instances.items():
            raised.setdefault(t, []).append(f"{p.name}={limit}")
    for t, laws in raised.items():
        if len(set(laws)) > 1:
            raise ValueError(
                f"{t}: rows disagree on its mounts law: {', '.join(sorted(laws))} ({path})"
            )


def registered_bskforge_types() -> set[str]:
    """Every physics type the registry claims.

    Checked against bskforge's schema union by the test gate (catalog
    before factory).
    """
    return {t for part in load_registry() for t in part.bskforge_types}


def mount_limits(parts: list[Part] | None = None) -> dict[str, int]:
    """How many mounts of each registered type ONE vehicle may carry.

    Every type a row names is under law: single unless a naming row raises
    the number. Types NO row names are absent - unlegislated physics the
    wire does not carry (the plant IMU, the command-path effectors) is not
    the registry's to count.
    """
    rows = load_registry() if parts is None else parts
    limits = {t: 1 for p in rows for t in p.bskforge_types}
    for p in rows:
        for t, limit in p.instances.items():
            limits[t] = limit
    return limits


def mount_law_violations(mounts: Iterable[str], parts: list[Part] | None = None) -> list[str]:
    """Sentences for every type mounted past its law, in first-breach order.

    ``mounts`` names every sensor/effector block the vehicle mounts, one
    entry per block. The forge speaks these with the vehicle's name in
    front; the arithmetic lives here, bare of Basilisk, so the law is
    unit-testable on a list like every other verdict.
    """
    limits = mount_limits(parts)
    counts: dict[str, int] = {}
    for t in mounts:
        counts[t] = counts.get(t, 0) + 1
    return [
        f"{count} {t} mounts; the registry law is {limits[t]} per vehicle"
        for t, count in counts.items()
        if t in limits and count > limits[t]
    ]


# --- the fit vocabulary        -------------------------------------------
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
    generated C table is rendered in the same order, and the test check pins
    that both sides agree - so the same number means the same part in Python,
    in the header, and in the operator's event log.
    """
    return [p for p in parts if p.landed and p.consumer == "bsk_link"]


def id_rows(parts: list[Part]) -> list[Part]:
    """The frame rows whose payloads carry an instance id, in slot order.

    Slot k of the manifest's declared-id region belongs to the k-th row
    here - the same order the generated frame table renders, so one number
    means the same row in the manifest, the C table, and the verdict.
    """
    return [p for p in frame_rows(parts) if p.instance_field]


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
    """What a vehicle's manifest CLAIMS, as a fit mask.

    A row is claimed when the scenario declares ANY of the bskforge types that
    row covers (the onboard bundle row covers four instruments, so one CSS is
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


def declared_id_masks(
    claims: Mapping[str, Iterable[int]],
    inject: Mapping[str, Iterable[int]] | None = None,
    rows: list[Part] | None = None,
) -> list[int]:
    """One uint32 per id row: the instance ids this vehicle CLAIMS to carry.

    ``claims`` maps a bskforge type to the ids its mounts carry; a row
    claims the union over the hardware it names. ``inject`` is the
    acceptance lie - ids this manifest claims that no device carries - and
    the driver prints every injection, because no run may whisper one. An
    id outside the 1..32 mask is refused by name: a claim that cannot be
    counted is a claim no verdict could judge.
    """
    table = frame_rows(load_registry()) if rows is None else rows
    ids = [r for r in table if r.instance_field]
    if inject:
        unknown = sorted(set(inject) - {r.name for r in ids})
        if unknown:
            raise ValueError(f"no id row(s): {unknown} (id rows: {sorted(r.name for r in ids)})")
    masks: list[int] = []
    for row in ids:
        claimed: set[int] = set()
        for t in row.bskforge_types:
            claimed.update(claims.get(t, ()))
        if inject and row.name in inject:
            claimed.update(inject[row.name])
        for value in sorted(claimed):
            if not 1 <= value <= 32:
                raise ValueError(f"{row.name}: instance id {value} does not fit the 1..32 mask")
        masks.append(sum(1 << (value - 1) for value in claimed))
    return masks


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
