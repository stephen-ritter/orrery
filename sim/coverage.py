"""Coverage ledger loader and renderer (D13).

`registry/coverage.toml` is the authoritative statement of which parts bskforge
exposes and what this project does with each; `docs/coverage.md` is this module's
output. Pure ``tomllib`` like ``sim/parts.py`` - importing bskforge's *schema*
happens only inside the union helpers, so the loader works before Basilisk is
importable and the test gate can decide when to pay for it.

The point of rendering rather than writing prose: a hand-maintained "we support
N of M" line is folklore. Here the count is computed from the rows, and
``tests/test_coverage.py`` refuses any edit to the document that is not the
output of this function - the same discipline that made the generated identity
headers trustworthy in REG.
"""

from __future__ import annotations

import argparse
import tomllib
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, get_args

REPO = Path(__file__).resolve().parent.parent
COVERAGE = REPO / "registry" / "coverage.toml"
DOC = REPO / "docs" / "coverage.md"

BEGIN = "<!-- BEGIN GENERATED (registry/coverage.toml via sim/coverage.py) -->"
END = "<!-- END GENERATED -->"

# Five states, each with a hard meaning; see the ledger header for the prose.
STATUSES = frozenset({"landed", "barrier", "catalog-unlanded", "command-path", "refused"})
AREA_STATUSES = frozenset({"used", "unused-with-reason", "banned"})

# The first missing hop, from the vocabulary docs/adding_a_sensor.md uses for a
# part's circuit. Fixed, because "somewhere, eventually" is not a cost.
FIRST_HOPS = frozenset(
    {
        "",  # nothing missing: it flies
        "scenario-block",
        "registry-row",
        "container-xml",
        "provider",
        "consumer",
        "compare-mode",
        "command-path",
        "fsw-command",
        "gate",
        "decision",
    }
)

# Owners, in words a reader can act on. registry/coverage.toml keeps the project's
# own short codes - that is what the working plan indexes gaps by - and the
# generated page translates them, because someone deciding what to build next
# needs to know WHO will close a gap, not what a private schedule calls it. A
# code with no label fails tests/test_docs_vocabulary.py rather than printing a
# puzzle in the page.
OWNER_LABELS = {
    "P7": "the per-configuration digest round",
    "P9": "the control-authority round",
    "CAM": "the imaging round",
    "P8b": "the constellation round",
}

KINDS = frozenset({"sensor", "effector", "disturbance"})

_TYPE_KEYS = frozenset(
    {
        "name",
        "kind",
        "status",
        "bskforge",
        "wire",
        "command_path",
        "first_hop",
        "scenarios",
        "phase",
        "why",
    }
)
_AREA_KEYS = frozenset({"name", "status", "why"})


@dataclass(frozen=True)
class TypeEntry:
    """One part type, as the project stands on it."""

    name: str
    kind: str  # "sensor" | "effector" | "disturbance"
    status: str
    bskforge: str
    wire: str = ""
    command_path: str = ""
    first_hop: str = ""
    scenarios: tuple[str, ...] = ()
    phase: str = ""
    why: str = ""

    @property
    def supported(self) -> bool:
        """Whether a reading of the bus can see this part today."""
        return self.status in {"landed", "barrier"}


@dataclass(frozen=True)
class AreaEntry:
    """One scenario-section: the other half of "do we support everything"."""

    name: str
    status: str
    why: str = ""


@dataclass(frozen=True)
class Coverage:
    """The whole ledger, with the queries the tests and the doc need."""

    types: tuple[TypeEntry, ...]
    areas: tuple[AreaEntry, ...]
    _by_name: dict[str, TypeEntry] = field(default_factory=dict, repr=False, compare=False)

    def __post_init__(self) -> None:
        """Index by type name without letting a mutable dict enter __eq__."""
        object.__setattr__(self, "_by_name", {t.name: t for t in self.types})

    def by_name(self, name: str) -> TypeEntry | None:
        """The entry for a type, or None: callers decide whether absent is a bug."""
        return self._by_name.get(name)

    @property
    def sensors(self) -> list[TypeEntry]:
        """Sensor rows, in ledger order."""
        return [t for t in self.types if t.kind == "sensor"]

    @property
    def effectors(self) -> list[TypeEntry]:
        """Actuator rows - the parts that push."""
        return [t for t in self.types if t.kind == "effector"]

    @property
    def disturbances(self) -> list[TypeEntry]:
        """Disturbance-model rows: they actuate too, but nobody commands them."""
        return [t for t in self.types if t.kind == "disturbance"]

    def names_by_status(self, status: str) -> set[str]:
        """Every type carrying a status - the shape most of the audits want."""
        return {t.name for t in self.types if t.status == status}

    def count(self, status: str) -> int:
        """How many types carry a status, for the doc's headline numbers."""
        return sum(1 for t in self.types if t.status == status)

    def area_count(self, status: str) -> int:
        """How many scenario sections carry a status."""
        return sum(1 for a in self.areas if a.status == status)


def _clean(text: str) -> str:
    """One line of prose, for tables and bullets: the ledger wraps, the doc must not."""
    return " ".join(text.split())


def load_coverage(path: str | Path = COVERAGE) -> Coverage:
    """Read the ledger, refusing unknown keys and unclassified statuses.

    Strict here so a typo fails with a name rather than as a silently missing
    row in the completeness test - a coverage file that quietly drops a type is
    exactly the failure this ledger exists to prevent.
    """
    data: dict[str, Any] = tomllib.loads(Path(path).read_text(encoding="utf-8"))

    types: list[TypeEntry] = []
    for raw in data.get("type", []):
        unknown = set(raw) - _TYPE_KEYS
        if unknown:
            raise ValueError(
                f"coverage: type {raw.get('name')!r} has unknown key(s) {sorted(unknown)}"
            )
        entry = TypeEntry(
            name=str(raw["name"]),
            kind=str(raw["kind"]),
            status=str(raw["status"]),
            bskforge=str(raw.get("bskforge", "")),
            wire=str(raw.get("wire", "")),
            command_path=str(raw.get("command_path", "")),
            first_hop=str(raw.get("first_hop", "")),
            scenarios=tuple(str(s) for s in raw.get("scenarios", ())),
            phase=str(raw.get("phase", "")),
            why=str(raw.get("why", "")),
        )
        if entry.kind not in KINDS:
            raise ValueError(f"coverage: {entry.name}: kind must be one of {sorted(KINDS)}")
        if entry.status not in STATUSES:
            raise ValueError(
                f"coverage: {entry.name}: unknown status {entry.status!r} (use {sorted(STATUSES)})"
            )
        if entry.first_hop not in FIRST_HOPS:
            raise ValueError(f"coverage: {entry.name}: unknown first_hop {entry.first_hop!r}")
        types.append(entry)

    areas: list[AreaEntry] = []
    for raw in data.get("area", []):
        unknown = set(raw) - _AREA_KEYS
        if unknown:
            raise ValueError(
                f"coverage: area {raw.get('name')!r} has unknown key(s) {sorted(unknown)}"
            )
        area = AreaEntry(
            name=str(raw["name"]), status=str(raw["status"]), why=str(raw.get("why", ""))
        )
        if area.status not in AREA_STATUSES:
            raise ValueError(f"coverage: area {area.name}: unknown status {area.status!r}")
        areas.append(area)

    if not types or not areas:
        raise ValueError("coverage: the ledger is empty - that is not a classification")
    return Coverage(types=tuple(types), areas=tuple(areas))


# --- the upstream unions (what bskforge actually exposes) -------------------


def schema_sensor_types() -> set[str]:
    """Every first-class sensor type in bskforge's schema union."""
    from bskforge.schema.sensors import SensorConfig

    return _discriminator_names(SensorConfig)


def schema_effector_types() -> set[str]:
    """Every first-class effector type (actuators and disturbance models)."""
    from bskforge.schema.effectors import EffectorConfig

    return _discriminator_names(EffectorConfig)


def schema_disturbance_types() -> set[str]:
    """Every environmental disturbance model bskforge can build.

    A separate union upstream (``[[disturbance]]`` beside ``[[effector]]``), kept
    separate here for the same reason: a disturbance has no command path, so
    lumping it in with actuators would ask the ledger to lie about who drives it.
    """
    from bskforge.schema.effectors import DisturbanceConfig

    return _discriminator_names(DisturbanceConfig)


def schema_area_names() -> set[str]:
    """Every scenario section bskforge's root schema accepts.

    Private attributes are excluded: ``_source_path`` is loader bookkeeping, not
    a capability the project can decline.
    """
    from bskforge.schema.root import ScenarioConfig

    return {name for name in ScenarioConfig.model_fields if not name.startswith("_")}


def _discriminator_names(config: Any) -> set[str]:  # noqa: ANN401 - pydantic model class
    names: set[str] = set()
    for variant in get_args(get_args(config)[0]):
        default = variant.model_fields["type"].default
        if isinstance(default, str):
            names.add(default)
    return names


# --- rendering --------------------------------------------------------------


def owner_label(phase: str) -> str:
    """Who will close this gap, in words a reader can act on.

    A code with no label is an error, not a fallback: the ledger is data and the
    page is prose, and the translation is the whole reason the page is generated.
    """
    if not phase:
        return ""
    try:
        return OWNER_LABELS[phase]
    except KeyError:
        raise KeyError(
            f"coverage: owner {phase!r} has no reader-facing label "
            "- add one to sim/coverage.py OWNER_LABELS"
        ) from None


def _cell(text: str) -> str:
    """Keep a table cell on one line and harmless."""
    return _clean(text).replace("|", "\\|") or "—"


def render_doc(text: str, cov: Coverage) -> str:
    """Replace the GENERATED region of ``docs/coverage.md`` with the ledger.

    Callers compare the result against the file they read: an edit inside the
    region that does not come from this function fails the gate, which is the
    only way the summary numbers can be trusted.
    """
    if BEGIN not in text or END not in text:
        raise ValueError("coverage: the doc has no GENERATED region - restore the markers")

    total = len(cov.types)
    landed, barrier = cov.count("landed"), cov.count("barrier")
    areas_used = cov.area_count("used")
    lines = [
        BEGIN,
        "",
        f"**{landed + barrier} of {total} part types are supported on the bus** "
        f"({landed} landed, {barrier} as the barrier tick); "
        f"**{areas_used} of {len(cov.areas)} scenario sections are used**. "
        "Everything upstream builds; every gap below is ours. The counts are "
        "computed from `registry/coverage.toml`, and a hand-edit inside this "
        "region fails the coverage gate.",
        "",
        "## Sensors",
        "",
        "| type | status | on the wire | first missing hop | owner |",
        "| --- | --- | --- | --- | --- |",
    ]
    lines += _type_rows(cov.sensors, wire_column=True)
    lines += [
        "",
        "## Effectors",
        "",
        "An effector's *telemetry* is knowledge and rides a row; its *actuation* is",
        'a command path and never earns a MsgId (docs/adding_a_sensor.md, "Start',
        'here: which kind of thing are you adding?").',
        "",
        "| type | status | command path | first missing hop | owner |",
        "| --- | --- | --- | --- | --- |",
    ]
    lines += _type_rows(cov.effectors, wire_column=False)
    lines += [
        "",
        "## Disturbance models",
        "",
        "Not commanded, not sensed: these change the world the plant lives in,",
        "which is exactly why they move every pinned trajectory.",
        "",
        "| type | status | how it enters the world | first missing hop | owner |",
        "| --- | --- | --- | --- | --- |",
    ]
    lines += _type_rows(cov.disturbances, wire_column=False)
    lines += [
        "",
        "## Scenario sections",
        "",
        "| section | status | why |",
        "| --- | --- | --- |",
    ]
    lines += [f"| `{a.name}` | {a.status} | {_cell(a.why)} |" for a in cov.areas]

    lines += ["", "## The reasons, in full", ""]
    for entry in (*cov.sensors, *cov.effectors, *cov.disturbances):
        owner = f" · owner: **{owner_label(entry.phase)}**" if entry.phase else ""
        lines.append(f"- **`{entry.name}`** — {entry.status}{owner}. {_clean(entry.why)}")
    lines += [
        "",
        "## What the statuses mean",
        "",
        "| status | meaning |",
        "| --- | --- |",
        "| `landed` | a landed row in `registry/parts.toml`, and the circuit flies |",
        "| `barrier` | crosses as the sim's own tick; deliberately never a row |",
        "| `catalog-unlanded` | bskforge builds it, we have no row and no wire yet |",
        "| `command-path` | it actuates, so it lands through `fcs`/`sps`, not a MsgId |",
        "| `refused` | a decision with a reason, not a gap |",
        "| `banned` (sections) | refused at load, by profile rather than by omission |",
        "",
        "## Reading this file",
        "",
        "- **Completeness is enforced.** A new sensor, effector or disturbance type",
        "  upstream, or a new scenario section, fails our build until it appears here.",
        "  The ledger is checked against bskforge's live schema unions, never a copy.",
        "- **`landed` is bidirectional.** A row that lands without a classification, and",
        "  a classification whose row has gone, both fail the offline gates.",
        "- **`banned` has teeth.** Every fleet here is read through bskforge's audited",
        "  profile (`sim/fleet_policy.py` asks for it), so a file that wires hardware in",
        "  by hand is refused at load, before validation; the policy tests prove the",
        "  refusal fires, that no unaudited door exists, and that our fleets pass.",
        "- **Every gap costs something specific.** Each unlanded part names its first",
        '  missing hop from a fixed vocabulary and carries a reason; "eventually" is',
        "  not a classification, and the offline gates say so.",
        "- **Upstream is not the constraint.** bskforge ships a builder for every one of",
        f'  the {total} types listed here, so not one gap above is "Basilisk cannot".',
        "  That is why this ledger lives beside the parts registry: the parts exist,",
        "  and what remains is what this project has decided not to wire yet.",
        END,
    ]

    head, rest = text.split(BEGIN, 1)
    _, tail = rest.split(END, 1)
    return head + "\n".join(lines) + tail


def _type_rows(entries: list[TypeEntry], *, wire_column: bool) -> list[str]:
    rows = []
    for entry in entries:
        middle = entry.wire if wire_column else entry.command_path
        rows.append(
            f"| `{entry.name}` | {entry.status} | {_cell(middle)} | "
            f"{f'`{entry.first_hop}`' if entry.first_hop else '—'} | "
            f"{owner_label(entry.phase) or '—'} |"
        )
    return rows


# --- command line -----------------------------------------------------------


def main(argv: list[str] | None = None) -> int:
    """`--check` says whether the page matches the ledger; `--write` refreshes it.

    A generated page whose generator has no command is a page that gets edited by
    hand the first time it is stale. `tests/test_coverage.py` pins the page to this
    renderer either way, so the gate stays honest about which half is authoritative.
    """
    parser = argparse.ArgumentParser(
        description="render registry/coverage.toml into docs/coverage.md"
    )
    parser.add_argument("--write", action="store_true", help="regenerate the page")
    parser.add_argument(
        "--check",
        action="store_true",
        help="report whether the page matches the ledger; exit 1 if it does not (the default)",
    )
    args = parser.parse_args(argv)

    cov = load_coverage()
    current = DOC.read_text(encoding="utf-8")
    rendered = render_doc(current, cov)
    if args.write:
        if rendered == current:
            print("coverage: docs/coverage.md already current")
            return 0
        DOC.write_text(rendered, encoding="utf-8")
        print("coverage: docs/coverage.md regenerated from registry/coverage.toml")
        return 0
    if rendered != current:
        print("coverage: docs/coverage.md is stale - run `uv run python -m sim.coverage --write`")
        return 1
    print(f"coverage: docs/coverage.md matches the ledger ({len(cov.types)} part types)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
