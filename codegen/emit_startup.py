"""emit_startup: the scenario decides which apps the core wakes.

One ES startup table per vehicle, derived - never hand-written:

    registry/apps.toml         the inventory: every app, its boot fields,
                               and whether it is plumbing (baseline) or
                               payload authority (declared)
    registry/parts.toml        a part that brings an app says so (fsw_apps)
    the scenario file          what the vehicle actually mounts

The grammar law (measured, from the pinned core's ES parser): records are
eight comma-separated fields terminated by ';', and '!' ends the file -
there are NO comments. So the generated table is pure records, and this
module's docstrings are where the prose lives. A wake list must also be
*coherent*: an app's entailments (``requires``) have to wake on the same
bus, or the door refuses with a named sentence - "you asked for the
manifest but no file-transfer engine" is caught here, not at request time.

The build-time default table (``generate_startup.cmake``, the fallback for a
hand-run compose yard) is pinned equal to this generator's all-parts union
by tests/test_startup_table.py: two sources of one grammar, checked against
each other, like every other surface here.

    uv run python -m codegen.emit_startup                       # the union
    uv run python -m codegen.emit_startup --scenario s.toml --vehicle chief
    uv run python -m codegen.emit_startup --scenario s.toml \
        --write cfs_mission/mission/build/exe/cpu1/cf   # the yard door's step
"""

from __future__ import annotations

import argparse
import sys
import tomllib
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from sim.fleet import scenario_vehicles
from sim.parts import REPO, Part, load_registry

APPS_REGISTRY = REPO / "registry" / "apps.toml"

# characters the ES startup grammar reserves: field, record, and file end.
# A field containing one would silently rewrite the table's meaning.
_ES_RESERVED = (",", ";", "!")
_ES_CLASSES = ("CFE_APP", "CFE_TASK", "CFE_LIB", "CFE_DRIVER")


class StartupDoorError(ValueError):
    """The wake list cannot be built; the sentence is the product."""


@dataclass(frozen=True)
class AppRow:
    """One inventory row: an app the core may wake, with its boot fields."""

    name: str
    class_name: str
    entry: str
    short: str
    priority: int
    stack: int
    flags: str
    load: int
    baseline: bool
    requires: tuple[str, ...] = ()
    # EDS packages whose ground definitions ride this app's wake-up (the
    # ground-scope seam, codegen/eds2cosmos --scope-scenario).
    eds_packages: tuple[str, ...] = ()
    # Core-shipped apps are built by the core or the per-CPU lists, never by
    # the scenario's build fragment (codegen/emit_applist): they answer for
    # themselves on every image.
    core_shipped: bool = False


def _checked_field(name: str, field_name: str, value: str) -> str:
    value = value.strip()
    if not value:
        raise StartupDoorError(f"app {name!r}: empty {field_name}")
    for bad in _ES_RESERVED:
        if bad in value:
            raise StartupDoorError(f"app {name!r}: {field_name} {value!r} carries reserved {bad!r}")
    return value


def load_apps(path: Path = APPS_REGISTRY, parts: list[Part] | None = None) -> list[AppRow]:
    """Every inventory row, in file order (the file order IS the boot order).

    ``parts`` is the parts registry for the cross-checks; the real one loads
    by default, and tests may hand in a synthetic one.
    """
    with path.open("rb") as fh:
        doc = tomllib.load(fh)
    rows = [
        AppRow(
            name=_checked_field(str(row.get("name")), "name", str(row["name"])),
            class_name=str(row["class"]),
            entry=_checked_field(str(row.get("name")), "entry", str(row["entry"])),
            short=_checked_field(str(row.get("name")), "short", str(row["short"])),
            priority=int(row["priority"]),
            stack=int(row["stack"]),
            flags=str(row.get("flags", "0x0")),
            load=int(row.get("load", 0)),
            baseline=str(row["when"]) == "baseline",
            requires=tuple(row.get("requires", ())),
            eds_packages=tuple(row.get("eds_packages", ())),
            core_shipped=str(row.get("shipped_with", "mission")) == "core",
        )
        for row in doc.get("app", [])
    ]
    if not rows:
        raise StartupDoorError(f"{path}: no [[app]] rows - an empty inventory wakes nothing")
    for row in rows:
        if row.class_name not in _ES_CLASSES:
            raise StartupDoorError(
                f"app {row.name!r}: class {row.class_name!r} is not one of {_ES_CLASSES}"
            )
    names = [row.name for row in rows]
    if len(names) != len(set(names)):
        raise StartupDoorError(f"{path}: duplicate app rows: {names}")
    owned: dict[str, str] = {}
    for row in rows:
        for pkg in row.eds_packages:
            if pkg in owned:
                raise StartupDoorError(
                    f"{path}: EDS package {pkg!r} owned by both {owned[pkg]!r} and {row.name!r}"
                )
            owned[pkg] = row.name
    by_name = {row.name: row for row in rows}
    for row in rows:
        for want in row.requires:
            if want not in by_name:
                raise StartupDoorError(f"app {row.name!r} requires {want!r}: not in the inventory")
    declared = [row.name for row in rows if not row.baseline]
    brought = {
        app for part in (parts if parts is not None else load_registry()) for app in part.fsw_apps
    }
    unclaimed = sorted(set(declared) - brought)
    if unclaimed:
        raise StartupDoorError(
            f"{path}: declared apps no part brings onto a vehicle: {unclaimed} "
            "- no scenario could ever wake them (registry/parts.toml fsw_apps)"
        )
    unknown = sorted(brought - set(names))
    if unknown:
        raise StartupDoorError(f"{path}: parts bring apps the inventory lacks: {unknown}")
    return rows


def _vehicle_mounted_types(doc: dict[str, Any], vehicle: str) -> set[str]:
    """Every bskforge type this vehicle's scenario sections mount."""
    mounted: set[str] = set()
    for craft in doc.get("spacecraft", []):
        if str(craft.get("name")) != vehicle:
            continue
        for section in ("sensors", "effectors"):
            for device in craft.get(section, []):
                if "type" in device:
                    mounted.add(str(device["type"]))
    for camera in doc.get("camera", []):
        if str(camera.get("parent")) == vehicle:
            mounted.add("camera")
    return mounted


def declared_apps(parts: list[Part], mounted: set[str]) -> set[str]:
    """The payload authorities a mounting vehicle wakes, via its rows."""
    return {app for part in parts for app in part.fsw_apps if set(part.bskforge_types) & mounted}


def select_apps(
    apps: list[AppRow], parts: list[Part], scenario: Path | None, vehicle: str | None
) -> list[AppRow]:
    """The coherent wake list, in boot order. ``scenario=None`` is the union."""
    if scenario is None:
        selected = list(apps)
    else:
        if vehicle is None:
            raise StartupDoorError("a per-vehicle table needs --vehicle (or write all)")
        with scenario.open("rb") as fh:
            doc = tomllib.load(fh)
        mounted = _vehicle_mounted_types(doc, vehicle)
        if not any(craft.get("name") == vehicle for craft in doc.get("spacecraft", [])):
            raise StartupDoorError(f"{scenario.name}: no [[spacecraft]] named {vehicle!r}")
        woken = declared_apps(parts, mounted)
        selected = [row for row in apps if row.baseline or row.name in woken]
    names = {row.name for row in selected}
    for row in selected:
        missing = [want for want in row.requires if want not in names]
        if missing:
            raise StartupDoorError(
                f"{vehicle or 'union'}: app {row.name!r} requires {missing} - "
                "the scenario asks for a service its wake list cannot serve"
            )
    return selected


def render_rows(selected: list[AppRow]) -> list[str]:
    """ES records: eight fields, ';' ends each; whitespace is the parser's to skip."""
    return [
        f"{row.class_name}, {row.name}, {row.entry}, {row.short}, "
        f"{row.priority}, {row.stack}, {row.flags}, {row.load};"
        for row in selected
    ]


def startup_table(
    apps: list[AppRow], parts: list[Part], scenario: Path | None, vehicle: str | None
) -> str:
    """The whole wake-up record file: coherent, in boot order, comment-free."""
    return "\n".join(render_rows(select_apps(apps, parts, scenario, vehicle))) + "\n"


def table_path(out_dir: Path, vehicle: str) -> Path:
    """Where one vehicle's landed table lives, beside the core."""
    return out_dir / f"cfe_es_startup.{vehicle}.scr"


def write_yard(out_dir: Path, scenario: Path) -> list[Path]:
    """Land every vehicle's table, sweeping stale doors first.

    The sweep matters as much as the write: a leftover table from a dead
    scenario would wake the next same-named vehicle on the last world's
    authority list - the one failure mode a deploy-time artifact invites.
    """
    out_dir.mkdir(parents=True, exist_ok=True)
    for stale in out_dir.glob("cfe_es_startup.*.scr"):
        stale.unlink()
    apps, parts = load_apps(), load_registry()
    written = []
    for craft in scenario_vehicles(scenario):
        path = table_path(out_dir, craft.name)
        path.write_text(startup_table(apps, parts, scenario, craft.name), encoding="utf-8")
        written.append(path)
    return written


def check_yard(out_dir: Path, scenario: Path) -> list[str]:
    """Recompute every landed table; return the sentences that disagree."""
    apps, parts = load_apps(), load_registry()
    problems = []
    for craft in scenario_vehicles(scenario):
        path = table_path(out_dir, craft.name)
        want = startup_table(apps, parts, scenario, craft.name)
        if not path.exists():
            problems.append(f"{path.name}: not landed (boot would fall back to the image default)")
        elif path.read_text(encoding="utf-8") != want:
            problems.append(f"{path.name}: stale - it does not match the scenario's declaration")
    extra = sorted(
        p
        for p in out_dir.glob("cfe_es_startup.*.scr")
        if p.stem.removeprefix("cfe_es_startup.")
        not in {c.name for c in scenario_vehicles(scenario)}
    )
    problems.extend(f"{p.name}: table for a vehicle this scenario does not fly" for p in extra)
    return problems


def main(argv: list[str] | None = None) -> int:
    """The door's command line: print one table, land a fleet's, or check them."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--scenario", type=Path, help="scenario file; omit for the all-parts union")
    parser.add_argument("--vehicle", help="one vehicle's table (needs --scenario)")
    parser.add_argument(
        "--write", type=Path, metavar="DIR", help="land per-vehicle tables (the yard door's step)"
    )
    parser.add_argument(
        "--check", type=Path, metavar="DIR", help="recompute and compare the landed tables"
    )
    args = parser.parse_args(argv)

    try:
        if args.write:
            if not args.scenario:
                raise StartupDoorError("--write needs --scenario: a union has no vehicles")
            for path in write_yard(args.write, args.scenario):
                print(f"landed {path}")
            return 0
        if args.check:
            if not args.scenario:
                raise StartupDoorError("--check needs --scenario")
            problems = check_yard(args.check, args.scenario)
            for line in problems:
                print(f"emit_startup: {line}", file=sys.stderr)
            if not problems:
                print("startup tables match the scenario")
            return 1 if problems else 0
        sys.stdout.write(startup_table(load_apps(), load_registry(), args.scenario, args.vehicle))
        return 0
    except StartupDoorError as exc:
        print(f"emit_startup: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
