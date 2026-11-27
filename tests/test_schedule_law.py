"""The craft's timer ticks only for organs every vehicle grew.

The schedule table (orrery_defs/tables/sch_lab_table.c) is compiled into
every image and fires on its own, onboard - so it can only ever speak to
apps the scenario cannot refuse: the core and the baseline organs. A
payload app's housekeeping (the eye's CAM_HK) is therefore ground-polled
through the scoped dictionary (codegen/eds2cosmos --scope-scenario),
never timer-driven. This is the law that lets a per-scenario flight
image leave a declared app out without stranding a timer that talks to
it (docs/what_the_scenario_rules.md).

Two halves. The ledger: every message the committed table schedules, with
the app it belongs to, each app checked against the inventory - an edit
that schedules a declared app's message refuses offline. The wire truth
(where the mission build exists): every scheduled MsgId in the built
.tbl resolves through the EDS to an owner, and the owners match the
ledger exactly - the ledger cannot rot into a polite fiction.
"""

from __future__ import annotations

import functools
import re
import struct
from collections import Counter

import pytest

from codegen.eds2cosmos import build_eds, mission_packets
from codegen.emit_startup import load_apps
from sim.parts import REPO

TABLE_SRC = REPO / "cfs_mission" / "mission" / "orrery_defs" / "tables" / "sch_lab_table.c"
TABLE_BIN = REPO / "cfs_mission" / "mission" / "build" / "exe" / "cpu1" / "cf" / "sch_lab_table.tbl"
CFS = REPO / "cfs_mission" / "cFS"
MISSION_DEFS = REPO / "cfs_mission" / "mission" / "orrery_defs" / "eds"

CORE = "core"

# The committed schedule, stem by stem: the topicid name (the macro minus
# _MID) and the app whose organ it pokes. The eye is absent on purpose -
# its housekeeping answers the ground, not a timer. The 2026-11-27 prebaked
# purge left with its table rows: the timer now speaks only to the core and
# the four baseline organs that answer it (registry/apps.toml).
LEDGER = {
    "CFE_ES_SEND_HK": CORE,
    "CFE_EVS_SEND_HK": CORE,
    "CFE_TIME_SEND_HK": CORE,
    "CFE_SB_SEND_HK": CORE,
    "CFE_TBL_SEND_HK": CORE,
    "CF_SEND_HK": "cf",
    "CF_WAKE_UP": "cf",
    "FCS_SEND_HK": "fcs",
    "OBC_HK_SEND_HK": "obc_hk",
    "SPS_SEND_HK": "sps",
}

ENTRY = re.compile(r"CFE_SB_MSGID_WRAP_VALUE\((\w+)_MID\)")
GUARD = re.compile(r"#ifdef\s+(\w+)")


def scheduled_stems() -> set[str]:
    """Every scheduled topicid stem in the table source, guards respected.

    Entries inside a #ifdef are disabled in this mission, and since the
    2026-11-27 purge the table carries NO guards at all: SBN was excluded
    entirely before that, and the last guarded rows left with the retired
    apps. A new guard is therefore a new decision - refused here rather
    than parsed into silence.
    """
    stems: set[str] = set()
    guards: set[str] = set()
    depth = 0
    for line in TABLE_SRC.read_text(encoding="utf-8").splitlines():
        if line.lstrip().startswith("#ifdef"):
            guards.update(m.group(1) for m in GUARD.finditer(line))
            depth += 1
        elif line.lstrip().startswith("#endif"):
            depth -= 1
        elif depth == 0:
            stems.update(m.group(1) for m in ENTRY.finditer(line))
    assert not guards, f"schedule guards returned: {sorted(guards)}"
    return stems


@functools.lru_cache(maxsize=1)
def scheduled_msgids() -> list[int]:
    """The nonzero MsgIds of the built image, in slot order.

    The .tbl layout, measured against the core's own loader: big-endian
    cFE header (64 B) then big-endian table header (12 B + 40 B name),
    data from byte 116 in NATIVE order: TickRate u32 + 32 entries of
    stride 76 (MsgId u32 first). No CRC exists in the file.
    """
    raw = TABLE_BIN.read_bytes()
    assert struct.unpack_from(">I", raw, 0)[0] == 0x63464531  # 'cFE1'
    assert raw[76:93].split(b"\0")[0] == b"SCH_LAB.Schedule"
    data = raw[116:]
    return [mid for i in range(32) if (mid := struct.unpack_from("<I", data, 4 + i * 76)[0]) != 0]


@functools.lru_cache(maxsize=1)
def owners_on_wire() -> Counter[str]:
    """Each scheduled MsgId's owning app, resolved through the EDS."""
    byid = {p.msgid: p for p in mission_packets(build_eds(CFS, MISSION_DEFS).packets())}
    owner = {pkg: row.name for row in load_apps() for pkg in row.eds_packages}
    seen: Counter[str] = Counter()
    for mid in scheduled_msgids():
        pkg = byid[mid].target_app
        seen[CORE if pkg.startswith("CFE_") else owner[pkg]] += 1
    return seen


# --- the offline door: the committed table, ledger by ledger --------------------


def test_the_committed_schedule_is_exactly_the_ledger() -> None:
    """Any timer the table gains or loses must be re-declared here first."""
    stems, ledger = scheduled_stems(), set(LEDGER)
    assert stems == ledger, (
        f"schedule moved: gained {sorted(stems - ledger)}, lost {sorted(ledger - stems)}"
    )


def test_every_timer_target_is_an_organ_every_vehicle_grows() -> None:
    """The law in one assertion: scheduled means baseline (or the core).

    The day someone schedules a declared app's message - the eye included
    - this refuses, because a vehicle whose scenario never mounted that
    organ would still hear a timer for it every tick, and a per-scenario
    image without the app would schedule a command to nothing.
    """
    rows = {row.name: row for row in load_apps()}
    for stem, app in LEDGER.items():
        if app == CORE:
            continue
        assert app in rows, f"{stem}: scheduled app {app!r} is not in the inventory"
        assert rows[app].baseline, f"{stem}: the timer may not speak to declared {app!r}"


def test_the_eye_never_ticks_from_the_timer() -> None:
    """Named consequence: CAM_HK answers the station - or stays silent.

    The startup table lets cam wake, the scoped dictionary lets the
    station ask, and silence undeclared stays silence: no onboard path
    fires the eye's housekeeping without the ground asking for it.
    """
    assert "cam" not in LEDGER.values()
    assert not [stem for stem in scheduled_stems() if stem.startswith("CAM_")]


# --- the wire truth: the built image, where one exists --------------------------


@pytest.mark.skipif(not TABLE_BIN.exists(), reason="no mission build on this machine")
def test_the_built_table_schedules_exactly_the_ledger() -> None:
    """The binary obeys the ledger: same count, same owners, slot for slot.

    MsgIds (not stems) resolve here - the macro names are topicid symbols,
    not packet names (OBC_HK_SEND_HK_MID schedules OBC_SEND_HK) - so the
    ledger's honesty is checked where the names do agree: on ownership.
    """
    assert len(scheduled_msgids()) == len(LEDGER)
    assert owners_on_wire() == Counter(a for a in LEDGER.values() if a != CORE) + Counter(
        {CORE: sum(1 for a in LEDGER.values() if a == CORE)}
    )
