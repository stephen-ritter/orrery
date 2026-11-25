"""eds2cosmos: generate COSMOS cmd/tlm definitions and the gs_tcp MsgId table.

Input  : the EDS XML of the pinned cFS checkout (the EDS is the single
         source of truth) plus the mission define files (topic IDs).
Output : OpenC3 ``.txt`` command/telemetry definitions for the SAT_CHIEF target
         and a C header (MsgId subscription table) for the generic ``gs_tcp``
         gateway app. Regenerate after any cFS bump; never hand-edit output.

usage:
    uv run python -m codegen.eds2cosmos \
        --cfs cfs_mission/cFS \
        --mission-defs cfs_mission/mission/orrery_defs/eds \
        --out-cosmos cosmos_plugin/cmd_tlm \
        --out-c cfs_mission/mission/apps/gs_tcp/fsw/src/rs_mid_table.h
"""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
from collections.abc import Iterable
from dataclasses import asdict, is_dataclass
from pathlib import Path
from typing import Any

from codegen.eds_model import Eds, Item, Packet, Padding, load_eds

TARGET = "SAT_CHIEF"


def content_digest(nodes: Iterable[Any]) -> str:
    """A fingerprint of WHAT a generated file was built from, not when.

    This replaced a wall-clock stamp, and the clock cost more than it was worth:
    regenerating unchanged defs rewrote six tracked files whose only difference
    was the second they were written, so "did the wire schema move?" could not be
    read off a diff, and a test comparing two emissions failed whenever the two
    calls happened to straddle a second boundary. A digest of the schema is
    byte-stable and strictly more informative: an unchanged digest means the
    inputs did not move, and a changed one means a decoder somewhere needs to be
    rebuilt - which is the sentence the header exists to support.

    Pinned by ``tests/test_codegen_idempotence.py``, which demands that
    regeneration reproduce the committed bytes exactly.
    """
    digest = hashlib.sha256()
    for node in nodes:
        payload = asdict(node) if is_dataclass(node) else repr(node)
        digest.update(json.dumps(payload, sort_keys=True, default=str).encode())
        digest.update(b"\0")
    return digest.hexdigest()[:8]


def _cosmos_name(path: str) -> str:
    """Last path segment, upper snake case (C/HK.HkTlm_Payload.X -> X)."""
    leaf = path.rsplit(".", 1)[-1]
    out = []
    for i, ch in enumerate(leaf):
        if ch.isupper() and i and not leaf[i - 1].isupper() and leaf[i - 1] != "_":
            out.append("_")
        out.append(ch.upper())
    name = "".join(out)
    while "__" in name:  # COSMOS forbids double underscores
        name = name.replace("__", "_")
    return name.rstrip("_")


def _emit_item(item: Item, seen: set[str], is_cmd: bool) -> list[str]:
    base = _cosmos_name(item.name)
    name = base
    suffix = 2
    while name in seen:
        name = f"{base}_{suffix}"
        suffix += 1
    seen.add(name)
    desc = "alignment padding" if isinstance(item, Padding) else "auto-generated from EDS"
    # v6 arity: ITEM takes [desc][endian]; PARAMETER (numeric) takes
    # min max default [desc][endian] -- nil selects the defaults.
    if isinstance(item, Padding):
        kind, cbase = "UINT", "uint"
    else:
        kind = {
            "uint": "UINT",
            "int": "INT",
            "float": "FLOAT",
            "string": "STRING",
            "binary": "BLOCK",
        }[item.cbase]
        cbase = item.cbase
    # non-header items are C struct fields: target-native (little) order;
    # padding/string/binary have no meaningful endianness (and sub-byte LE
    # is invalid)
    le = (
        not isinstance(item, Padding)
        and cbase not in ("binary", "string")
        and not getattr(item, "big_endian", False)
    )
    if is_cmd:
        kw = "APPEND_PARAMETER"
        if kind in ("UINT", "INT", "FLOAT"):
            # v6 requires explicit min/max/default for numeric parameters
            if kind == "UINT":
                lo, hi = 0, (1 << item.bits) - 1
            elif kind == "INT":
                lo, hi = -(1 << (item.bits - 1)), (1 << (item.bits - 1)) - 1
            else:
                lo, hi = (
                    "-3.4e38" if item.bits <= 32 else "-1.8e308",
                    "3.4e38" if item.bits <= 32 else "1.8e308",
                )
            states = getattr(item, "states", {})
            dflt = 0 if not states or 0 in states else min(states)
            tail = f'{kind} {lo} {hi} {dflt} "{desc}"'
        else:
            tail = f'{kind} "{desc}"'
    else:
        kw = "APPEND_ITEM"
        tail = f'{kind} "{desc}"'
    if le:
        tail += " LITTLE_ENDIAN"
    lines = [f"  {kw} {name} {item.bits} {tail}"]
    for value, state in sorted(getattr(item, "states", {}).items()):
        lines.append(f'      STATE "{state}" {value}')
    return lines


def _emit_packet_lines(pkt: Packet, target: str = TARGET) -> list[str]:
    """OpenC3 def lines straight off the model item chain.

    The flattened EDS items already include the full CCSDS primary and cFS
    secondary headers, so no layout is hardcoded here; the first 16 bits
    (VersionId/SecHdrFlags/AppId) become the identifying PKTID field and
    FunctionCode becomes the dispatch ID parameter for commands.
    """
    is_cmd = pkt.direction == "cmd"
    if len(pkt.items) < 3 or sum(i.bits for i in pkt.items[:3]) != 16:
        raise ValueError(f"{pkt.name}: unexpected CCSDS primary layout")
    mid_note = f"CCSDS packet ID (=MsgId 0x{pkt.msgid:04X})"
    lines = [
        f'{"COMMAND" if is_cmd else "TELEMETRY"} {target} {pkt.name} BIG_ENDIAN "{pkt.description}"'
    ]
    # v6: TELEMETRY uses ID_ITEM, COMMAND uses ID_PARAMETER; both pin the
    # packet id to the exact MsgId so stream classification is unambiguous.
    if is_cmd:
        lines.append(f'  APPEND_ID_PARAMETER PKTID 16 UINT 0 65535 {pkt.msgid} "{mid_note}"')
    else:
        lines.append(f'  APPEND_ID_ITEM PKTID 16 UINT {pkt.msgid} "{mid_note}"')
    seen = {"PKTID"}
    for item in pkt.items[3:]:
        leaf_name = item.name.rsplit(".", 1)[-1]
        if is_cmd and leaf_name == "FunctionCode":
            lines.append('  APPEND_ID_PARAMETER FUNCCODE 8 UINT 0 255 0 "command function code"')
            for name, code in sorted(pkt.function_codes.items(), key=lambda kv: kv[1]):
                lines.append(f'    STATE "{name}" {code}')
            seen.add("FUNCCODE")
            continue
        emitted = _emit_item(item, seen, is_cmd)
        if is_cmd and leaf_name == "SeqFlag":
            emitted.insert(1, "      DEFAULT_VALUE 1")  # unsegmented
        lines.extend(emitted)
    return lines


def emit_cosmos(packets: list[Packet], target: str = TARGET) -> tuple[str, str]:
    """Return (cmd_defs_text, tlm_defs_text); target is the COSMOS target name.

    Every vehicle's defs are generated the same way (heterogeneity is an
    input, not a fork): only the target column and file names move.
    """
    header = (
        f"# AUTO-GENERATED by codegen/eds2cosmos from cFS EDS "
        f"content {content_digest(packets)} - DO NOT EDIT\n"
    )
    cmd_chunks, tlm_chunks = [header], [header]
    for pkt in sorted(packets, key=lambda p: (p.target_app, p.name)):
        lines = _emit_packet_lines(pkt, target)
        (cmd_chunks if pkt.direction == "cmd" else tlm_chunks).append("\n".join(lines))
    return "\n\n".join(cmd_chunks) + "\n", "\n\n".join(tlm_chunks) + "\n"


# Packets whose shipped cFS EDS XML does NOT describe the runtime
# (CFE_EDS_ENABLED=false) message. Excluded from BOTH the ground defs and the
# beacon subscription: a wrong def misidentifies live traffic, and a
# subscription without a def is an UNKNOWN-packet storm on the ground.
# sim/wire_audit.py is the check that keeps this list honest in both
# directions; drop an entry when the XML (or cFS) is fixed upstream.
MODEL_DRIFT_EXCLUSIONS = {
    "CF_HK_TLM": "CF EDS XML does not match the default (non-EDS) CF_HkPacket "
    "layout: modeled 94 B vs 184 B observed on the wire (QueueIdx/counter drift)",
}

# Packets that are legitimate on the bus but must never reach the ground feed
# by design: the lockstep barrier ack is link protocol - fcs answers
# every processed sensor frame, which is exactly the chatter an operator must
# not see. Excluded from the defs as well, on purpose: if it ever leaked onto
# the beacon, COSMOS stamps it UNKNOWN, so the absence of a def is the
# tripwire, not an oversight. This is drift's mirror image - not "the def is
# wrong" but "there must be no def".
BEACON_EXCLUSIONS = {
    "BSK_LINK_STEP_ACK_TLM": "lockstep barrier ack (fcs -> bsk_link -> sim): bus-internal",
}


def mission_packets(packets: Iterable[Packet]) -> list[Packet]:
    """All packets minus drift exclusions and bus-internal exclusions."""
    drop = MODEL_DRIFT_EXCLUSIONS.keys() | BEACON_EXCLUSIONS.keys()
    return [p for p in packets if p.name not in drop]


def emit_c_table(packets: list[Packet], cfs_sha: str) -> str:
    """C header: the gs_tcp MsgId subscription table (downlink telemetry only).

    Commands are deliberately NOT subscribed: they are consumed by their apps,
    but the ones scheduled onto the bus (sch_lab wake-ups) would otherwise be
    forwarded on the beacon, where the ground can only reject them as UNKNOWN
    packets. Command traffic belongs on the uplink, not on the beacon.
    """
    tlms = [pkt for pkt in packets if pkt.direction == "tlm"]
    lines = [
        "/* AUTO-GENERATED by codegen/eds2cosmos from cFS EDS - DO NOT EDIT. */",
        f"/* cFS rev: {cfs_sha} */",
        "#ifndef RS_MID_TABLE_H",
        "#define RS_MID_TABLE_H",
        "",
        "#include <common_types.h>",
        "",
        "typedef struct {",
        "    uint16     MsgId;",
        "    const char *Name;",
        "} RsMidEntry_t;",
        "",
        "/* Every SB *telemetry* message the ground knows how to interpret;",
        " * gs_tcp subscribes to all of them and streams raw packets on the",
        " * beacon. Command MsgIds are excluded: they are not telemetry. */",
        "static const RsMidEntry_t ORRERY_MID_TABLE[] = {",
    ]
    for pkt in sorted(tlms, key=lambda p: p.msgid):
        lines.append(f'    {{0x{pkt.msgid:04X}, "{pkt.name}"}},')
    lines += [
        "};",
        "",
        f"#define ORRERY_MID_COUNT {len(tlms)}",
        "",
        "#endif /* RS_MID_TABLE_H */",
        "",
    ]
    return "\n".join(lines)


def emit_c_cmd_table(packets: list[Packet], cfs_sha: str) -> str:
    """C header: the sps command dictionary (schedulable MsgId+Fc -> size).

    The mirror of emit_c_table: where gs_tcp subscribes to everything the
    ground may READ, sps validates what the ground may SCHEDULE. Rows come
    from the same EDS model that emits the COSMOS cmd defs, so a command the
    operator can send is by construction a command the spacecraft can judge:
    unknown MsgId/Fc, wrong payload length, and queue overflow are flight
    rejections with counters, never silently executed garbage.

    One row per (MsgId, function code): FC-dispatched packets contribute a
    row per declared subtype; plain and payload-bearing packets carry FC 0
    (the house convention the scheduler and the COSMOS defs already use).
    """
    cmds = [pkt for pkt in packets if pkt.direction == "cmd"]
    rows: dict[tuple[int, int], tuple[str, int]] = {}
    for pkt in cmds:
        payload_bytes = pkt.total_bytes() - 8  # primary(6) + FC + checksum
        # FC-dispatch packets get packet-qualified subtype names (a bare
        # "NoopCmd" is meaningless across 59 packets); plain/payload packets
        # are single-row at FC 0 and carry their own name.
        codes = (
            {f"{pkt.name}.{name}": fc for name, fc in pkt.function_codes.items()}
            if pkt.function_codes
            else {pkt.name: 0}
        )
        for name, fc in codes.items():
            key = (pkt.msgid, fc)
            if key in rows:
                raise ValueError(f"duplicate schedulable row {key}: {rows[key]} vs {pkt.name}")
            rows[key] = (name, payload_bytes)
    lines = [
        "/* AUTO-GENERATED by codegen/eds2cosmos from cFS EDS - DO NOT EDIT. */",
        f"/* cFS rev: {cfs_sha} */",
        "#ifndef SPS_CMD_TABLE_H",
        "#define SPS_CMD_TABLE_H",
        "",
        "#include <common_types.h>",
        "",
        "typedef struct {",
        "    uint16     MsgId;",
        "    uint8      Fc;",
        "    uint16     PayloadBytes;",
        "    const char *Name;",
        "} SpsCmdEntry_t;",
        "",
        "/* Every SB *command* the ground can send, one row per MsgId+Fc:",
        " * the schedule validator's whole world. Kept honest by the same",
        " * tests that pin the COSMOS cmd defs against the EDS. */",
        "static const SpsCmdEntry_t SPS_CMD_TABLE[] = {",
    ]
    for (msgid, fc), (name, payload_bytes) in sorted(rows.items()):
        lines.append(f'    {{0x{msgid:04X}, {fc}, {payload_bytes}, "{name}"}},')
    lines += [
        "};",
        "",
        "#define SPS_CMD_COUNT (sizeof(SPS_CMD_TABLE) / sizeof(SPS_CMD_TABLE[0]))",
        "",
        "#endif /* SPS_CMD_TABLE_H */",
        "",
    ]
    return "\n".join(lines)


def _cfs_sha(cfs: Path) -> str:
    return subprocess.run(
        ["git", "-C", str(cfs), "rev-parse", "--short", "HEAD"],
        capture_output=True,
        text=True,
        check=True,
    ).stdout.strip()


def build_eds(cfs: Path, mission_defs: Path) -> Eds:
    """Load the checkout EDS plus mission define XMLs."""
    if not cfs.is_dir():
        raise SystemExit(
            f"cFS core absent at {cfs}: an empty EDS tree is not an empty world - "
            "fetch the pinned core first (scripts/fetch_cfs.sh)"
        )
    mission_files = sorted(mission_defs.glob("*.xml"))
    if not mission_files:
        raise SystemExit(f"no mission define XMLs under {mission_defs}")
    return load_eds(cfs, mission_files, extra_files=[])


# ---------------------------------------------------------------------------
# The truth stream: BSK_LINK/TruthState is a bare EDS container - no TopicId
# by design (the tripwire test guarantees no TRUTH packet can exist in cFS),
# so it never appears in packets(). The truth target's one def is generated
# from it directly, behind a tiny GROUND-owned header. The no-forging rule
# guards the FLIGHT hop; this frame never crosses the satellite boundary, so
# the ground may define its own protocol - and it must, because COSMOS needs
# an id item to identify a headerless stream.
TRUTH_CONTAINER = "BSK_LINK/TruthState"
TRUTH_PKG = "BSK_LINK"
TRUTH_PACKET_ID = 1  # the truth stream's only packet type, for now
TRUTH_HEADER_BYTES = 4


def emit_cosmos_truth(items: list[Item], target: str) -> str:
    """Return the tlm defs text for a vehicle's TRUTH target (no cmd defs).

    Absence-of-cmd-def is a tripwire, like STEP_ACK's beacon exclusion:
    the truth side has no actuators, so the truth target has no commands,
    and a command attempt fails in the ground system before it can reach
    any wire.
    """
    lines = [
        f"# AUTO-GENERATED by codegen/eds2cosmos from cFS EDS "
        f"content {content_digest(items)} - DO NOT EDIT",
        "",
        f'TELEMETRY {target} TRUTH_STATE_TLM BIG_ENDIAN "ideal navigators truth at one '
        'sim instant - ground-only stream (provenance boundary: never on cFS)"',
        f"  APPEND_ID_ITEM PACKET_ID 8 UINT {TRUTH_PACKET_ID} "
        '"ground truth stream packet id (ground-owned framing, not CCSDS)"',
        '  APPEND_ITEM HEADER_RESERVED 8 UINT "zero on the wire"',
        '  APPEND_ITEM PACKET_LENGTH 16 UINT "octets following this header (total = field + 4)"',
    ]
    seen = {"PACKET_ID", "HEADER_RESERVED", "PACKET_LENGTH"}
    for item in items:
        lines.extend(_emit_item(item, seen, is_cmd=False))
    return "\n".join(lines) + "\n"


def main(argv: list[str] | None = None) -> int:
    """CLI entry."""
    ap = argparse.ArgumentParser(prog="eds2cosmos", description=__doc__.splitlines()[0])
    ap.add_argument("--cfs", type=Path, required=True, help="pinned cFS checkout")
    ap.add_argument("--mission-defs", type=Path, required=True, help="dir with topic-id XMLs")
    ap.add_argument("--out-cosmos", type=Path, help="dir for cmd/tlm .txt files")
    ap.add_argument("--out-c", type=Path, help="path for gs_tcp rs_mid_table.h")
    ap.add_argument(
        "--out-cmd-c", type=Path, help="path for sps sps_cmd_table.h (command dictionary)"
    )
    ap.add_argument("--target", default=TARGET, help=f"COSMOS target name (default {TARGET})")
    ap.add_argument(
        "--emit-truth",
        action="store_true",
        help="emit the truth target's tlm defs (from BSK_LINK/TruthState) instead of flight defs",
    )
    args = ap.parse_args(argv)

    eds = build_eds(args.cfs, args.mission_defs)
    if args.emit_truth:
        truth_items = eds.leaf(TRUTH_CONTAINER, TRUTH_PKG)
        text = emit_cosmos_truth(truth_items, target=args.target)
        if args.out_cosmos:
            args.out_cosmos.mkdir(parents=True, exist_ok=True)
            (args.out_cosmos / f"{args.target.lower()}_tlm_def.txt").write_text(text)
        print(f"truth defs: 1 tlm / 0 cmd for {args.target}")
        for warning in eds.warnings:
            print("WARN:", warning, file=sys.stderr)
        return 0

    all_packets = eds.packets()
    packets = mission_packets(all_packets)
    for p in all_packets:
        if p.name in MODEL_DRIFT_EXCLUSIONS:
            print(f"EXCLUDED {p.name}: {MODEL_DRIFT_EXCLUSIONS[p.name]}", file=sys.stderr)
        if p.name in BEACON_EXCLUSIONS:
            print(f"EXCLUDED {p.name}: {BEACON_EXCLUSIONS[p.name]}", file=sys.stderr)
    n_tlm = sum(1 for p in packets if p.direction == "tlm")
    n_cmd = len(packets) - n_tlm
    cmd_text, tlm_text = emit_cosmos(packets, target=args.target)

    if args.out_cosmos:
        args.out_cosmos.mkdir(parents=True, exist_ok=True)
        stem = args.target.lower()
        (args.out_cosmos / f"{stem}_cmd_def.txt").write_text(cmd_text)
        (args.out_cosmos / f"{stem}_tlm_def.txt").write_text(tlm_text)
    if args.out_c:
        args.out_c.parent.mkdir(parents=True, exist_ok=True)
        args.out_c.write_text(emit_c_table(packets, _cfs_sha(args.cfs)))

    if args.out_cmd_c:
        args.out_cmd_c.parent.mkdir(parents=True, exist_ok=True)
        args.out_cmd_c.write_text(emit_c_cmd_table(packets, _cfs_sha(args.cfs)))

    print(f"packets: {n_tlm} tlm / {n_cmd} cmd")
    for warning in eds.warnings:
        print("WARN:", warning, file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
