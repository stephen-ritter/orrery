"""Stored-command revival acceptance: a booted sequencer that stores nothing.

Requires the running mission (gs_tcp :5050/:5051) - and deliberately no
plant. The wake law seats sc's wakeup on the bsk_link sensor frame, so with
no sim flying there are no frames and no wakes: what this bench holds the
revived app to is the boot contract and the ground interface.

The boot half of the law (initialized; every RTS slot empty per the
factory-ships-no-sequences prune; the power-on autostart REFUSED) happens
before any socket exists - the beacon carries no replay - so the shell
script reads it from the craft's own console log. This probe owns what a
ground station can own from the moment it connects: SC_HK_TLM must appear
on the beacon (0x08AA) - the HK poll is sch's, needing no plant - and
SC_NOOP over the uplink must answer with sc's own receipt event and exactly
one counted ground request, no errors.

Nothing here predicts: every check compares what the craft reported with
what the mission law says the craft should have done.
"""

from __future__ import annotations

import argparse
import time

from codegen.eds2cosmos import build_eds
from sim.fleet import ports
from sim.ground_probe import CFS, MISSION_DEFS, GroundClient, build_cmd

SC_NOOP_FC = 0

# sc's receipt for a good NOOP. MEASURED, not read: the pinned bundle ships
# two headers defining SC_NOOP_INF_EID (fsw/inc/sc_events.h says 52,
# sc_eventids.h says 31) and the wire answered 52 - which header wins is
# include order, so the bench pins what the craft actually emits.
EID_NOOP = 52


def main(argv: list[str] | None = None) -> int:
    """Command the revived sequencer and verdict its receipts; exit = verdict."""
    ap = argparse.ArgumentParser(description="sc revival acceptance client")
    ap.add_argument("--instance", type=int, default=0, help="fleet port block to address")
    ap.add_argument("--deadline", type=float, default=60.0, help="wall seconds for the exchange")
    args = ap.parse_args(argv)

    eds = build_eds(CFS, MISSION_DEFS)
    pkts = {p.name: p for p in eds.packets()}
    sc_cmd = pkts["SC_CMD"]
    sc_hk = pkts["SC_HK_TLM"]
    evs = pkts["CFE_EVS_LONG_EVENT_MSG"]
    evs_app_off = evs.byte_offset(".AppName")
    evs_eid_off = evs.byte_offset(".EventID")
    evs_msg_off = evs.byte_offset(".Message")
    hk_cmdctr_off = sc_hk.byte_offset(".CmdCtr")
    hk_cmderr_off = sc_hk.byte_offset(".CmdErrCtr")

    block = ports(args.instance)
    client = GroundClient(tlm_port=block.beacon, cmd_port=block.uplink)

    print(f"connected; SC_CMD=0x{sc_cmd.msgid:04X} SC_HK_TLM=0x{sc_hk.msgid:04X}")

    noop_sent = False
    noop_acked = False
    hk_seen = 0
    ctr_base: int | None = None
    err_base: int | None = None
    ctr_after: int | None = None
    err_after: int | None = None

    deadline = time.monotonic() + args.deadline
    while time.monotonic() < deadline and not (noop_acked and ctr_after is not None):
        pkt = client.recv_packet(min(deadline, time.monotonic() + 0.5))
        if pkt is None:
            continue
        msgid = int.from_bytes(pkt[:2], "big")
        if msgid == evs.msgid:
            app = pkt[evs_app_off : evs_app_off + 20].split(b"\x00")[0].decode(errors="replace")
            # multi-byte fields ride the beacon target-native (little-endian
            # here) - same reason clock_probe unpacks its float64 as "<d";
            # the BIG_ENDIAN defs describe the CCSDS golden form, not this wire
            eid = int.from_bytes(pkt[evs_eid_off : evs_eid_off + 2], "little")
            text = pkt[evs_msg_off:].split(b"\x00")[0].decode(errors="replace")
            if app == "SC" and eid == EID_NOOP:
                noop_acked = True
                print(f"sc receipt: [{eid}] {text}")
        elif msgid == sc_hk.msgid:
            hk_seen += 1
            ctr, err = pkt[hk_cmdctr_off], pkt[hk_cmderr_off]
            print(f"HK #{hk_seen}: CmdCtr={ctr} CmdErrCtr={err}")
            if not noop_sent:
                ctr_base, err_base = ctr, err
                client.send_packet(build_cmd(sc_cmd.msgid, SC_NOOP_FC))
                noop_sent = True
                print(f"sent SC_NOOP over uplink: SC_CMD 0x{sc_cmd.msgid:04X} FC {SC_NOOP_FC}")
            elif noop_sent:
                # the HK AFTER the send carries the receipt. The law is the
                # DELTA: one clean ground request added. (An absolute
                # CmdErrCtr=0 would be wrong on this image: the refused
                # power-on autostart of the phantom RTS 001 is itself a
                # counted command error before the ground ever speaks -
                # the gatekeeper's own footsteps, asserted from the console
                # by the script, not charged to this command.)
                ctr_after, err_after = ctr, err

    if not noop_sent:
        print("SC BOOT FAILED: no SC_HK_TLM on the beacon - the sequencer is mute")
        return 1
    if not noop_acked:
        print("SC BOOT FAILED: SC_NOOP answered nothing (no NOOP event)")
        return 1
    if ctr_after != (ctr_base or 0) + 1 or err_after != err_base:
        print(
            f"SC BOOT FAILED: HK counter delta wrong: "
            f"CmdCtr {ctr_base}->{ctr_after}, CmdErrCtr {err_base}->{err_after}"
        )
        return 1

    print(
        "SC BOOT OK: mute without frames, answerable by ground: one noop, one receipt "
        f"({hk_seen} HK packets, CmdCtr {ctr_base}->{ctr_after}, CmdErrCtr unchanged {err_base})"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
