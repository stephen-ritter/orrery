"""Rung-1 burn acceptance: the quota door, the per-tick stream, both ledgers.

Requires the running mission (gs_tcp :5050/:5051) and the burn plant flying
in lockstep (``forge_scenario --zmq --pace --scenario sim/scenarios/burn.toml``
: four 5 N devices in the CoM plane, so in-plane demand is feasible and an
over-demand saturates honestly). From a ground seat this drives, in order:

1. sense: frames flow, HK says BurnState 0 - and the control loop stays
   DISABLED for the whole run, because burn authority is the ground's door,
   not a side effect of the attitude law's mode;
2. REFUSE the nonsense ask (negative force): EVS + HK refusal counter, no
   quota opens, the world stays untouched (truth BurnId 0);
3. open burn 1 (3 N along +Y for 10 s), then REFUSE an overlapping second
   ask while it runs - quotas do not queue;
4. watch the claim rise onboard (HK BurnFramesForwarded / BurnImpulseClaimed
   plus the BSK_LINK_BURN_COMMAND_TLM frames on the beacon) while the APPLIED
   ledger rises on the truth stream - the same burn read through two
   independent ledgers, the two-ledger law breathing;
5. BURNSTOP closes burn 1 mid-quota; a second BURNSTOP with nothing to stop
   is REFUSED - a stop with no burn is a misunderstanding, not a no-op;
6. burn 2 (2.5 N +X, 4 s) closes on its own clock (EVS complete), HK LastBurnId
   advances, and the ledgers reconcile within a tick's grace;
7. the honest gap: 8 N along +Y passes the craft's door (firmware caps
   nonsense, not the plant) and saturates in the world - HK claims 8 N per
   tick, truth applies 5 N and counts every tick saturated. Claim and
   applied MUST diverge here, and the world says so with its own counter.

Wire offsets come from the EDS model end to end; the truth frame is decoded
as the 18 f64 the EDS declares. The sim is the only place both ledgers can
be read - that is the law, and this probe is its exhibit.
"""

from __future__ import annotations

import argparse
import struct
import time

import zmq

from codegen.eds2cosmos import build_eds
from sim.fleet import ports
from sim.ground_probe import CFS, MISSION_DEFS, GroundClient, build_cmd

FCS_APP = "FCS"
BURNSTOP_FC = 3
EVS_FCS_BURN_OPEN_EID = 18  # fcs_events.h FCS_BURN_INF_EID
EVS_FCS_BURN_REJ_EID = 19  # FCS_BURN_REJ_ERR_EID
EVS_FCS_BURNSTOP_EID = 21  # FCS_BURNSTOP_INF_EID
EVS_FCS_BURN_DONE_EID = 22  # FCS_BURN_DONE_INF_EID

TRUTH_F64_COUNT = 18  # BSK_LINK/TruthState: 14 orbit/attitude + 4 burn rows
IMPULSE = 14  # BurnImpulseAppliedNs
TICKS_APPLIED = 15  # BurnTicksApplied
TICKS_SATURATED = 16  # BurnTicksSaturated
TRUTH_BURN_ID = 17  # BurnId

MAX_NOZZLE_N = 5.0  # every device in sim/scenarios/burn.toml
TICK_S = 1.0  # control tick the plant runs at (the claim's nominal promise)


class TruthWatcher:
    """The world's own voice: the sim's truth PUB, ground-side.

    The applied ledger lives here and nowhere else onboard can see it -
    that separation IS the two-ledger law. Raw 144 B frames (18 LE f64)
    on topic TRUTH, one per lockstep tick.
    """

    def __init__(self, truth_dial: str) -> None:
        """Subscribe to the sim's truth PUB for this instance's port block."""
        context = zmq.Context.instance()
        self.sock = context.socket(zmq.SUB)
        self.sock.connect(truth_dial)
        self.sock.setsockopt(zmq.SUBSCRIBE, b"TRUTH")
        self.sock.setsockopt(zmq.RCVTIMEO, 200)
        self.latest: tuple[float, ...] | None = None
        self.frames = 0

    def poll(self) -> None:
        """Drain queued truth frames, keeping the newest decoded one."""
        while True:
            try:
                parts = self.sock.recv_multipart(zmq.NOBLOCK)
            except zmq.Again:
                return
            if len(parts) == 2 and len(parts[1]) == TRUTH_F64_COUNT * 8:
                self.latest = struct.unpack("<18d", parts[1])
                self.frames += 1


class BurnLog:
    """Beacon-side evidence: HK claims, the burn-frame stream, events."""

    def __init__(self, pkts: dict) -> None:
        """Cache the EDS packet models; decode happens once per field, not per frame."""
        self.sensor = pkts["BSK_LINK_SENSOR_FRAME_TLM"]
        self.torque = pkts["BSK_LINK_TORQUE_COMMAND_TLM"]
        self.hk = pkts["FCS_HK_TLM"]
        self.burn_tlm = pkts["BSK_LINK_BURN_COMMAND_TLM"]
        self.evs = pkts["CFE_EVS_LONG_EVENT_MSG"]
        self.evs_app_off = self.evs.byte_offset(".AppName")
        self.evs_id_off = self.evs.byte_offset(".EventID")
        self.evs_msg_off = self.evs.byte_offset(".Message")
        self.sensor_frames: set[int] = set()
        self.torque_packets = 0
        self.burn_frames_on_beacon = 0
        self.last_hk: dict[str, float | int | None] | None = None
        self.events: list[tuple[str, int]] = []
        self.texts: dict[tuple[str, int], str] = {}

    def feed(self, msgid: int, pkt: bytes) -> None:
        """Decode one beacon packet into the claim-side record."""
        if msgid == self.sensor.msgid:
            self.sensor_frames.add(
                int(struct.unpack_from("<Q", pkt, self.sensor.byte_offset(".Frame"))[0])
            )
        elif msgid == self.torque.msgid:
            self.torque_packets += 1
        elif msgid == self.burn_tlm.msgid:
            self.burn_frames_on_beacon += 1
        elif msgid == self.hk.msgid:
            self.last_hk = {
                "mode": self.hk.wire_get(pkt, ".Mode"),
                "burn_state": self.hk.wire_get(pkt, ".BurnState"),
                "accepted": self.hk.wire_get(pkt, ".BurnCommandsAccepted"),
                "refused": self.hk.wire_get(pkt, ".BurnCommandsRefused"),
                "forwarded": self.hk.wire_get(pkt, ".BurnFramesForwarded"),
                "claimed": struct.unpack_from(
                    "<d", pkt, self.hk.byte_offset(".BurnImpulseClaimed")
                )[0],
                "last_force": struct.unpack_from("<d", pkt, self.hk.byte_offset(".LastBurnForceN"))[
                    0
                ],
                "last_id": self.hk.wire_get(pkt, ".LastBurnId"),
            }
        elif msgid == self.evs.msgid:
            app = (
                pkt[self.evs_app_off : self.evs_app_off + 20]
                .split(b"\x00")[0]
                .decode(errors="replace")
            )
            eid = int.from_bytes(pkt[self.evs_id_off : self.evs_id_off + 2], "little")
            self.events.append((app, eid))
            self.texts[(app, eid)] = (
                pkt[self.evs_msg_off :].split(b"\x00")[0].decode(errors="replace")
            )

    def saw(self, app: str, eid: int) -> bool:
        """True when the event has been seen at least once."""
        return (app, eid) in self.events

    def count(self, app: str, eid: int) -> int:
        """How many times this exact (app, event id) pair has fired."""
        return sum(1 for got_app, got_id in self.events if got_app == app and got_id == eid)

    def last_text(self, app: str, eid: int) -> str:
        """The most recent message text carried by this event."""
        return self.texts.get((app, eid), "")


def hk_field(log: BurnLog, key: str) -> float:
    """An HK burn field as a float, or -1 if HK has never arrived."""
    hk = log.last_hk
    if hk is None:
        return -1.0
    value = hk[key]
    if value is None:
        return -1.0
    return float(value)


def burn_packet(
    pkts: dict, force_n: float, duration_s: float, axis: tuple[float, float, float]
) -> bytes:
    """One FCS_BURN ask: five doubles in EDS field order (Force, Duration, Axis)."""
    burn = pkts["FCS_BURN"]
    return build_cmd(burn.msgid, 0, struct.pack("<5d", force_n, duration_s, *axis))


def main(argv: list[str] | None = None) -> int:
    """Run the burn acceptance; returns the process exit code."""
    ap = argparse.ArgumentParser(description="Rung-1 burn acceptance client")
    ap.add_argument("--instance", type=int, default=0, help="fleet port block to address")
    args = ap.parse_args(argv)

    eds = build_eds(CFS, MISSION_DEFS)
    pkts = {p.name: p for p in eds.packets()}
    fcs_cmd = pkts["FCS_CMD"]

    block = ports(args.instance)
    client = GroundClient(tlm_port=block.beacon, cmd_port=block.uplink)
    log = BurnLog(pkts)
    truth = TruthWatcher(f"tcp://127.0.0.1:{block.truth}")
    send_hk = pkts["FCS_SEND_HK"]
    print(
        f"connected; FCS_BURN=0x{pkts['FCS_BURN'].msgid:04X} "
        f"BURNSTOP="
        f"0x{fcs_cmd.msgid:04X}/FC{BURNSTOP_FC}"
    )

    deadline = time.monotonic() + 420.0
    stage = "sense"
    forwarded_at_stop = 0
    claimed_at_stop = 0.0
    fails: list[str] = []
    last_pull = 0.0

    def pull_hk(force: bool = False) -> None:
        """Demand fresh evidence: HK rides a slow scheduler tick, so PULL it.

        An HK published in reply to this beat cannot predate any ground act
        already uplinked, which turns every stage's freshness question from a
        cadence race into a round trip. After each command: force a pull;
        between beats: one per second keeps the ledger honest cheaply.
        """
        nonlocal last_pull
        now = time.monotonic()
        if force or now - last_pull >= 1.0:
            client.send_packet(build_cmd(send_hk.msgid, 0))
            last_pull = now

    def uplink(data: bytes) -> None:
        """One ground act, then immediately demand the HK that proves it."""
        client.send_packet(data)
        pull_hk(force=True)

    def require(cond: bool, verdict: str) -> None:
        """Fail LOUD but keep flying: a half-run hides the second ledger."""
        if cond:
            print(verdict)
        else:
            fails.append(verdict)
            print(f"FAIL: {verdict}")

    try:
        while time.monotonic() < deadline and stage != "done":
            pkt = client.recv_packet(min(deadline, time.monotonic() + 0.2))
            truth.poll()
            pull_hk()
            if pkt is not None:
                log.feed(int.from_bytes(pkt[:2], "big"), pkt)
            hk_ready = log.last_hk is not None

            if stage == "sense":
                if len(log.sensor_frames) >= 3 and hk_ready:
                    mode = int(hk_field(log, "mode"))
                    state = int(hk_field(log, "burn_state"))
                    require(
                        mode == 0 and state == 0,
                        f"sense: frames flowing, loop DISABLED, burn door idle "
                        f"(mode={mode}, burn_state={state})",
                    )
                    uplink(burn_packet(pkts, -5.0, 5.0, (0.0, 1.0, 0.0)))
                    stage = "refuse-nonsense"
                    print("uplinked nonsense ask (-5 N): expecting refusal")
                continue

            if stage == "refuse-nonsense":
                # EVS proves the door spoke; HK proves it landed in the
                # ledger. HK rides the scheduler tick, so wait for the HK
                # that SHOWS the refusal, never assume the event implies it.
                if log.saw(FCS_APP, EVS_FCS_BURN_REJ_EID) and hk_field(log, "refused") >= 1:
                    refused = hk_field(log, "refused")
                    require(
                        refused >= 1
                        and hk_field(log, "burn_state") == 0
                        and hk_field(log, "accepted") == 0
                        and (truth.latest is None or truth.latest[TRUTH_BURN_ID] == 0.0),
                        f"refusal proven: EVS + HK refused={int(refused)}, world untouched "
                        f"({log.last_text(FCS_APP, EVS_FCS_BURN_REJ_EID)})",
                    )
                    uplink(burn_packet(pkts, 3.0, 15.0, (0.0, 1.0, 0.0)))
                    stage = "open"
                    print("uplinked burn 1: 3 N along +Y for 15 s (while the loop stays DISABLED)")
                continue

            if stage == "open":
                if (
                    log.saw(FCS_APP, EVS_FCS_BURN_OPEN_EID)
                    and hk_field(log, "burn_state") == 1
                    and hk_field(log, "last_id") == 1
                ):
                    require(
                        hk_field(log, "accepted") == 1
                        and hk_field(log, "last_id") == 1
                        and abs(hk_field(log, "last_force") - 3.0) < 1e-12,
                        f"burn 1 open: claim {hk_field(log, 'claimed'):.1f} N s (accepted=1, id=1)",
                    )
                    uplink(burn_packet(pkts, 2.0, 5.0, (1.0, 0.0, 0.0)))
                    stage = "refuse-overlap"
                    print("uplinked an overlapping ask: expecting refusal (quotas do not queue)")
                continue

            if stage == "refuse-overlap":
                if hk_field(log, "refused") >= 2:
                    require(
                        hk_field(log, "burn_state") == 1,
                        f"overlap refused, burn 1 unharmed: refused="
                        f"{int(hk_field(log, 'refused'))} and still burning",
                    )
                    stage = "stream"
                continue

            if stage == "stream":
                claimed = hk_field(log, "claimed")
                truth_ok = (
                    truth.latest is not None
                    and truth.latest[TICKS_APPLIED] >= 2
                    and truth.latest[IMPULSE] > 0.0
                )
                if (
                    hk_field(log, "forwarded") >= 3
                    and claimed >= 2 * 3.0
                    and truth_ok
                    and log.burn_frames_on_beacon >= 1
                ):
                    require(
                        abs(truth.latest[IMPULSE] - claimed) <= 4 * 3.0 * TICK_S,
                        f"two ledgers rise together: craft claims {claimed:.1f} N s "
                        f"({int(hk_field(log, 'forwarded'))} frames, "
                        f"{log.burn_frames_on_beacon} on beacon), "
                        f"world applied {truth.latest[IMPULSE]:.1f} N s "
                        f"({int(truth.latest[TICKS_APPLIED])} ticks)",
                    )
                    uplink(build_cmd(fcs_cmd.msgid, BURNSTOP_FC))
                    stage = "stop"
                    print("uplinked BURNSTOP mid-quota")
                continue

            if stage == "stop":
                # forwarded>=3 proves the HK is mid-burn-1-era or later, so
                # state==0 on it means the STOP closed it, not a pre-open echo.
                if (
                    log.saw(FCS_APP, EVS_FCS_BURNSTOP_EID)
                    and hk_field(log, "burn_state") == 0
                    and hk_field(log, "forwarded") >= 3
                ):
                    forwarded_at_stop = int(hk_field(log, "forwarded"))
                    claimed_at_stop = hk_field(log, "claimed")
                    require(
                        True,
                        f"BURNSTOP honored: burn 1 stopped at {forwarded_at_stop} frames, "
                        f"{claimed_at_stop:.1f} N s claimed (10 s quota cut short)",
                    )
                    uplink(build_cmd(fcs_cmd.msgid, BURNSTOP_FC))
                    stage = "stop-idle"
                    print("uplinked BURNSTOP AGAIN with no burn: expecting refusal")
                continue

            if stage == "stop-idle":
                if log.count(FCS_APP, EVS_FCS_BURN_REJ_EID) >= 3:
                    require(
                        hk_field(log, "burn_state") == 0
                        and hk_field(log, "forwarded") == forwarded_at_stop,
                        f"stop-with-nothing refused: forwarded stayed {forwarded_at_stop}",
                    )
                    uplink(burn_packet(pkts, 2.5, 4.0, (1.0, 0.0, 0.0)))
                    stage = "clock"
                    print("uplinked burn 2: 2.5 N along +X for 4 s (closing on its own clock)")
                continue

            if stage == "clock":
                # last_id==2 proves the HK is from burn 2's era; state==0 on
                # THAT packet is the close. An HK sent during the idle gap
                # after burn 1's stop (state 0, last_id 1) must not pass -
                # freshness is proven by content, not by arrival order.
                if (
                    log.count(FCS_APP, EVS_FCS_BURN_DONE_EID) >= 1
                    and hk_field(log, "last_id") == 2
                    and hk_field(log, "burn_state") == 0
                ):
                    require(
                        True,
                        f"burn 2 closed by clock: {log.last_text(FCS_APP, EVS_FCS_BURN_DONE_EID)}",
                    )
                    uplink(burn_packet(pkts, 8.0, 4.0, (0.0, 1.0, 0.0)))
                    stage = "saturate"
                    print("uplinked 8 N along +Y: the door allows it, the 5 N plant must not")
                continue

            if stage == "saturate":
                truth_ok = (
                    truth.latest is not None
                    and truth.latest[TRUTH_BURN_ID] == 3.0
                    and truth.latest[TICKS_APPLIED] >= 2
                )
                if (
                    truth_ok
                    and log.count(FCS_APP, EVS_FCS_BURN_DONE_EID) >= 2
                    and hk_field(log, "burn_state") == 0
                ):
                    applied = truth.latest[IMPULSE]
                    claimed_total = hk_field(log, "claimed")
                    saturated_ticks = truth.latest[TICKS_SATURATED]
                    applied_ticks = truth.latest[TICKS_APPLIED]
                    require(
                        saturated_ticks >= 3,
                        f"world says saturated: {int(saturated_ticks)} ticks over-authority "
                        f"(craft door capped nonsense, not the plant - as designed)",
                    )
                    require(
                        abs(applied - claimed_total) > 0.5 * MAX_NOZZLE_N,
                        f"the ledgers DIVERGE honestly: craft claims {claimed_total:.1f} N s, "
                        f"world applied {applied:.1f} N s over {int(applied_ticks)} ticks",
                    )
                    require(
                        log.torque_packets == 0,
                        f"the disabled law stayed silent through every burn "
                        f"({log.torque_packets} torque pkts)",
                    )
                    stage = "done"
                continue

        if stage != "done":
            fails.append(f"run ended in stage '{stage}' before the story completed")
    finally:
        client.close()

    print(
        f"burn summary: beacon burn frames={log.burn_frames_on_beacon}, "
        f"HK forwarded={int(hk_field(log, 'forwarded'))} "
        f"claimed={hk_field(log, 'claimed'):.1f} N s, "
        f"truth applied={truth.latest[IMPULSE] if truth.latest else -1:.1f} N s "
        f"saturated={int(truth.latest[TICKS_SATURATED]) if truth.latest else -1}"
    )
    if fails:
        print(f"BURN BENCH FAILED ({len(fails)}):")
        for line in fails:
            print(f"  - {line}")
        return 1
    print(
        "BURN BENCH OK: quota doors, per-tick stream, apply-once truth, honest divergence",
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
