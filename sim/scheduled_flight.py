"""Scheduled-execution acceptance: a command scheduled aboard fires on the world's frame.

Full-loop, mission-in-the-loop, LOCKSTEP (same rig as the closed loop): the sim owns the
clock at 1 Hz, so "sim time T" is an observable the ground can check against
the beacon itself. From a COSMOS-role client this drives and proves, in order:

1. uploads (all before the first schedule horizon): SPS NOOP + ENABLE, then
   schedules FCS ENABLE at frame T1=40 and FCS SET_GAIN at T2=70 - absolute
   sim times, so the trajectory is a function of the SIM timeline, never of
   client wall-clock jitter;
2. the negatives the exit demands, all refused at the uplink and counted in
   HK (nothing enters the queue): a MsgId outside the dictionary (unknown),
   SET_GAIN with a lying length field (param), a 9th entry onto a queue
   pinned full by never-draining future-T entries (full); plus one entry
   uploaded while its T lay in the past - late, counted, fired anyway
   on the next frame (the queue never time-travels, and it never silently
   drops either). Uploads ride half-second spacing: a 14-packet burst once
   overflowed the evaluator's pipe on a host whose queues are truncated to
   10 and whose poll-cycle wakes delay the drain - SB ate the injections
   silently, and an operator's intent must not evaporate in OS latency;
3. frame-exact firing off the beacon: the fire event carries the evaluating
   frame's own timecode, and for T=40 it must read exactly 40.000 - "the
   first frame >= T", proven from the frame payload. No torque may exist at
   or before frame 40 (a fire cannot actuate before its own frame); from
   there the effect is CAUSAL, not frame-exact: the stored command joins
   fcs's pipe behind that frame's sensor copy, so actuation starts on the
   first frame delivered after the fire posts - T+1 on a host with prompt
   OS wake, T+2 on one whose OSAL poll cycle delays the evaluator (this
   laptop's truncated message queues, the O1 precondition). What stays
   exact on every host: the spacing. Two schedules 30 s apart move the
   vehicle exactly 30 frames apart - the quantization cancels;
4. SPS HK agrees with the wire: LastFiredMsgId / LastExecTimeS /
   LastFiredSimTimeS are the firing evidence, and Fired/Scheduled/Rejected/
   Late counters equal the exact numbers this script uploaded;
5. the SET_GAIN schedule re-tunes the loop live one wake cycle after T2,
   thirty frames behind the ENABLE effect, with no further contact from
   the ground - the satellite executing the future, by itself.

Offsets and MsgIds read from the EDS model, so this client and the flight
dictionary cannot quietly disagree about the wire.
"""

from __future__ import annotations

import argparse
import struct
import time

from codegen.eds2cosmos import build_eds
from codegen.eds_model import Packet
from sim.fcs_probe import FrameLog, wire_float
from sim.fleet import ports
from sim.ground_probe import CFS, MISSION_DEFS, GroundClient, build_cmd

T_ENABLE = 40.0  # sim second the scheduled FCS ENABLE is due at
T_GAIN = 70.0  # sim second the scheduled SET_GAIN is due at
T_PAST = 1.0  # already-passed T: the LATE upload (fires next frame)
T_FUTURE = 200.0  # never-reached horizon for the queue-fill NOOPs
NEW_KP = (25.0, 25.0, 25.0)
DEFAULT_KP = (60.0, 50.0, 45.0)  # FCS_DEFAULT_KP in fcs_app.c

EVS_SPS_FIRE_EID = 7  # sps_events.h SPS_FIRE_INF_EID


def schedule_packet(
    sched_msgid: int, exec_time_s: float, cmd_msgid: int, fc: int, data: bytes
) -> bytes:
    """Envelope per sps.xml: T(f64) MsgId(u16) Fc(u8) Len(u8) Data(48 B)."""
    payload = struct.pack("<d", exec_time_s) + struct.pack("<HBB", cmd_msgid, fc, len(data))
    payload += data.ljust(48, b"\0")
    return build_cmd(sched_msgid, 0, payload)


def gain_data(kp: tuple[float, float, float]) -> bytes:
    """FCS_SET_GAIN payload: three little-endian doubles, straight from EDS."""
    return struct.pack("<3d", *kp)


class SpsHkSnapshot:
    """The last SPS_HK_TLM seen on the beacon, decoded through the model."""

    def __init__(self, pkt: bytes, model: Packet) -> None:
        """Decode the queue state and firing evidence from raw beacon bytes."""

        def u(leaf: str) -> int:
            return model.wire_get(pkt, leaf)

        def f(leaf: str) -> float:
            return wire_float(pkt, model.byte_offset(leaf))

        self.enabled = u(".Enabled")
        self.queue_count = u(".QueueCount")
        self.queue_peak = u(".QueuePeak")
        self.scheduled_total = u(".ScheduledTotal")
        self.fired_total = u(".FiredTotal")
        self.rejected_unknown = u(".RejectedUnknownTotal")
        self.rejected_full = u(".RejectedFullTotal")
        self.rejected_param = u(".RejectedParamTotal")
        self.late_fired = u(".LateFiredTotal")
        self.frames_evaluated = u(".FramesEvaluated")
        self.last_fired_msgid = u(".LastFiredMsgId")
        self.last_exec_time_s = f(".LastExecTimeS")
        self.last_fired_sim_time_s = f(".LastFiredSimTimeS")
        self.last_frame_sim_time_s = f(".LastFrameSimTimeS")

    def __str__(self) -> str:
        """One readable line for the acceptance log."""
        return (
            f"sps hk: enabled={self.enabled} queue={self.queue_count} peak={self.queue_peak} "
            f"sched={self.scheduled_total} fired={self.fired_total} "
            f"rej(u/f/p)={self.rejected_unknown}/{self.rejected_full}/{self.rejected_param} "
            f"late={self.late_fired} frames={self.frames_evaluated} "
            f"fired(mid=0x{self.last_fired_msgid:04X} T={self.last_exec_time_s:.1f} "
            f"at={self.last_fired_sim_time_s:.1f})"
        )


def main(argv: list[str] | None = None) -> int:
    """Run the scheduled-execution acceptance; returns process exit code."""
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--instance", type=int, default=0, help="fleet port block to address")
    ap.add_argument(
        "--duration",
        type=float,
        default=115.0,
        help="wall seconds to observe (the paced sim runs ~SIM_DURATION sim-s)",
    )
    args = ap.parse_args()
    block = ports(args.instance)

    eds = build_eds(CFS, MISSION_DEFS)
    pkts = {p.name: p for p in eds.packets()}
    sched = pkts["SPS_SCHEDULE"]
    sps_cmd = pkts["SPS_CMD"]
    sps_send_hk = pkts["SPS_SEND_HK"]
    sps_hk = pkts["SPS_HK_TLM"]
    fcs_cmd = pkts["FCS_CMD"]
    fcs_gain = pkts["FCS_SET_GAIN"]

    client = GroundClient(tlm_port=block.beacon, cmd_port=block.uplink)
    log = FrameLog(pkts)
    last_sps_hk: SpsHkSnapshot | None = None

    def send(data: bytes) -> None:
        client.send_packet(data)

    def request_sps_hk() -> None:
        send(build_cmd(sps_send_hk.msgid, 0))

    print(
        f"connected; SPS_SCHEDULE=0x{sched.msgid:04X} SPS_CMD=0x{sps_cmd.msgid:04X} "
        f"FCS_CMD=0x{fcs_cmd.msgid:04X} SET_GAIN=0x{fcs_gain.msgid:04X}"
    )

    ok = False
    reason = "deadline expired"
    uploaded = False
    armed_checked = False
    deadline = time.monotonic() + args.duration

    try:
        while time.monotonic() < deadline:
            pkt = client.recv_packet(min(deadline, time.monotonic() + 0.5))
            if pkt is None:
                continue
            msgid = int.from_bytes(pkt[:2], "big")
            log.feed(msgid, pkt)
            if msgid == sps_hk.msgid:
                last_sps_hk = SpsHkSnapshot(pkt, sps_hk)
                # Fail fast on an un-armed queue (the first HK that already
                # shows schedules was necessarily composed after ENABLE was
                # processed - same pipe, FIFO): 3 minutes of silence would
                # otherwise end as a vague "deadline expired".
                if uploaded and not armed_checked and last_sps_hk.scheduled_total > 0:
                    armed_checked = True
                    if last_sps_hk.enabled == 0:
                        reason = "SPS ENABLE did not take: queue FROZEN with schedules aboard"
                        break

            if not uploaded and log.sensor_count >= 3:
                # --- uploads: everything before the first horizon (T=40),
                # at deliberate half-second spacing. Physics hangs on the
                # schedules, not on client timing, so wall-clock jitter
                # cannot move the trajectory - and the spacing keeps a
                # poll-cycle-delayed evaluator's pipe (host-truncated to 10
                # deep) from EVER seeing a burst it cannot drain: an SB pipe
                # overflow silently eats injections (this host caught
                # exactly that with a 14-packet burst), which would punish
                # the operator for the OS's latency.
                for step in (build_cmd(sps_cmd.msgid, 0), build_cmd(sps_cmd.msgid, 1)):
                    send(step)
                    time.sleep(0.4)
                request_sps_hk()
                # The LATE upload comes FIRST: its T has already passed, so
                # it fires on the very next evaluation and vacates its slot
                # BEFORE the fillers pin the queue full. Uploaded last it met
                # the full-queue refusal instead (measured: rej full=2,
                # late=0) - the queue's no-drop promise needs a slot to keep.
                send(schedule_packet(sched.msgid, T_PAST, fcs_cmd.msgid, 0, b""))  # LATE
                time.sleep(1.5)  # let it fire (next frame) before we meter the queue
                accepted = [
                    (T_ENABLE, fcs_cmd.msgid, 1, b""),  # the scheduled ENABLE
                    (T_GAIN, fcs_gain.msgid, 0, gain_data(NEW_KP)),  # live retune
                    # six far-future NOOPs: with ENABLE+GAIN these fill 8/8,
                    # and since T=200 never drains, the NEXT entry is refused
                    # no matter when or whether the mid-run fires pop slots.
                    *[(T_FUTURE, fcs_cmd.msgid, 0, b"") for _ in range(6)],
                ]
                for exec_t, mid, fc, data in accepted:
                    send(schedule_packet(sched.msgid, exec_t, mid, fc, data))
                    time.sleep(0.4)
                # --- the three refusals: 9th onto the pinned-full queue,
                # a MsgId outside the dictionary, a lying length field
                send(schedule_packet(sched.msgid, T_FUTURE, fcs_cmd.msgid, 0, b""))  # FULL
                time.sleep(0.4)
                send(schedule_packet(sched.msgid, T_FUTURE, 0xFFFF, 0, b""))  # not in dictionary
                time.sleep(0.4)
                send(
                    schedule_packet(sched.msgid, T_FUTURE, fcs_gain.msgid, 0, gain_data(NEW_KP)[:8])
                )  # lying length field
                request_sps_hk()
                uploaded = True
                print(
                    f"uploads in at sim t~{log.max_frame}: {len(accepted) + 1} schedules "
                    f"(T={T_PAST:.0f} LATE first, {T_ENABLE:.0f}, {T_GAIN:.0f}, 6x{T_FUTURE:.0f}) "
                    "+ 3 refusals (full, unknown, bad length)"
                )

            # mid-run witnesses (also fresh HK samples after each horizon)
            fires = [e for e in log.evs_events if e[0] == "SPS" and e[1] == EVS_SPS_FIRE_EID]
            if uploaded and len(fires) == 1 and log.max_frame >= 42:
                request_sps_hk()
            if uploaded and len(fires) == 2 and log.max_frame >= 72:
                request_sps_hk()

            if uploaded and log.max_frame >= 98 and last_sps_hk:
                reason = _verify(log, fcs_gain, last_sps_hk)
                ok = reason is None
                break
            # hard failure guard: torque may not exist before the fire's effect
            early = [f for f in log.torque_by_frame if f <= int(T_ENABLE)]
            if early and uploaded:
                reason = f"torque already commanded at frame {early[0]} (< fire frame + 1)"
                break
    finally:
        client.close()

    if not ok:
        print(f"events seen: {[f'{a}/{e}' for a, e, _ in log.evs_events][-10:]}")
        all_fleet_events = [
            f"{a}/{e}:{t[:40]}" for a, e, t in log.evs_events if a in ("SPS", "FCS")
        ]
        print(f"all SPS/FCS events: {all_fleet_events}")
        print(f"torque keys head: {sorted(log.torque_by_frame)[:5]}")
        if log.last_hk:
            hk = log.last_hk
            print(
                f"fcs hk: mode={hk.mode} frames={hk.sensor_frames} torques={hk.torque_issued} "
                f"kp={tuple(round(k, 1) for k in hk.kp)}"
            )
        if last_sps_hk:
            print(last_sps_hk)
        print(f"SCHEDULED RUN FAILED: {reason}")
        return 1
    print("SCHEDULED RUN OK")
    return 0


def _verify(log: FrameLog, fcs_gain: Packet, hk: SpsHkSnapshot) -> str | None:
    """Full acceptance arithmetic over everything the beacon showed.

    Returns None on success, else the first failed statement in prose.

    The queue's contract is frame-EXACT and checked so: the fire event
    carries the evaluating frame's own SimTimeS (from the payload, not from
    a stamp), so "fired on the first frame >= T" is proven to the sample.
    The EFFECT a fire has on another app is causal, not frame-exact: the
    stored command joins the target's pipe after that pipe's copy of the
    fire frame already sat there (FIFO), so actuation starts on the first
    frame whose DELIVERY followed the fire - T+1 when OS wake is prompt,
    T+2 when an OSAL poll cycle delays the evaluator's wake (this host runs
    truncated message queues; O1's named precondition). What IS exact and
    host-independent is the causality floor and the spacing: no torque at
    or before T, onset within one cycle after, and two schedules D apart
    move the vehicle exactly D frames apart - the quantization cancels.
    """
    fires = [(app, eid, text) for app, eid, text in log.evs_events if eid == EVS_SPS_FIRE_EID]
    if len(fires) != 3:
        return f"expected 3 SPS fire events, saw {len(fires)}"
    enable_fire = [t for _, _, t in fires if "mid=0x18C8" in t]
    if not any("T=40.000" in t and "frame t=40.000" in t for t in enable_fire):
        return "no fire of FCS_CMD mid at exactly frame t=40.000 (first frame >= T)"
    if not any(
        "mid=0x18CA" in t and "T=70.000" in t and "frame t=70.000" in t for _, _, t in fires
    ):
        return "no fire of FCS_SET_GAIN at exactly frame t=70.000"
    if not any("T=1.000" in t for _, _, t in fires):
        return "the late schedule (T=1.0) did not fire on its next frame"

    # Causality floor first: the fire cannot actuate before its own frame.
    if any(f <= int(T_ENABLE) for f in log.torque_by_frame):
        return "torque before the scheduled ENABLE could have acted"
    frames = sorted(log.torque_by_frame)
    onset = frames[0] - int(T_ENABLE)
    if onset not in (1, 2):
        return (
            f"torque onset is {onset} frames after the fire: expected 1 (prompt wake) "
            "or 2 (poll-cycle host)"
        )
    missing = [f for f in range(frames[0], frames[-1] + 1) if f not in log.torque_by_frame]
    if missing:
        return f"frames without a torque answer: {missing[:5]}"

    # Per-frame law: exactly one gain set fits each torque; the frame where
    # the NEW set starts fitting is the observed switch, which must sit
    # exactly 30 frames behind the onset (T=70 minus T=40) - the host's
    # quantization cancels, the schedule's spacing cannot lie.
    def _fits(frame: int, kp: tuple[float, float, float]) -> bool:
        omega = log.sensor_omega.get(frame)
        if omega is None:
            return False
        return all(
            abs(tau - (-k * w)) <= 1e-9
            for tau, k, w in zip(log.torque_by_frame[frame], kp, omega, strict=True)
        )

    switch: int | None = None
    for frame in frames:
        fits_default = _fits(frame, DEFAULT_KP)
        fits_new = _fits(frame, NEW_KP)
        if not fits_default and not fits_new:
            omega = log.sensor_omega.get(frame)
            return f"frame {frame}: torque matches neither gain law (omega={omega})"
        if fits_default and fits_new:
            return f"frame {frame}: torque ambiguous between gains (model drift?)"
        if fits_new and switch is None:
            switch = frame
    if switch is None:
        return "the scheduled SET_GAIN never took effect on the torque stream"
    if switch - frames[0] != int(T_GAIN - T_ENABLE):
        return (
            f"effect spacing {switch - frames[0]} != schedule spacing "
            f"{int(T_GAIN - T_ENABLE)} (fires are frame-exact; effects must keep the distance)"
        )

    # HK agrees with the wire, count for count
    checks = [
        (hk.scheduled_total, 9, "ScheduledTotal"),
        (hk.fired_total, 3, "FiredTotal"),
        (hk.rejected_unknown, 1, "RejectedUnknownTotal"),
        (hk.rejected_full, 1, "RejectedFullTotal"),
        (hk.rejected_param, 1, "RejectedParamTotal"),
        (hk.late_fired, 1, "LateFiredTotal"),
        (hk.queue_count, 6, "QueueCount (9 accepted - 3 fired)"),
        (hk.queue_peak, 8, "QueuePeak"),
        (hk.enabled, 1, "Enabled"),
        (hk.last_fired_msgid, fcs_gain.msgid, "LastFiredMsgId"),
        (abs(hk.last_exec_time_s - T_GAIN) < 1e-9, True, "LastExecTimeS == 70.0"),
        (abs(hk.last_fired_sim_time_s - T_GAIN) < 1e-9, True, "LastFiredSimTimeS == 70.0"),
        (hk.frames_evaluated > 90, True, "FramesEvaluated tracks the world"),
    ]
    for got, want, label in checks:
        if got != want:
            return f"{label}: {got} != {want}"

    rejections = [f"{a}/{e}" for a, e, _ in log.evs_events if a == "SPS" and e in (8, 9, 10)]
    print(f"rejections on EVS: {rejections}")
    print(hk)
    spacing = switch - frames[0]
    print(
        f"fire evidence: {len(fires)} events at exact frame times; torque onset {frames[0]} "
        f"(fire+{onset}), gain effect {switch} (spacing {spacing} == schedule spacing 30)"
    )
    return None


if __name__ == "__main__":
    raise SystemExit(main())
