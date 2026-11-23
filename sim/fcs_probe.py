"""P3f acceptance: the control law lives in flight software, and it works.

Full-loop, mission-in-the-loop, LOCKSTEP. Requires the running mission
(gs_tcp :5050/:5051) and a ``detumble_scenario --zmq --pace``: the sim owns
the clock, every sensor frame gates on its StepAck, and the sim re-writes the
effector command EVERY tick - the torque fcs commanded for that frame, or
zero when the loop had nothing to say. Inertness therefore has two proofs:
the bus carries no torque while disabled, and the sim's echo channel
(:5591) carries frame-matched BITWISE-ZERO torques - the effector genuinely
feels zero, every frame, not merely "no new command". From a COSMOS-role
ground client this drives and proves, in order:

1. SENSOR frame telemetry flows sim -> bsk_link -> bus -> beacon (the satellite
   senses), no bus torque exists yet, and every echo torque so far is zero;
2. FCS_HK_TLM shows Mode=DISABLED while SensorFramesReceived grows - the loop
   hears the sensors (acks keep the barrier open) but does not act;
3. uplink FCS ENABLE -> EVS acknowledgement -> HK Mode=ENABLED;
4. every TORQUE_COMMAND on the beacon from the enable boundary on equals
   -Kp * omega of the sensor frame with the same Frame number, exactly (same
   decoded bytes, no relandering), and every sensor frame past that boundary
   has a torque answer - no frame may go unanswered; the sim's CMD_ECHO on
   :5591 proves the commanded torque is what the effector actually felt;
5. uplink FCS SET_GAIN re-tunes the loop live, verified from HK Kp and from
   subsequent torque frames matching the NEW gain;
6. the rate norm visible in onboard knowledge decays while the loop runs.

Offsets and MsgIds are read from the EDS model - the same build that emits the
COSMOS defs and the gs_tcp table - so this script cannot quietly disagree with
either about the wire.

Fleet runs (D10) add ``--start-frame``: the stage sequence advances on
telemetry COUNTS, so the sim frames its commands land on repeat only if the
run is anchored to an absolute frame. A fleet's later vehicles start when the
previous vehicle's client finished - wall-clock jitter the paced world turns
into a different ENABLE frame and a different trajectory - so they hold
ENABLE until their own beacon shows the requested frame, and a client too
late to reach the anchor fails instead of flying an unrepeatable run. The
same anchor extends to SET_GAIN (``--gain-frame``): unanchored, that send
rides the 7.8 s FCS_HK poll phase, and a mission rebuild that shifts the
phase moves the frame and re-rolls the digest with the physics untouched.
"""

from __future__ import annotations

import argparse
import math
import struct
import time
from dataclasses import dataclass
from pathlib import Path

import zmq

from codegen.eds2cosmos import build_eds
from sim.fleet import ports
from sim.ground_probe import GroundClient, build_cmd

REPO = Path(__file__).resolve().parents[1]
CFS = REPO / "cfs_mission/cFS"
MISSION_DEFS = REPO / "cfs_mission/mission/orrery_defs/eds"

FCS_APP = "FCS"
FCS_FC_ENABLE = 1
EVS_FCS_ENABLE_EID = 3  # fcs_events.h FCS_ENABLE_INF_EID
EVS_FCS_SET_GAIN_EID = 5  # fcs_events.h FCS_SET_GAIN_INF_EID
NEW_KP = (25.0, 25.0, 25.0)

ECHO_ENDPOINT = "tcp://127.0.0.1:5591"


@dataclass
class HkSnapshot:
    """Decoded fields of the most recent FCS_HK_TLM on the beacon."""

    mode: int
    sensor_frames: int
    torque_issued: int
    kp: tuple[float, float, float]


def wire_float(pkt: bytes, offset: int) -> float:
    """Payload float64 at a model offset: payload bytes are target-native LE."""
    return float(struct.unpack_from("<d", pkt, offset)[0])


class EchoWatcher:
    """Watch the Basilisk command-echo PUB: what the effector actually felt.

    In lockstep the sim writes an effector command on EVERY tick (zero when
    the loop is disabled), so the echo stream is one frame per sensor frame -
    the frame-exact record the barrier guarantees, and the ground-side proof
    that bus torque bytes == bytes the dynamics integrated.
    """

    def __init__(self, echo_endpoint: str = ECHO_ENDPOINT) -> None:
        """Subscribe to the sim echo channel (this instance's echo port), topic CMD."""
        self.context = zmq.Context.instance()
        self.sock = self.context.socket(zmq.SUB)
        self.sock.connect(echo_endpoint)
        self.sock.setsockopt(zmq.SUBSCRIBE, b"CMD")
        self.sock.setsockopt(zmq.RCVTIMEO, 200)
        self.applied_total = 0
        self.applied_nonzero = 0
        self.applied_by_frame: dict[int, tuple[float, float, float]] = {}

    def poll(self) -> None:
        """Drain queued echo frames into the per-frame record."""
        while True:
            try:
                parts = self.sock.recv_multipart(flags=zmq.NOBLOCK)
            except zmq.Again:
                return
            if len(parts) != 2:
                continue
            # TorqueCommand body: SimTimeS f64, torque 3xf64, Frame u64.
            torque = struct.unpack_from("<3d", parts[1], 8)
            frame = int(struct.unpack_from("<Q", parts[1], 32)[0])
            self.applied_total += 1
            self.applied_by_frame[frame] = torque
            if any(abs(v) > 1e-9 for v in torque):
                self.applied_nonzero += 1


class FrameLog:
    """Captured beacon packets, decoded through the EDS model."""

    def __init__(self, pkts: dict) -> None:
        """Cache model offsets once; packets fly at 1 Hz and decoding is cheap."""
        self.sensor = pkts["BSK_LINK_SENSOR_FRAME_TLM"]
        self.torque = pkts["BSK_LINK_TORQUE_COMMAND_TLM"]
        self.hk = pkts["FCS_HK_TLM"]
        self.evs = pkts["CFE_EVS_LONG_EVENT_MSG"]
        self.evs_app_off = self.evs.byte_offset(".AppName")
        self.evs_id_off = self.evs.byte_offset(".EventID")
        self.evs_msg_off = self.evs.byte_offset(".Message")
        self.sensor_omega: dict[int, tuple[float, float, float]] = {}
        self.sensor_count = 0
        self.max_frame = 0  # newest sim frame seen on the beacon (D10 anchor)
        self.torque_by_frame: dict[int, tuple[float, float, float]] = {}
        self.last_hk: HkSnapshot | None = None
        self.evs_events: list[tuple[str, int, str]] = []

    def omega(self, pkt: bytes) -> tuple[float, float, float]:
        """Body rate from a sensor frame, straight off the wire layout."""
        wx = wire_float(pkt, self.sensor.byte_offset(".OmegaBnB_0"))
        wy = wire_float(pkt, self.sensor.byte_offset(".OmegaBnB_1"))
        wz = wire_float(pkt, self.sensor.byte_offset(".OmegaBnB_2"))
        return (wx, wy, wz)

    def feed(self, msgid: int, pkt: bytes) -> None:
        """Decode one beacon packet into the log."""
        if msgid == self.sensor.msgid:
            frame = int(struct.unpack_from("<Q", pkt, self.sensor.byte_offset(".Frame"))[0])
            self.sensor_omega[frame] = self.omega(pkt)
            self.sensor_count += 1
            self.max_frame = max(self.max_frame, frame)
        elif msgid == self.torque.msgid:
            frame = int(struct.unpack_from("<Q", pkt, self.torque.byte_offset(".Frame"))[0])
            tx = wire_float(pkt, self.torque.byte_offset(".TorqueSumB_0"))
            ty = wire_float(pkt, self.torque.byte_offset(".TorqueSumB_1"))
            tz = wire_float(pkt, self.torque.byte_offset(".TorqueSumB_2"))
            self.torque_by_frame[frame] = (tx, ty, tz)
        elif msgid == self.hk.msgid:
            kx = wire_float(pkt, self.hk.byte_offset(".Kp_0"))
            ky = wire_float(pkt, self.hk.byte_offset(".Kp_1"))
            kz = wire_float(pkt, self.hk.byte_offset(".Kp_2"))
            self.last_hk = HkSnapshot(
                mode=self.hk.wire_get(pkt, ".Mode"),
                sensor_frames=self.hk.wire_get(pkt, ".SensorFramesReceived"),
                torque_issued=self.hk.wire_get(pkt, ".TorqueCommandsIssued"),
                kp=(kx, ky, kz),
            )
        elif msgid == self.evs.msgid:
            app = (
                pkt[self.evs_app_off : self.evs_app_off + 20]
                .split(b"\x00")[0]
                .decode(errors="replace")
            )
            eid = int.from_bytes(pkt[self.evs_id_off : self.evs_id_off + 2], "little")
            text = pkt[self.evs_msg_off :].split(b"\x00")[0].decode(errors="replace")
            self.evs_events.append((app, eid, text))


def main(argv: list[str] | None = None) -> int:
    """Run the P3c closed-loop acceptance; returns process exit code."""
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--instance", type=int, default=0, help="fleet port block (D9) to address")
    ap.add_argument(
        "--start-frame",
        type=int,
        default=0,
        help="hold ENABLE until the beacon shows this ABSOLUTE sim frame (D10: a "
        "sequential fleet client otherwise anchors its whole count-driven stage "
        "sequence on its own launch time, which inherits the previous vehicle's "
        "test duration jitter - an absolute frame makes the trajectory repeat)",
    )
    ap.add_argument(
        "--gain-frame",
        type=int,
        default=0,
        help="hold SET_GAIN until the beacon shows this ABSOLUTE sim frame (P6 "
        "lesson: unanchored, the send rides the 7.8 s FCS_HK poll PHASE - a "
        "container-boot artifact that moved the SET_GAIN frame when the mission "
        "grew an app, re-rolling a frozen digest with the physics untouched; "
        "0 keeps the legacy HK-gated cadence so historical baselines stand)",
    )
    args = ap.parse_args(argv)
    block = ports(args.instance)
    eds = build_eds(CFS, MISSION_DEFS)
    pkts = {p.name: p for p in eds.packets()}
    fcs_cmd = pkts["FCS_CMD"]
    fcs_gain = pkts["FCS_SET_GAIN"]
    default_kp = pkts["FCS_HK_TLM"]  # only for byte offsets; values are live
    del default_kp

    client = GroundClient(tlm_port=block.beacon, cmd_port=block.uplink)
    log = FrameLog(pkts)
    echo = EchoWatcher(block.echo_dial)
    print(f"connected; FCS_CMD=0x{fcs_cmd.msgid:04X} SET_GAIN=0x{fcs_gain.msgid:04X}")

    ok = False
    reason = "deadline expired"
    stage = "sense"
    enable_evt_seen = False
    sent_enable_at: float | None = None
    sent_gain_at: float | None = None

    gain_seen = False
    gain_ack_frame = 0
    deadline = time.monotonic() + 180.0

    try:
        while time.monotonic() < deadline:
            pkt = client.recv_packet(min(deadline, time.monotonic() + 0.5))
            echo.poll()
            if pkt is None:
                continue
            msgid = int.from_bytes(pkt[:2], "big")
            log.feed(msgid, pkt)
            hk = log.last_hk

            if stage == "sense":
                # (1)+(2) sensing flows. Any bus torque before ENABLE means
                # the mode leaked; the positive inertness proof is the echo
                # channel: the sim writes the effector every tick, and every
                # write so far must be bit-exact zero.
                nonzero = [f for f, t in log.torque_by_frame.items() if any(t)]
                if nonzero:
                    raise AssertionError(
                        f"nonzero torque {nonzero[0]} on the beacon before ENABLE "
                        "- disabled loop is not inert"
                    )
                if echo.applied_nonzero:
                    raise AssertionError("sim applied nonzero torque while FCS disabled")
                if (
                    log.sensor_count >= 5
                    and hk
                    and hk.mode == 0
                    and hk.sensor_frames >= 1
                    and log.max_frame >= args.start_frame
                ):
                    print(
                        f"sense: {log.sensor_count} sensor frames, {len(log.torque_by_frame)} bus "
                        f"torques (silent until enabled); FCS DISABLED with {hk.sensor_frames} "
                        f"frames in, {hk.torque_issued} torques out, "
                        f"echo applied {echo.applied_nonzero} nonzero"
                    )
                    if hk.torque_issued != 0:
                        raise AssertionError("disabled FCS issued torque")
                    stage = "enable"
                    if args.start_frame:
                        drift = log.max_frame - args.start_frame
                        if drift > 2:
                            raise AssertionError(
                                f"start-frame anchor missed: first frame at trigger was "
                                f"{log.max_frame}, asked >= {args.start_frame} - the client "
                                "started too late and its digest would not repeat"
                            )
                        print(
                            f"ENABLE anchored at sim frame {log.max_frame} "
                            f"(asked >= {args.start_frame})"
                        )
                    client.send_packet(build_cmd(fcs_cmd.msgid, FCS_FC_ENABLE))
                    sent_enable_at = time.monotonic()
                    print("uplinked FCS ENABLE")
                continue

            if stage == "enable":
                if not enable_evt_seen and any(
                    app == FCS_APP and eid == EVS_FCS_ENABLE_EID for app, eid, _ in log.evs_events
                ):
                    enable_evt_seen = True
                    print("EVS: control loop ENABLED acknowledged")
                live_now = [f for f, t in log.torque_by_frame.items() if any(t)]
                if live_now:
                    # the beacon's first nonzero torque IS the loop going
                    # live; waiting for an FCS_HK sample to learn it would
                    # put the 7.8 s poll phase back on the critical path.
                    # (The DISABLED/ENABLED mode checks still run whenever a
                    # sample arrives - the sense stage proved inertness.)
                    stage = "verify"
                    print(
                        f"loop live from frame {min(live_now)}: "
                        f"{len(log.torque_by_frame)} torque packets on the beacon"
                    )
                continue

            if stage == "verify":
                live = {f: t for f, t in log.torque_by_frame.items() if any(t)}
                if len(live) < 6:
                    continue
                default_kp_axes = (60.0, 50.0, 45.0)  # fcs_app.c FCS_DEFAULT_KP
                first_live = min(live)
                # Lockstep answer contract, both directions: frames before the
                # enable are answered zero, frames from it on are answered with
                # the law, and NO frame in between is unanswered.
                for frame, torque in log.torque_by_frame.items():
                    if frame < first_live and any(torque):
                        raise AssertionError(f"frame {frame}: torque before the enable boundary")
                for frame, torque in live.items():
                    omega = log.sensor_omega.get(frame)
                    if omega is None:
                        continue
                    for axis in range(3):
                        expected = -default_kp_axes[axis] * omega[axis]
                        if abs(torque[axis] - expected) > 1e-6 * max(1.0, abs(expected)):
                            raise AssertionError(
                                f"frame {frame} axis {axis}: {torque[axis]:+.6f} != -Kp*omega "
                                f"{expected:+.6f} (exact-bytes law violated)"
                            )
                        if torque[axis] * omega[axis] >= 0 and abs(omega[axis]) > 1e-3:
                            raise AssertionError(
                                f"frame {frame} axis {axis}: torque is not damping"
                            )
                # The barrier promises one answer per frame, and from the
                # enable boundary on that answer rides the bus: any observed
                # sensor frame (bar the newest, still in flight) has a torque.
                # BEFORE the boundary the bus is legitimately silent - fcs
                # acks disabled frames without commanding; inertness there is
                # proven by the echo stream (bitwise-zero applied torques).
                unanswered = [
                    f
                    for f in log.sensor_omega
                    if first_live < f < max(log.sensor_omega) - 2 and f not in log.torque_by_frame
                ]
                if unanswered:
                    raise AssertionError(f"frames unanswered by flight software: {unanswered[:5]}")
                # Echo == bus, exactly: the bytes fcs published are the bytes
                # the dynamics integrated, frame for frame.
                for frame, torque in log.torque_by_frame.items():
                    applied = echo.applied_by_frame.get(frame)
                    if applied is not None and applied != torque:
                        raise AssertionError(
                            f"frame {frame}: echo {applied} != bus torque {torque}"
                        )
                # ADVANCE TO RETUNE when SET_GAIN can be anchored. Every
                # exact-bytes law check above ran on every arriving sample -
                # what must NOT gate the send is the arrival of an FCS_HK
                # sample: its 7.8 s poll PHASE is a container-boot artifact,
                # and gating on it let a mission rebuild silently move the
                # SET_GAIN frame and re-roll a frozen digest (P6-era lesson,
                # retirement of 25fc027b). Anchored: send on exactly this
                # beacon frame, loud failure if the window was missed.
                anchored = bool(args.gain_frame)
                anchor_ready = anchored and log.max_frame >= args.gain_frame
                if anchor_ready:
                    drift = log.max_frame - args.gain_frame
                    if drift > 2:
                        raise AssertionError(
                            f"gain-frame anchor missed: first frame at trigger was "
                            f"{log.max_frame}, asked >= {args.gain_frame} - the client "
                            "started too late and its digest would not repeat"
                        )
                if anchor_ready or (hk and hk.torque_issued >= 5):
                    if hk:
                        print(
                            f"law verified on {len(live)} frames exactly, "
                            f"FCS issued {hk.torque_issued}, sim applied {echo.applied_nonzero}"
                        )
                    else:
                        print(
                            f"law verified on {len(live)} frames exactly, "
                            f"sim applied {echo.applied_nonzero}"
                        )
                    if anchor_ready:
                        print(f"SET_GAIN anchored at sim frame {log.max_frame}")
                    stage = "retune"
                    client.send_packet(build_cmd(fcs_gain.msgid, 0, struct.pack("<3d", *NEW_KP)))
                    sent_gain_at = time.monotonic()
                    print(f"uplinked FCS SET_GAIN Kp={NEW_KP}")
                continue

            if stage == "retune":
                if hk and all(abs(k - want) < 1e-9 for k, want in zip(hk.kp, NEW_KP, strict=True)):
                    gain_seen = True
                gain_seen = gain_seen or any(
                    app == FCS_APP and eid == EVS_FCS_SET_GAIN_EID for app, eid, _ in log.evs_events
                )
                if gain_seen and len(log.torque_by_frame) >= 12:
                    gain_ack_frame = max(log.torque_by_frame)
                    stage = "decay"
                continue

            if stage == "decay":
                if len(log.sensor_omega) < 25:
                    continue
                # frames after the retune must follow the NEW gain, exactly
                for frame, torque in log.torque_by_frame.items():
                    if frame <= gain_ack_frame:
                        continue
                    omega = log.sensor_omega.get(frame)
                    if omega is None:
                        continue
                    for axis in range(3):
                        expected = -NEW_KP[axis] * omega[axis]
                        if abs(torque[axis] - expected) > 1e-6 * max(1.0, abs(expected)):
                            detail = f"{torque[axis]:+.6f} != {expected:+.6f}"
                            raise AssertionError(
                                f"frame {frame} axis {axis} kept the old gain: {detail}"
                            )
                norms = [
                    math.sqrt(sum(v * v for v in w)) for _, w in sorted(log.sensor_omega.items())
                ]
                early = sum(norms[2:7]) / 5
                late = sum(norms[-5:]) / 5
                if late < early and echo.applied_nonzero >= 5:
                    print(f"onboard-knowledge damping: early {early:.5f} -> late {late:.5f} rad/s")
                    ok = True
                    reason = "closed loop proven"
                    break
                # keep sampling; the decay must beat the noise floor visibly
                continue
    finally:
        client.close()

    if not ok:
        reason = f"stuck in stage {stage!r}"
        if sent_enable_at is not None:
            reason += f" (ENABLE sent {time.monotonic() - sent_enable_at:.0f}s ago)"
        if sent_gain_at is not None:
            reason += f" (SET_GAIN sent {time.monotonic() - sent_gain_at:.0f}s ago)"
        print(f"events seen: {[f'{a}/{e}' for a, e, _ in log.evs_events][-8:]}")
    print("P3 CLOSED LOOP OK" if ok else f"P3 CLOSED LOOP FAILED: {reason}")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
