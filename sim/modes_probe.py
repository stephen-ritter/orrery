"""P4b-2 acceptance: the mode supervisor flies on onboard knowledge.

Requires the running mission (gs_tcp :5050/:5051) and the chief plant
flying in lockstep (``forge_scenario --zmq --pace``: the ONBOARD frames the
supervisor feeds on only exist on the TOML plant). From a ground seat this
drives and proves, in order:

1. sense/ENABLE as P3 knows it - the loop lives on SensorFrame, unchanged;
2. uplink SET_WHEEL_LIMIT 10 -> EVS + HK acknowledge the new threshold;
3. the first frame whose knowledge shows |wheel| >= 10 trips ENABLED -> SAFE:
   no bus torque on ANY later frame, the sim echo proves the effector felt
   zero, while acks keep the lockstep gate open (HK keeps counting frames);
4. uplink ENABLE while SAFE is REFUSED - wheels still coast above the limit -
   proven by EVS and by the mode staying 2, still no torque;
5. SET_WHEEL_LIMIT 600 + ENABLE recovers: torque resumes on new frames,
   frame-exact against the default gain, echo matching the bus again.

Knowledge-driven, and provably so: every stage reads what the CONTROLLER
knows (HK WheelSpeeds / OnboardFramesReceived / WheelLimit), not what the
plant happens to do. Offsets come from the EDS model - this script cannot
disagree with the defs about the wire.
"""

from __future__ import annotations

import argparse
import re
import struct
import time
from dataclasses import dataclass

from codegen.eds2cosmos import build_eds
from sim.fcs_probe import EchoWatcher
from sim.fleet import ports
from sim.ground_probe import CFS, MISSION_DEFS, GroundClient, build_cmd

FCS_APP = "FCS"
FCS_FC_ENABLE = 1
EVS_FCS_ENABLE_EID = 3  # fcs_events.h FCS_ENABLE_INF_EID
EVS_FCS_MODE_SAFE_EID = 12  # FCS_MODE_SAFE_ERR_EID (this cFS has no WARNING)
EVS_FCS_ENABLE_REJ_EID = 13  # FCS_ENABLE_REJ_ERR_EID
EVS_FCS_SET_LIMIT_EID = 14  # FCS_SET_LIMIT_INF_EID

LIMIT_TRIP = 10.0  # rad/s: the detumble law passes it within seconds
LIMIT_RECOVER = 600.0  # rad/s: above the coast speed, below the sanity bound
DEFAULT_KP = (60.0, 50.0, 45.0)  # fcs_app.c FCS_DEFAULT_KP (never retuned here)


def wire_float(pkt: bytes, offset: int) -> float:
    """Payload float64 at a model offset: payload bytes are target-native LE."""
    return float(struct.unpack_from("<d", pkt, offset)[0])


@dataclass
class HkSnapshot:
    """Decoded fields of the most recent FCS_HK_TLM, knowledge included."""

    mode: int
    sensor_frames: int
    torque_issued: int
    onboard_frames: int
    wheel_limit: float
    wheel_speeds: tuple[float, float, float]
    sun_cosine: float


class FrameLog:
    """Captured beacon packets, decoded through the EDS model."""

    def __init__(self, pkts: dict) -> None:
        """Cache model offsets once; packets fly at 1 Hz and decoding is cheap."""
        self.sensor = pkts["BSK_LINK_SENSOR_FRAME_TLM"]
        self.torque = pkts["BSK_LINK_TORQUE_COMMAND_TLM"]
        self.hk = pkts["FCS_HK_TLM"]
        self.obc = pkts["OBC_HK_TLM"]
        self.evs = pkts["CFE_EVS_LONG_EVENT_MSG"]
        self.evs_app_off = self.evs.byte_offset(".AppName")
        self.evs_id_off = self.evs.byte_offset(".EventID")
        self.evs_msg_off = self.evs.byte_offset(".Message")
        self.sensor_frames: set[int] = set()
        self.sensor_omega: dict[int, tuple[float, float, float]] = {}
        self.torque_frames: set[int] = set()
        self.torque_by_frame: dict[int, tuple[float, float, float]] = {}
        self.last_hk: HkSnapshot | None = None
        self.last_obc: dict | None = None
        self.evs_events: list[tuple[str, int, str]] = []

    def feed(self, msgid: int, pkt: bytes) -> None:
        """Decode one beacon packet into the log."""
        if msgid == self.sensor.msgid:
            frame = int(struct.unpack_from("<Q", pkt, self.sensor.byte_offset(".Frame"))[0])
            self.sensor_frames.add(frame)
            self.sensor_omega[frame] = (
                wire_float(pkt, self.sensor.byte_offset(".OmegaBnB_0")),
                wire_float(pkt, self.sensor.byte_offset(".OmegaBnB_1")),
                wire_float(pkt, self.sensor.byte_offset(".OmegaBnB_2")),
            )
        elif msgid == self.torque.msgid:
            frame = int(struct.unpack_from("<Q", pkt, self.torque.byte_offset(".Frame"))[0])
            self.torque_frames.add(frame)
            self.torque_by_frame[frame] = (
                wire_float(pkt, self.torque.byte_offset(".TorqueSumB_0")),
                wire_float(pkt, self.torque.byte_offset(".TorqueSumB_1")),
                wire_float(pkt, self.torque.byte_offset(".TorqueSumB_2")),
            )
        elif msgid == self.hk.msgid:
            self.last_hk = HkSnapshot(
                mode=self.hk.wire_get(pkt, ".Mode"),
                sensor_frames=self.hk.wire_get(pkt, ".SensorFramesReceived"),
                torque_issued=self.hk.wire_get(pkt, ".TorqueCommandsIssued"),
                onboard_frames=self.hk.wire_get(pkt, ".OnboardFramesReceived"),
                wheel_limit=wire_float(pkt, self.hk.byte_offset(".WheelLimit")),
                wheel_speeds=(
                    wire_float(pkt, self.hk.byte_offset(".WheelSpeeds_0")),
                    wire_float(pkt, self.hk.byte_offset(".WheelSpeeds_1")),
                    wire_float(pkt, self.hk.byte_offset(".WheelSpeeds_2")),
                ),
                sun_cosine=wire_float(pkt, self.hk.byte_offset(".SunCosine")),
            )
        elif msgid == self.obc.msgid:
            # The OBC's report on itself, decoded through its own def. The
            # mode field is a MIRROR of the last FCS_HK_TLM (7.8 s cadence),
            # so consumers wait for it, never assume it.
            self.last_obc = {
                "mode": self.obc.wire_get(pkt, ".FcsMode"),
                "sensor_frames": self.obc.wire_get(pkt, ".SensorFramesSeen"),
                "onboard_frames": self.obc.wire_get(pkt, ".OnboardFramesSeen"),
                "torque_seen": self.obc.wire_get(pkt, ".TorquePacketsSeen"),
                "fcs_torques": self.obc.wire_get(pkt, ".FcsTorquesIssued"),
                "sim_time_s": wire_float(pkt, self.obc.byte_offset(".LastSensorSimTimeS")),
                "mag_norm_t": wire_float(pkt, self.obc.byte_offset(".MagFieldNormT")),
            }
        elif msgid == self.evs.msgid:
            app = (
                pkt[self.evs_app_off : self.evs_app_off + 20]
                .split(b"\x00")[0]
                .decode(errors="replace")
            )
            eid = int.from_bytes(pkt[self.evs_id_off : self.evs_id_off + 2], "little")
            text = pkt[self.evs_msg_off :].split(b"\x00")[0].decode(errors="replace")
            self.evs_events.append((app, eid, text))

    def saw(self, app: str, eid: int) -> bool:
        """True once an (app, event id) has been seen on the beacon."""
        return any(seen_app == app and seen_id == eid for seen_app, seen_id, _ in self.evs_events)

    def last_text(self, app: str, eid: int) -> str:
        """Message text of the newest matching event, for honest prints."""
        return next(
            text
            for seen_app, seen_id, text in reversed(self.evs_events)
            if seen_app == app and seen_id == eid
        )


def main(argv: list[str] | None = None) -> int:
    """Run the P4b-2 mode-supervisor acceptance; returns process exit code."""
    ap = argparse.ArgumentParser(description="P4b mode-supervisor acceptance client")
    ap.add_argument("--instance", type=int, default=0, help="fleet port block (D9) to address")
    args = ap.parse_args(argv)
    eds = build_eds(CFS, MISSION_DEFS)
    pkts = {p.name: p for p in eds.packets()}
    fcs_cmd = pkts["FCS_CMD"]
    fcs_limit = pkts["FCS_SET_WHEEL_LIMIT"]

    block = ports(args.instance)
    client = GroundClient(tlm_port=block.beacon, cmd_port=block.uplink)
    log = FrameLog(pkts)
    echo = EchoWatcher(block.echo_dial)
    print(f"connected; FCS_CMD=0x{fcs_cmd.msgid:04X} SET_WHEEL_LIMIT=0x{fcs_limit.msgid:04X}")

    ok = False
    stage = "sense"
    trip_frame = 0
    resume_after = 0
    deadline = time.monotonic() + 240.0

    try:
        while time.monotonic() < deadline:
            pkt = client.recv_packet(min(deadline, time.monotonic() + 0.5))
            echo.poll()
            if pkt is None:
                continue
            log.feed(int.from_bytes(pkt[:2], "big"), pkt)
            hk = log.last_hk

            if stage == "sense":
                if len(log.sensor_frames) >= 5 and hk and hk.mode == 0 and hk.onboard_frames >= 1:
                    wheels = " ".join(f"{v:+1.1f}" for v in hk.wheel_speeds)
                    print(
                        f"sense: {len(log.sensor_frames)} frames, {hk.onboard_frames} onboard; "
                        f"wheels [{wheels}], sun {hk.sun_cosine:.2f}"
                    )
                    client.send_packet(build_cmd(fcs_cmd.msgid, FCS_FC_ENABLE))
                    stage = "enable"
                    print("uplinked FCS ENABLE")
                continue

            if stage == "enable":
                if hk and hk.mode == 1 and log.saw(FCS_APP, EVS_FCS_ENABLE_EID):
                    client.send_packet(build_cmd(fcs_limit.msgid, 0, struct.pack("<d", LIMIT_TRIP)))
                    stage = "arm"
                    print(f"loop live; uplinked FCS SET_WHEEL_LIMIT {LIMIT_TRIP}")
                continue

            if stage == "arm":
                if (
                    hk
                    and abs(hk.wheel_limit - LIMIT_TRIP) < 1e-9
                    and log.saw(FCS_APP, EVS_FCS_SET_LIMIT_EID)
                ):
                    stage = "safe"
                    print(
                        f"EVS+HK confirm limit {hk.wheel_limit:.1f} rad/s; awaiting supervisor trip"
                    )
                continue

            if stage == "safe":
                if not (hk and hk.mode == 2 and log.saw(FCS_APP, EVS_FCS_MODE_SAFE_EID)):
                    continue
                # The EVS text carries the exact frame the supervisor tripped
                # on; HK arrival only tells us WHEN ground learned, not WHEN
                # the law stopped. Anchor the stop-proof to the machine's word.
                trip_text = log.last_text(FCS_APP, EVS_FCS_MODE_SAFE_EID)
                trip_match = re.search(r"frame=(\d+)", trip_text)
                trip_frame = int(trip_match.group(1)) if trip_match else max(log.sensor_frames)
                print(f"SAFE latched: {trip_text}")
                stage = "inert"
                continue

            if stage == "inert":
                # Let a few more frames fly, then demand the proof: no torque
                # packet at or after the trip frame on the bus, and the sim's
                # echo showing the effector genuinely felt zero.
                if max(log.sensor_frames) < trip_frame + 4:
                    continue
                late_torque = [f for f in log.torque_frames if f >= trip_frame]
                if late_torque:
                    raise AssertionError(
                        f"torque on the bus after the SAFE trip: frames {late_torque[:4]}"
                    )
                late_motion = [
                    f
                    for f, t in echo.applied_by_frame.items()
                    if f > trip_frame + 1 and any(abs(v) > 1e-9 for v in t)
                ]
                if late_motion:
                    raise AssertionError(
                        f"effector felt torque after the SAFE trip: frames {late_motion[:4]}"
                    )
                if hk and hk.mode != 2:
                    raise AssertionError("mode left SAFE without a ground command")
                client.send_packet(build_cmd(fcs_cmd.msgid, FCS_FC_ENABLE))
                stage = "refuse"
                print(
                    f"inert in SAFE: no bus torque since frame {trip_frame}, echo zero, "
                    f"gate alive ({hk.sensor_frames if hk else '?'} frames in) - ENABLE uplinked"
                )
                continue

            if stage == "refuse":
                if not log.saw(FCS_APP, EVS_FCS_ENABLE_REJ_EID):
                    continue
                if hk and hk.mode != 2:
                    raise AssertionError("ENABLE accepted while wheels sat at the limit")
                print(f"refusal proven: {log.last_text(FCS_APP, EVS_FCS_ENABLE_REJ_EID)}")
                client.send_packet(build_cmd(fcs_limit.msgid, 0, struct.pack("<d", LIMIT_RECOVER)))
                stage = "recover_limit"
                print(f"uplinked FCS SET_WHEEL_LIMIT {LIMIT_RECOVER}")
                continue

            if stage == "recover_limit":
                if hk and abs(hk.wheel_limit - LIMIT_RECOVER) < 1e-9:
                    client.send_packet(build_cmd(fcs_cmd.msgid, FCS_FC_ENABLE))
                    resume_after = max(log.sensor_frames)
                    stage = "recover_enable"
                    print("limit raised; uplinked ENABLE (expect recovery)")
                continue

            if stage == "recover_enable":
                if hk and hk.mode == 1:
                    stage = "verify"
                    print(f"ENABLED again from frame {resume_after} on; awaiting law-exact torque")
                continue

            if stage == "verify":
                live = {f: t for f, t in log.torque_by_frame.items() if f > resume_after and any(t)}
                if len(live) < 3:
                    continue
                for frame, torque in live.items():
                    omega = log.sensor_omega.get(frame)
                    if omega is None:
                        continue
                    for axis in range(3):
                        expected = -DEFAULT_KP[axis] * omega[axis]
                        if abs(torque[axis] - expected) > 1e-6 * max(1.0, abs(expected)):
                            raise AssertionError(
                                f"recovered frame {frame} axis {axis}: {torque[axis]:+.6f} "
                                f"!= -Kp*omega {expected:+.6f} (exact-bytes law violated)"
                            )
                    applied = echo.applied_by_frame.get(frame)
                    if applied is not None and applied != torque:
                        raise AssertionError(
                            f"recovered frame {frame}: echo {applied} != bus torque {torque}"
                        )
                print(f"recovery proven: {len(live)} frames torque-exact, echo matches bus")
                stage = "obc"
                continue

            if stage == "obc":
                # P4b-3 finale: the OBC must REPORT the incident it just
                # lived through. Its own counts are fresh; the mode mirror
                # rides the 7.8 s FCS_HK poll, so this waits, briefly.
                obc = log.last_obc
                if (
                    obc
                    and obc["mode"] == 1
                    and obc["torque_seen"] > 0
                    and obc["onboard_frames"] > 0
                ):
                    if obc["sensor_frames"] < len(log.sensor_frames) - 8:
                        raise AssertionError(
                            f"OBC frame count lags the bus: {obc['sensor_frames']}"
                        )
                    seen = f"{obc['sensor_frames']}/{obc['onboard_frames']}/{obc['torque_seen']}"
                    print(
                        f"OBC HK agrees: s/o/t frames {seen}, mode mirror {obc['mode']}, "
                        f"|B| {obc['mag_norm_t'] * 1e6:.2f} uT at t={obc['sim_time_s']:.0f}s"
                    )
                    ok = True
                    break
                continue
    finally:
        client.close()

    if not ok:
        recent = [f"{a}/{e}" for a, e, _ in log.evs_events][-8:]
        print(f"P4b MODES FAILED: stuck in stage {stage!r}; events {recent}")
        return 1
    print("P4b MODES OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
