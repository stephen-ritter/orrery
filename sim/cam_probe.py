"""CAM v2a ground probe: command the eye, witness both views.

The acceptance this probe drives is the pair the phase was built to prove:
CAM_HK (what the FSW COMMANDED) and BSK_LINK_CAM_STATE_TLM (what the device
CONFIRMS) must converge - and their divergence is allowed to exist only in
the flight-software refusal path, where EVS says so. Nothing here may move
physics: the digest pair (control flight vs this spam cadence, flown by
scripts/check_cam_v2a.sh) is the argument, this probe is the witness.

Two modes:

  control  send NOTHING, watch only: the eye runs as TOML built it - open,
           at scenario rate, captures rising. The control leg of the digest
           pair; its verdict is that the link is alive and nothing moved.
  spam     the v2a cadence: close + rerate after the boot stream, reopen,
           two refusals at the app (the B1 zero, a shutter of 2), one
           ghost-eye order the app MUST accept and only the sim door
           refuses (the divergence pair, on purpose), then restore.
           Verdict: every accept landed on BOTH views, every app refusal
           moved the error counter and nothing else, the v2b/v2c promises
           stayed at their honest zero.

Like every probe here it speaks the mission's own schema (build_eds), so a
field rename is a crash at startup, not a silently wrong offset.
"""

from __future__ import annotations

import argparse
import struct
import time

from codegen.eds2cosmos import build_eds
from codegen.eds_model import Packet
from sim.fcs_probe import wire_float
from sim.fleet import ports
from sim.ground_probe import CFS, MISSION_DEFS, GroundClient, build_cmd

CAM_PACKETS = ("CAM_SET_CONFIG", "CAM_HK_TLM", "BSK_LINK_CAM_STATE_TLM", "CAM_CMD")


class CamHk:
    """One CAM_HK_TLM, decoded where it matters."""

    def __init__(self, pkt: bytes, model: Packet) -> None:
        """Decode the commanded view straight off the beacon bytes."""

        def u(leaf: str) -> int:
            return model.wire_get(pkt, leaf)

        def f(leaf: str) -> float:
            return wire_float(pkt, model.byte_offset(leaf))

        self.cmds = u(".CommandCounter")
        self.errs = u(".CommandErrorCounter")
        self.shutter = u(".Shutter")
        self.camera_id = u(".CameraId")
        self.forwarded = u(".ConfigFramesForwarded")
        self.rate_cmd = f(".RenderRateCmd")
        self.recorded = u(".FramesRecorded")
        self.stored = u(".FramesStored")
        self.transferred = u(".FramesTransferred")

    def __str__(self) -> str:
        """One readable line for the acceptance log."""
        return (
            f"hk: cmds={self.cmds} errs={self.errs} camera={self.camera_id} "
            f"cmd(shutter={self.shutter} rate={self.rate_cmd:g}) fwd={self.forwarded} "
            f"promises={self.recorded}/{self.stored}/{self.transferred}"
        )


class CamState:
    """One BSK_LINK_CAM_STATE_TLM: what the device is, not what was asked."""

    def __init__(self, pkt: bytes, model: Packet) -> None:
        """Decode the confirmed device view straight off the beacon bytes."""

        def u(leaf: str) -> int:
            return model.wire_get(pkt, leaf)

        self.sim_time_s = wire_float(pkt, model.byte_offset(".SimTimeS"))
        self.frame = u(".Frame")
        self.camera_id = u(".CameraId")
        self.shutter = u(".Shutter")
        self.rate = wire_float(pkt, model.byte_offset(".RenderRateS"))
        self.captured = u(".Captured")

    def key(self) -> tuple[int, float]:
        """The state identity: transitions are (shutter, rate) pairs."""
        return (self.shutter, round(self.rate, 6))

    def __str__(self) -> str:
        """One readable line for the acceptance log."""
        return (
            f"camstate t={self.sim_time_s:6.1f}s shutter={self.shutter} "
            f"rate={self.rate:g}s captured={self.captured}"
        )


def set_config(model: Packet, seq: int, shutter: int, camera_id: int, rate_s: float) -> bytes:
    """Build a CAM_SET_CONFIG: u8 shutter, u8 camera id, 6 B pad, f64 seconds."""
    payload = struct.pack("<BB", shutter, camera_id) + b"\x00" * 6 + struct.pack("<d", rate_s)
    return build_cmd(model.msgid, 0, payload, seq=seq)


def main(argv: list[str] | None = None) -> int:
    """Drive (or just watch) one camera flight; returns process exit code."""
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--instance", type=int, default=0, help="fleet port block (D9)")
    ap.add_argument("--mode", choices=("control", "spam"), default="spam")
    ap.add_argument("--duration", type=float, default=56.0, help="wall seconds to watch")
    args = ap.parse_args(argv)
    block = ports(args.instance)

    eds = build_eds(CFS, MISSION_DEFS)
    pkts = {p.name: p for p in eds.packets()}
    missing = [name for name in CAM_PACKETS if name not in pkts]
    if missing:
        raise SystemExit(f"the schema has no {missing} - defs older than the app?")
    cfg_model, hk_model, cs_model = (
        pkts["CAM_SET_CONFIG"],
        pkts["CAM_HK_TLM"],
        pkts["BSK_LINK_CAM_STATE_TLM"],
    )

    client = GroundClient(tlm_port=block.beacon, cmd_port=block.uplink)
    print(
        f"connected; CAM_SET_CONFIG=0x{cfg_model.msgid:04X} CAM_HK=0x{hk_model.msgid:04X} "
        f"CamState=0x{cs_model.msgid:04X} mode={args.mode}"
    )

    # cadence (spam), in wall seconds from connect; the paced sim keeps these
    # ~= sim seconds. The close waits for the boot stream to end (firstPass
    # streams 11 renders past every gate - closing before it finishes would
    # grade the shutter against a pattern it does not own). The ghost-eye
    # command is the divergence exhibit ON PURPOSE: the FSW must accept a
    # well-formed order for an eye this bird does not have - the app cannot
    # know the fit - and only the sim door refuses, so CAM_HK says camera=7
    # while CamState keeps answering camera=1. That pair, side by side, is
    # what the phase was built to show.
    sends = {
        20.0: ("accept-close", (0, 1, 4.0)),
        34.0: ("accept-reopen", (1, 1, 4.0)),
        40.0: ("refuse-zero", (1, 1, 0.0)),
        41.0: ("refuse-shutter", (2, 1, 2.0)),
        42.0: ("ghost-eye", (1, 7, 2.0)),
        45.0: ("accept-restore", (1, 1, 2.0)),
    }
    sent: list[str] = []

    t0 = time.monotonic()
    hk_seen = 0
    cs_seen = 0
    last_hk: CamHk | None = None
    last_cs: CamState | None = None
    transitions: list[CamState] = []  # distinct (shutter, rate) states, in order
    closed_window: list[tuple[float, int]] = []  # (t, captured) while shutter closed
    last_poll = 0.0
    seq = 0

    deadline = t0 + args.duration
    try:
        while time.monotonic() < deadline:
            now = time.monotonic() - t0
            if args.mode == "spam":
                for at, (label, (shutter, camera_id, rate)) in sends.items():
                    if now >= at and label not in sent:
                        client.send_packet(set_config(cfg_model, seq, shutter, camera_id, rate))
                        seq += 1
                        sent.append(label)
                        print(
                            f"t={now:5.1f}s sent CAM_SET_CONFIG shutter={shutter} "
                            f"camera={camera_id} rate={rate:g} ({label})"
                        )
                if now - last_poll >= 2.0:
                    client.send_packet(build_cmd(pkts["CAM_SEND_HK"].msgid, 0))
                    last_poll = now

            pkt = client.recv_packet(min(deadline, time.monotonic() + 0.5))
            if pkt is None:
                continue
            msgid = int.from_bytes(pkt[:2], "big")
            if msgid == hk_model.msgid:
                hk_seen += 1
                last_hk = CamHk(pkt, hk_model)
                print(f"t={now:5.1f}s {last_hk}")
            elif msgid == cs_model.msgid:
                cs_seen += 1
                state = CamState(pkt, cs_model)
                if not transitions or transitions[-1].key() != state.key():
                    transitions.append(state)
                    print(f"t={now:5.1f}s {state}")
                if state.shutter == 0:
                    closed_window.append((state.sim_time_s, state.captured))
                last_cs = state
    finally:
        client.close()

    reasons: list[str] = []
    if last_cs is None:
        reasons.append("no CamState frame ever arrived: the device view is deaf")

    if args.mode == "control":
        if cs_seen < 10:
            reasons.append(f"control leg barely heard the link: {cs_seen} CamState frames")
        if transitions and not all(key == (1, 2.0) for key in (t.key() for t in transitions)):
            reasons.append(
                f"control leg saw device transitions it never commanded: "
                f"{[str(t) for t in transitions]}"
            )
        if last_cs is not None and last_cs.captured < 10:
            reasons.append(
                f"eye never streamed: captured={last_cs.captured} after {args.duration:g}s"
            )
        verdict = "CAM V2A CONTROL OK" if not reasons else "CAM V2A CONTROL FAILED"
    else:
        if last_hk is None:
            reasons.append("no CAM_HK ever arrived: the commanded view is deaf")
        else:
            if last_hk.cmds < 4:
                reasons.append(f"accepted commands stuck at {last_hk.cmds} (expected >= 4)")
            if last_hk.errs < 2:
                reasons.append(f"refusals stuck at {last_hk.errs} (expected >= 2)")
            if last_hk.forwarded < 4:
                reasons.append(f"only {last_hk.forwarded} configs egressed (expected >= 4)")
            if last_hk.camera_id != 1:
                reasons.append(
                    f"the commanded eye ended at {last_hk.camera_id}, not 1 (restore lost)"
                )
            if (last_hk.shutter, round(last_hk.rate_cmd, 3)) != (1, 2.0):
                reasons.append(
                    f"final commanded state is {last_hk.shutter}/{last_hk.rate_cmd}, want 1/2.0"
                )
            if (last_hk.recorded, last_hk.stored, last_hk.transferred) != (0, 0, 0):
                reasons.append(
                    "a v2b/v2c counter moved before its phase: that is a lie, not a feature"
                )
        keys = [t.key() for t in transitions]
        for want, where in ((0, 4.0), (1, 4.0), (1, 2.0)):
            if (want, where) not in keys:
                reasons.append(
                    f"device never confirmed shutter={want} rate={where}: a command was lost"
                )
        if keys and keys[-1] != (1, 2.0):
            reasons.append(f"device ended at {keys[-1]}, not the restored (1, 2.0)")
        if len(closed_window) >= 2:
            delta = closed_window[-1][1] - closed_window[0][1]
            if delta > 1:
                reasons.append(
                    f"the eye captured {delta} frames while the shutter was closed "
                    "(a closed shutter that still streams is a fake)"
                )
        else:
            reasons.append("the close window never reached the device view at all")
        verdict = "CAM V2A OK" if not reasons else "CAM V2A FAILED"

    states_txt = keys if args.mode == "spam" else "watched"
    print(f"camstate frames={cs_seen} hk frames={hk_seen} states={states_txt}")
    for why in reasons:
        print(f"  reason: {why}")
    print(verdict)
    return 0 if not reasons else 1


if __name__ == "__main__":
    raise SystemExit(main())
