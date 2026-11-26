"""CAM ground probe: command the eye, witness both views, watch the store.

The acceptance this probe drives is the pair the phase was built to prove:
CAM_HK (what the FSW COMMANDED) and BSK_LINK_CAM_STATE_TLM (what the device
CONFIRMS) must converge - and their divergence is allowed to exist only in
the flight-software refusal path, where EVS says so. Nothing here may move
physics: the digest pair (control flight vs this spam cadence, flown by
scripts/check_cam_command.sh) is the argument, this probe is the witness.

Three modes:

  control  send NOTHING, watch only: the eye runs as TOML built it - open,
           at scenario rate, captures rising. The control leg of the digest
           pair; its verdict is that the link is alive and nothing moved.
  spam     the command cadence: close + rerate after the boot stream, reopen,
           two refusals at the app (the B1 zero, a shutter of 2), one
           ghost-eye order the app MUST accept and only the sim door
           refuses (the divergence pair, on purpose), then restore.
           Verdict: every accept landed on BOTH views, every app refusal
           moved the error counter and nothing else, the record/transfer promises
           stayed at their honest zero (there: no record_rate in TOML).
  record   the store outlet, flown by scripts/check_cam_record.sh: no
           uplink at all. The four-number report, CamState's RECORDED
           tally, and CamStorage's STORED ledger must fill up unprompted
           - while FramesTransferred stays zero until the ground asks for a file.

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

CAM_PACKETS = (
    "CAM_SET_CONFIG",
    "CAM_HK_TLM",
    "BSK_LINK_CAM_STATE_TLM",
    "BSK_LINK_CAM_STORAGE_TLM",
    "CAM_CMD",
)


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
        self.frames_recorded = u(".FramesRecorded")  # store outlet sends

    def key(self) -> tuple[int, float]:
        """The state identity: transitions are (shutter, rate) pairs."""
        return (self.shutter, round(self.rate, 6))

    def __str__(self) -> str:
        """One readable line for the acceptance log."""
        return (
            f"camstate t={self.sim_time_s:6.1f}s shutter={self.shutter} "
            f"rate={self.rate:g}s captured={self.captured} recorded={self.frames_recorded}"
        )


class CamStorage:
    """One BSK_LINK_CAM_STORAGE_TLM: bsk_link's /cf/img ledger."""

    def __init__(self, pkt: bytes, model: Packet) -> None:
        """Decode the store ledger off the beacon bytes."""
        self.sim_time_s = wire_float(pkt, model.byte_offset(".SimTimeS"))
        self.files_stored = model.wire_get(pkt, ".FilesStored")
        self.bytes_stored = model.wire_get(pkt, ".BytesStored")
        self.capacity_bytes = model.wire_get(pkt, ".CapacityBytes")
        self.capacity_files = model.wire_get(pkt, ".CapacityFiles")
        self.files_now = model.wire_get(pkt, ".FilesNow")
        self.used_bytes = model.wire_get(pkt, ".UsedBytes")
        self.generation = model.wire_get(pkt, ".Generation")
        self.files_evicted = model.wire_get(pkt, ".FilesEvicted")
        self.captures_lost = model.wire_get(pkt, ".CapturesLost")
        self.transfers_acked = model.wire_get(pkt, ".TransfersAcked")
        self.state = model.wire_get(pkt, ".State")

    def __str__(self) -> str:
        """One readable line for the acceptance log."""
        return (
            f"storage t={self.sim_time_s:6.1f}s files={self.files_stored} bytes={self.bytes_stored}"
        )

    def recorder_str(self) -> str:
        """The recorder half on one line: the bound and what did about it."""
        return (
            f"capacity={self.capacity_bytes}B/{self.capacity_files} files "
            f"now={self.files_now}/{self.capacity_files} used={self.used_bytes}B "
            f"gen={self.generation} evicted={self.files_evicted} "
            f"lost={self.captures_lost} acked={self.transfers_acked} state={self.state}"
        )


def set_config(model: Packet, seq: int, shutter: int, camera_id: int, rate_s: float) -> bytes:
    """Build a CAM_SET_CONFIG: u8 shutter, u8 camera id, 6 B pad, f64 seconds."""
    payload = struct.pack("<BB", shutter, camera_id) + b"\x00" * 6 + struct.pack("<d", rate_s)
    return build_cmd(model.msgid, 0, payload, seq=seq)


def main(argv: list[str] | None = None) -> int:
    """Drive (or just watch) one camera flight; returns process exit code."""
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--instance", type=int, default=0, help="fleet port block")
    ap.add_argument(
        "--mode", choices=("control", "spam", "record", "twins", "store"), default="spam"
    )
    ap.add_argument("--duration", type=float, default=56.0, help="wall seconds to watch")
    args = ap.parse_args(argv)
    block = ports(args.instance)

    eds = build_eds(CFS, MISSION_DEFS)
    pkts = {p.name: p for p in eds.packets()}
    missing = [name for name in CAM_PACKETS if name not in pkts]
    if missing:
        raise SystemExit(f"the schema has no {missing} - defs older than the app?")
    cfg_model, hk_model, cs_model, store_model = (
        pkts["CAM_SET_CONFIG"],
        pkts["CAM_HK_TLM"],
        pkts["BSK_LINK_CAM_STATE_TLM"],
        pkts["BSK_LINK_CAM_STORAGE_TLM"],
    )

    client = GroundClient(tlm_port=block.beacon, cmd_port=block.uplink)
    print(
        f"connected; CAM_SET_CONFIG=0x{cfg_model.msgid:04X} CAM_HK=0x{hk_model.msgid:04X} "
        f"CamState=0x{cs_model.msgid:04X} CamStorage=0x{store_model.msgid:04X} mode={args.mode}"
    )

    # cadence (spam), in wall seconds from connect; the paced sim keeps these
    # ~= sim seconds. The close waits for the boot stream to end (firstPass
    # streams 11 renders past every check - closing before it finishes would
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
    # The two-eyed script: every order is well-formed (no refusals this leg -
    # refusals are the command leg's exhibit), each eye gets its own cadence,
    # and the closing/reopening targets eye 2 while eye 1 holds a DIFFERENT
    # commanded pair: the audit then screens that eye 1 never blinked. One
    # commanded pair per vehicle stays the law; the leg proves the pair
    # follows the named eye, not the other way round.
    sends_twins = {
        18.0: ("eye2-open", (1, 2, 3.0)),
        28.0: ("eye1-fast", (1, 1, 2.5)),
        38.0: ("eye2-close", (0, 2, 3.0)),
        46.0: ("eye2-reopen", (1, 2, 3.0)),
    }
    if args.mode == "twins":
        sends = sends_twins
    sent: list[str] = []

    t0 = time.monotonic()
    hk_seen = 0
    cs_seen = 0
    last_hk: CamHk | None = None
    last_cs: CamState | None = None
    last_store: CamStorage | None = None
    store_seen = 0
    transitions: list[CamState] = []  # distinct (shutter, rate) states, in order
    by_id: dict[int, CamState] = {}  # latest confirmed view per eye (twins)
    trans_by_id: dict[int, list[CamState]] = {}  # per-eye transitions (twins)
    closed_window: list[tuple[float, int]] = []  # (t, captured) while shutter closed
    last_poll = 0.0
    seq = 0

    deadline = t0 + args.duration
    try:
        while time.monotonic() < deadline:
            now = time.monotonic() - t0
            if args.mode in ("spam", "twins"):
                for at, (label, (shutter, camera_id, rate)) in sends.items():
                    if now >= at and label not in sent:
                        client.send_packet(set_config(cfg_model, seq, shutter, camera_id, rate))
                        seq += 1
                        sent.append(label)
                        print(
                            f"t={now:5.1f}s sent CAM_SET_CONFIG shutter={shutter} "
                            f"camera={camera_id} rate={rate:g} ({label})"
                        )
            if args.mode in ("spam", "record", "twins", "store") and now - last_poll >= 2.0:
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
                if args.mode == "twins":
                    # Per-eye screens: a two-eyed leg reads the device view
                    # BY EYE - the id in the frame is the authority on which
                    # instrument a transition belongs to.
                    by_id[state.camera_id] = state
                    per_eye = trans_by_id.setdefault(state.camera_id, [])
                    if not per_eye or per_eye[-1].key() != state.key():
                        per_eye.append(state)
                        print(f"t={now:5.1f}s cam{state.camera_id}: {state}")
                if not transitions or transitions[-1].key() != state.key():
                    transitions.append(state)
                    if args.mode != "twins":
                        print(f"t={now:5.1f}s {state}")
                if state.shutter == 0:
                    closed_window.append((state.sim_time_s, state.captured))
                last_cs = state
            elif msgid == store_model.msgid:
                store_seen += 1
                ledger = CamStorage(pkt, store_model)
                if last_store is None or (
                    ledger.files_stored,
                    ledger.captures_lost,
                    ledger.files_evicted,
                ) != (last_store.files_stored, last_store.captures_lost, last_store.files_evicted):
                    print(f"t={now:5.1f}s {ledger}")
                last_store = ledger
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
        verdict = "CAM CONTROL OK" if not reasons else "CAM CONTROL FAILED"
    elif args.mode == "record":
        # The eye is a payload. Nobody commands anything here - the
        # outlet lives in TOML (record_rate) and its whole life must be
        # visible without a single uplink: sim sends (CamState.recorded),
        # bsk_link files (CamStorage), and the four-number HK repeating both.
        keys = []
        if last_store is None:
            reasons.append("no CamStorage frame ever arrived: the ledger is deaf")
        if last_hk is None:
            reasons.append("no CAM_HK ever arrived: the four-number report is deaf")
        if last_cs is not None and last_cs.frames_recorded < 1:
            reasons.append("the store outlet never sent a capture (recorded stayed 0)")
        if last_store is not None:
            if last_store.files_stored < 1:
                reasons.append("no image file completed under /cf/img")
            if last_store.bytes_stored < 1:
                reasons.append("the ledger counted a file with no bytes")
        if last_hk is not None:
            if last_hk.recorded < 1:
                reasons.append(f"CAM_HK recorded stuck at {last_hk.recorded}")
            if last_store is not None and last_hk.stored != last_store.files_stored:
                reasons.append(
                    f"CAM_HK stored={last_hk.stored} disagrees with the ledger "
                    f"{last_store.files_stored}"
                )
            if last_hk.transferred != 0:
                reasons.append(
                    "FramesTransferred moved without a transfer: that is a lie, not a feature"
                )
        verdict = "CAM RECORD OK" if not reasons else "CAM RECORD FAILED"
    elif args.mode == "store":
        # The recorder law, read off the wire. The bench scenario sizes a
        # tiny recorder (a few frames of empty sky), so the flight must show
        # the whole life: declared size arrived (never state 3), files landed,
        # the watermark passed, the bound refused - and the loud absence:
        # nothing is requested on this bench, so no receipt was ever PROVEN,
        # nothing was evictable, and the eviction counter stays zero. A store
        # that recycles unproven bytes fails this leg by name.
        if last_store is None:
            reasons.append("no CamStorage frame ever arrived: the ledger is deaf")
        else:
            print(f"final recorder: {last_store.recorder_str()}")
            if last_store.state == 3:
                reasons.append(
                    "state 3: the craft ran unclaimed - the scenario's size "
                    "never reached the binary"
                )
            if last_store.capacity_bytes <= 0 or last_store.capacity_files <= 0:
                reasons.append(
                    f"capacity {last_store.capacity_bytes}B/{last_store.capacity_files}: "
                    "the declared recorder did not arrive on the wire"
                )
            if last_store.files_stored < 1:
                reasons.append(
                    "not one file landed: the outlet was closed, the law was never tested"
                )
            if last_store.captures_lost < 1:
                reasons.append(
                    f"a recorder bounded at {last_store.capacity_files} files refused nothing "
                    "in the whole flight: the bound is decoration"
                )
            if last_store.files_now > last_store.capacity_files:
                reasons.append(
                    f"files_now={last_store.files_now} exceeds the declared cap "
                    f"{last_store.capacity_files}"
                )
            if last_store.files_evicted != 0:
                reasons.append(
                    "bytes were recycled with no proven transfer on this bus: the law broke"
                )
            if last_store.transfers_acked != 0:
                reasons.append(
                    "a class 1 completion was counted as receipt: 'sent' is not 'received'"
                )
            if last_store.state != 2:
                reasons.append(
                    f"state ended at {last_store.state}, want 2: a refused recorder reports FULL"
                )
            if last_hk is None:
                reasons.append(
                    "no CAM_HK ever arrived: no housekeeping to cross-check the ledger against"
                )
            elif last_store is not None:
                if last_hk.stored != last_store.files_stored:
                    reasons.append(
                        f"CAM_HK stored={last_hk.stored} disagrees with the ledger "
                        f"{last_store.files_stored}"
                    )
                if last_hk.transferred != 0:
                    reasons.append(
                        "FramesTransferred moved with no request flown: a lie, not a feature"
                    )
            if last_cs is not None:
                # Refusals are a SUBSET of the recorded-vs-stored divergence:
                # the airgap can eat frames too. More refusals than divergence
                # would mean the same loss counted twice, or a counter inventing
                # captures nobody sent.
                diverged = last_cs.frames_recorded - last_store.files_stored
                if last_store.captures_lost > diverged:
                    reasons.append(
                        f"CapturesLost={last_store.captures_lost} exceeds the recorded-vs-stored "
                        f"divergence {diverged}: the loss is counted twice"
                    )
        verdict = "CAM STORE OK" if not reasons else "CAM STORE FAILED"
    elif args.mode == "twins":
        # The two-eyed law, screened from the ground: both eyes confirm
        # themselves independently, the commanded pair follows the NAMED
        # eye, and a command aimed at one eye never edits the other.
        if not {1, 2} <= set(by_id):
            reasons.append(f"the device view never showed both eyes: ids={sorted(by_id)}")
        for eye_id, state in sorted(by_id.items()):
            if state.captured < 1:
                reasons.append(f"eye {eye_id} never streamed: captured={state.captured}")
        eye1, eye2 = by_id.get(1), by_id.get(2)
        if eye1 is not None and eye1.key() != (1, 2.5):
            reasons.append(
                f"eye 1 ended at {eye1.key()}, want (1, 2.5): the fast cadence did not stick"
            )
        if eye2 is not None and eye2.key() != (1, 3.0):
            reasons.append(f"eye 2 ended at {eye2.key()}, want (1, 3.0) after the reopen")
        if any(state.key()[0] == 0 for state in trans_by_id.get(1, [])):
            reasons.append("eye 1's shutter closed: a command aimed at eye 2 edited eye 1")
        if last_hk is None:
            reasons.append("no CAM_HK ever arrived: the commanded view is deaf")
        else:
            if last_hk.errs != 0:
                reasons.append(f"the twins leg refused {last_hk.errs} well-formed orders")
            if last_hk.camera_id != 2:
                reasons.append(f"the commanded eye ended at {last_hk.camera_id}, want 2")
            # Per-eye divergence audit, the twins leg's own screen: CAM_HK is the
            # commander's word for ONE eye; that eye's CamState is the
            # device's word. They must agree exactly - for the named eye.
            if eye2 is not None and (last_hk.shutter, round(last_hk.rate_cmd, 3)) != eye2.key():
                reasons.append(
                    f"per-eye divergence: CAM_HK commands {last_hk.shutter}/"
                    f"{last_hk.rate_cmd:g} while eye 2 confirms {eye2.key()}"
                )
        verdict = "CAM TWINS OK" if not reasons else "CAM TWINS FAILED"
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
                    "a witness counter moved without its mechanism: that is a lie, not a feature"
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
        verdict = "CAM COMMAND OK" if not reasons else "CAM COMMAND FAILED"

    states_txt = keys if args.mode == "spam" else "watched"
    print(f"camstate frames={cs_seen} hk frames={hk_seen} states={states_txt}")
    for why in reasons:
        print(f"  reason: {why}")
    print(verdict)
    return 0 if not reasons else 1


if __name__ == "__main__":
    raise SystemExit(main())
