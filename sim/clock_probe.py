"""Clock-slaving acceptance: the wake-time skew every frame consumer inherits.

Requires the running mission (gs_tcp :5050/:5051) and the chief plant flying
in lockstep (``forge_scenario --zmq``, paced or unpaced). The link law lives
in bsk_link: every sensor frame slaves the mission clock (SET_MET) BEFORE the
frame publishes, so the TIME task is queued ahead of every consumer the frame
wakes. The OBC's housekeeping carries both sides of that claim from a single
wake - the frame's own ``SimTimeS`` and the mission clock as ``CFE_TIME``
reads it in that frame's wake - so the ground subtracts and holds every
sampled wake to account.

The verdict law is pace-agnostic: |clock - frame time| must stay under half
a control tick. Ordering honoured measures milliseconds (microsecond command
quantization plus task dispatch); an ordering regressed to sync-after-publish
measures a whole tick (~1.0 s) on EVERY wake, and catching that regression
is precisely what this bench exists for - the stored-command evaluator of
the SC arc decides on this same wake class, and it inherits exactly this
skew.

Nothing here predicts a value: the probe only samples what the craft reports
and compares two quantities the craft itself reported, one bus packet at a
time.
"""

from __future__ import annotations

import argparse
import struct
import time

from codegen.eds2cosmos import build_eds
from sim.fleet import ports
from sim.ground_probe import CFS, MISSION_DEFS, GroundClient

SKEW_BOUND_S = 0.25  # half the 1 s control tick: the law's own margin
MIN_SAMPLES = 8  # wakes enough to make "every wake" mean something
WARMUP_S = 5.0  # let boot-phase wakes settle before any sample counts


def wire_float(pkt: bytes, offset: int) -> float:
    """Payload float64 at a model offset: payload bytes are target-native LE."""
    return float(struct.unpack_from("<d", pkt, offset)[0])


def main(argv: list[str] | None = None) -> int:
    """Sample OBC housekeeping and verdict the wake-time skew; exit code = verdict."""
    ap = argparse.ArgumentParser(description="clock-slaving acceptance client")
    ap.add_argument("--instance", type=int, default=0, help="fleet port block to address")
    ap.add_argument("--samples", type=int, default=MIN_SAMPLES, help="wakes to verdict")
    ap.add_argument("--deadline", type=float, default=120.0, help="wall seconds to collect")
    args = ap.parse_args(argv)

    eds = build_eds(CFS, MISSION_DEFS)
    pkts = {p.name: p for p in eds.packets()}
    obc = pkts["OBC_HK_TLM"]
    evs = pkts["CFE_EVS_LONG_EVENT_MSG"]
    evs_app_off = evs.byte_offset(".AppName")
    evs_msg_off = evs.byte_offset(".Message")

    block = ports(args.instance)
    client = GroundClient(tlm_port=block.beacon, cmd_port=block.uplink)

    print(f"connected; OBC_HK_TLM=0x{obc.msgid:04X}, bound |skew| < {SKEW_BOUND_S:.3f} s")

    skews: list[float] = []
    stamp_seen = False
    t_start = time.monotonic()
    deadline = t_start + args.deadline

    while time.monotonic() < deadline and len(skews) < args.samples:
        pkt = client.recv_packet(min(deadline, time.monotonic() + 0.5))
        if pkt is None:
            continue
        msgid = int.from_bytes(pkt[:2], "big")
        if msgid == evs.msgid:
            app = pkt[evs_app_off : evs_app_off + 20].split(b"\x00")[0].decode(errors="replace")
            text = pkt[evs_msg_off:].split(b"\x00")[0].decode(errors="replace")
            if app == "CFE_TIME" and "Set MET" in text and not stamp_seen:
                stamp_seen = True
                print(f"slaving law spoke: {text}")
        elif msgid == obc.msgid:
            frame_t = wire_float(pkt, obc.byte_offset(".LastSensorSimTimeS"))
            if frame_t < WARMUP_S:
                continue  # boot-phase wakes: TIME warm-up is not the law's business
            clock_t = wire_float(pkt, obc.byte_offset(".MissionClockS"))
            skew = clock_t - frame_t
            skews.append(skew)
            print(f"wake at frame t={frame_t:9.3f} s: clock={clock_t:9.3f} s skew={skew:+.6f} s")

    if len(skews) < args.samples:
        print(f"CLOCK FAILED: only {len(skews)} sampled wakes in {args.deadline:.0f} s")
        return 1
    if not stamp_seen:
        print(
            "CLOCK FAILED: the slaving law never spoke (no CFE_TIME 'Set MET' event on the beacon)"
        )
        return 1

    worst = max(skews, key=abs)
    wall = time.monotonic() - t_start
    print(
        f"{len(skews)} wakes sampled in {wall:.1f} s wall: "
        f"skew min {min(skews):+.6f} max {max(skews):+.6f} worst {worst:+.6f} s"
    )
    if abs(worst) >= SKEW_BOUND_S:
        print(
            f"CLOCK FAILED: a wake disagreed with its frame by {abs(worst):.3f} s "
            f"(>= half tick): the clock is not being slaved before the frame wakes consumers"
        )
        return 1
    print("CLOCK OK: the mission clock is the sensor feed's, every sampled wake")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
