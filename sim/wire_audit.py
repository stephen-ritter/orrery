"""Wire audit: every packet the beacon streams must match its generated def.

The beacon subscription table and the ground definitions are both generated
from the same EDS model, so anything on the wire must be identifiable AND
byte-exact. This is the gate model-vs-runtime drift can never sneak past:
the CFDP HK lesson (CF's EDS XML does not describe the non-EDS runtime
struct, 94 B modeled vs 184 B on the wire) failed exactly this invariant and
nobody noticed until COSMOS logged it. Drift excluded the other way (def for
a packet that never streams) stays silent here by design - absence is safe.

Usage: mission container up (any way you like), then
    uv run python -m sim.wire_audit [capture-seconds]
Exit 0: every streamed MsgId matched exactly one generated def, exact length.
"""

from __future__ import annotations

import socket
import sys
import time

from codegen.eds2cosmos import build_eds, mission_packets
from sim.ground_probe import CFS, MISSION_DEFS


def main() -> int:
    """Capture the beacon and diff against the model; returns exit code."""
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 25.0
    eds = build_eds(CFS, MISSION_DEFS)
    defs = {
        p.msgid: (p.name, p.total_bytes())
        for p in mission_packets(eds.packets())
        if p.direction == "tlm"
    }

    sock = socket.create_connection(("127.0.0.1", 5050), timeout=5)
    sock.settimeout(2)
    buf = b""
    obs: dict[int, set[int]] = {}
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        try:
            buf += sock.recv(65536)
        except TimeoutError:
            continue
        while len(buf) >= 6:
            total = int.from_bytes(buf[4:6], "big") + 7
            if len(buf) < total:
                break
            msgid = int.from_bytes(buf[:2], "big")
            obs.setdefault(msgid, set()).add(total)
            buf = buf[total:]
    sock.close()

    if not obs:
        print("no packets captured - is the mission running?")
        return 1
    ok = True
    for msgid in sorted(obs):
        lengths = sorted(obs[msgid])
        claimed = defs.get(msgid)
        if claimed is None:
            print(f"0x{msgid:04X} streams but NO generated def claims it  <- FAIL")
            ok = False
        elif lengths != [claimed[1]]:
            print(f"0x{msgid:04X} {claimed[0]}: def {claimed[1]} B, wire {lengths}  <- FAIL")
            ok = False
        else:
            print(f"0x{msgid:04X} {claimed[0]:26s} {lengths[0]:4d} B  ok")
    print("WIRE AUDIT OK" if ok else "WIRE AUDIT FAILED")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
