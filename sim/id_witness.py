"""Raw-port tally of streamed instance ids: a witness that no verdict writes.

The boot handshake compares two masks inside the spacecraft, and its verdict
appears on the console. A console line that names an id is the FLIGHT's word;
this tool is the GROUND's independent copy of the same fact - it dials the
world's sensor PUB directly, tallies the id field of every frame of one
registry row, and prints what it saw. A claim that shows in the manifest and
here agrees with the verdict; a claim that shows ONLY there is the verdict
hallucinating. Same rule as every acceptance leg here: at least one witness
is not derived from the file under test.

The id field is read through the mission's own EDS database, by container and
field name from the registry row - no hardcoded offsets, so a schema rename
moves this witness with the schema or it crashes at startup.

usage: python -m sim.id_witness --row camstate [--seconds 6] [--instance 0]
"""

from __future__ import annotations

import argparse
import sys
import time

import zmq

from sim.eds_link import _lib, available, entry
from sim.fleet import ports
from sim.parts import load_registry


def main(argv: list[str] | None = None) -> int:
    """Tally ids for one landed row and print ``ids=<sorted seen>``."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--row", required=True, help="registry row name (e.g. camstate)")
    parser.add_argument("--seconds", type=float, default=6.0)
    parser.add_argument("--instance", type=int, default=0, help="fleet port block to dial")
    args = parser.parse_args(argv)
    if not available():
        print("EdsLib database not built here: edslink/build.sh all", file=sys.stderr)
        return 2

    rows = {row.name: row for row in load_registry()}
    row = rows.get(args.row)
    if row is None or not row.instance_field:
        known = sorted(r.name for r in rows.values() if r.instance_field)
        print(f"{args.row}: not an id-carrying row (id rows: {known})", file=sys.stderr)
        return 2

    lib = _lib()
    socket = zmq.Context.instance().socket(zmq.SUB)
    socket.setsockopt_string(zmq.SUBSCRIBE, row.zmq_topic)
    socket.connect(f"tcp://127.0.0.1:{ports(args.instance).sensor}")
    time.sleep(args.seconds)

    frames = 0
    ids: set[int] = set()
    reads = 0
    while reads < 400:
        reads += 1
        if not socket.poll(50, zmq.POLLIN):
            continue
        parts = socket.recv_multipart()
        if len(parts) != 2 or parts[0].decode("ascii", "replace") != row.zmq_topic:
            continue
        obj = entry(row.eds_container)(lib.PackedObject(parts[1]))
        id_value = int(obj[row.instance_field]())
        frames += 1
        if 1 <= id_value <= 32:
            ids.add(id_value)
    print(f"row={row.name} frames={frames} ids={sorted(ids)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
