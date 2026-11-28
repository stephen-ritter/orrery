#!/usr/bin/env python3
"""Check a freshly built EdsLib database: link frames must be plain C struct bytes.

This is the contract in ``cfs_mission/mission/orrery_defs/eds/bsk_link.xml``:
what the Basilisk side writes with EdsLib has to be what an FSW C struct reads, with
no per-field conversion at ``bsk_link``. Run it after ``edslink/build.sh``:

    uv run python edslink/verify.py

The binding loads the database with ``dlopen("<name>_eds_db.so")``, so the dist
directory has to be on ``LD_LIBRARY_PATH`` (build.sh does this for you).
"""

from __future__ import annotations

import struct
import sys
from pathlib import Path
from typing import Any

DIST = Path(__file__).resolve().parent / "dist"

# container name -> (struct format, values to write)
CASES: dict[str, tuple[str, dict[str, tuple[float, ...] | float | int]]] = {
    "SensorFrame": ("<4dQ", {"SimTimeS": 1.5, "OmegaBnB": (-0.01, 0.02, -0.03), "Frame": 42}),
    "TorqueCommand": (
        "<4dQ",
        {"SimTimeS": 2.25, "TorqueSumB": (0.001, -0.002, 0.003), "Frame": 7},
    ),
    # The lockstep barrier ack: 8 pad bytes between Mode and the
    # counter are the EDS PaddingEntry's zero bytes, explicitly in the format -
    # the model packs sequentially, and this test is where that stays honest.
    "StepAck": (
        "<dQb3xI",
        {"SimTimeS": 4.5, "Frame": 9, "Mode": 1, "TorquesIssued": 7},
    ),
    # The onboard observability companion: wheel feedback + instruments, all LE
    # doubles around the frame counter. 128 B is also the C twin's contract
    # (orrery_bus_msg.h) and the defs' item count (4 header + 16 payload).
    "OnboardState": (
        "<dQ14d",
        {
            "SimTimeS": 5.5,
            "Frame": 11,
            "WheelSpeeds": (123.5, -60.25, 0.5),
            "StQuatBn": (0.99, 0.01, -0.02, 0.03),
            "CssCos": (0.8, 0.1, 0.0, 0.5),
            "MagB": (-2.6e-5, 5e-6, 1.1e-5),
        },
    ),
    # The GNSS opinion. 64 B is the C twin's contract (orrery_bus_msg.h)
    # and the defs' item count (4 header + 8 payload).
    "NavState": (
        "<dQ6d",
        {
            "SimTimeS": 6.5,
            "Frame": 13,
            "RBnN": (7.0e6, -2.0e5, 1.0e3),
            "VBnN": (-0.3, 7.1, 0.2),
        },
    ),
    # The star tracker's own opinion, quaternion only. 48 B is
    # what the registry row declares and what the generic consumer checks at
    # boot; scalar-first is a convention pinned against the device elsewhere.
    "AttitudeState": (
        "<dQ4d",
        {
            "SimTimeS": 7.5,
            "Frame": 15,
            "StqBn": (0.9998, 0.01, -0.02, 0.03),
        },
    ),
    # Fit handshake: not a part and not telemetry - a declared fit in eight
    # bytes plus the registry digest the world was started with, plus the
    # instance attribution: four uint32 slots, bit n is id n+1, slot k is the
    # k-th id-carrying frame row. The hash here is a dummy on purpose: this
    # test pins the LAYOUT, identity is compared at boot between the baked-in
    # digest and the world's own.
    "FitManifest": (
        "<dQII4I",
        {
            "SimTimeS": 8.5,
            "Frame": 17,
            "DeclaredBits": 0x5,
            "RegistryHash": 0xDEADBEEF,
            "DeclaredIds0": 0x1,
            "DeclaredIds1": 0x0,
            "DeclaredIds2": 0x0,
            "DeclaredIds3": 0x0,
        },
    ),
    # The camera's confirmed view is packed by the sim through
    # this same DB and read by a hand-written C twin (cam app subscribes it),
    # so the layout needs a pin on this side too - 64 B is parts.toml's row.
    "CamState": (
        "<dQQQdQQQ",
        {
            "SimTimeS": 9.5,
            "Frame": 19,
            "CameraId": 1,
            "Shutter": 1,
            "RenderRateS": 2.0,
            "Captured": 3,
            "FramesRecorded": 1,
            "Snapshots": 2,
        },
    ),
    # The storage ledger: written by bsk_link's C struct, read on the ground via
    # the generated defs. 96 B, the orrery_bus_msg.h assert's other half: the
    # recorder half (capacity down) carries the scenario's [[store]] row and
    # what the vehicle did about the difference.
    "CamStorage": (
        "<dQQQQIIQQQQQBBBBBBBB",
        {
            "SimTimeS": 10.5,
            "Frame": 21,
            "FilesStored": 4,
            "BytesStored": 8192,
            "CapacityBytes": 67108864,
            "CapacityFiles": 512,
            "FilesNow": 4,
            "UsedBytes": 8192,
            "Generation": 5,
            "FilesEvicted": 0,
            "CapturesLost": 1,
            "TransfersAcked": 2,
            "State": 1,
            "Pad": (0, 0, 0, 0, 0, 0, 0),
        },
    ),
    "TruthState": (
        "<22d",
        {
            "SimTimeS": 3.5,
            "r_BN_N": (7e6, -2e5, 1e3),
            "v_BN_N": (-0.3, 7.1, 0.2),
            "sigma_BN": (0.01, -0.02, 0.03, 0.999),
            "omega_BN_B": (0.004, -0.005, 0.006),
            "BurnImpulseAppliedNs": 120.5,
            "BurnTicksApplied": 40,
            "BurnTicksSaturated": 1,
            "BurnId": 7,
            "FuelRemainingKg": 0.041,
            "FuelSpentKg": 0.009,
            "FuelTicksStarved": 2,
            "FuelTicksThrottled": 5,
        },
    ),
}


def _fill(obj: Any, values: dict[str, Any]) -> None:  # noqa: ANN401 - C-extension objects
    for name, value in values.items():
        if isinstance(value, tuple):
            for index, item in enumerate(value):
                obj[name][index] = item
        else:
            obj[name] = value


def main(dist: Path = DIST) -> int:
    """Encode and decode each link frame; compare with struct.pack. Returns 0 on success."""
    sys.path.insert(0, str(dist))
    # EdsLib is the C extension produced by edslink/build.sh; no stub to resolve.
    import EdsLib  # ty: ignore[import-not-found]

    db = EdsLib.Database("orrery")
    failures = 0

    for container, (fmt, values) in sorted(CASES.items()):
        frame_type = db.Entry(f"BSK_LINK/{container}")
        obj = frame_type()
        _fill(obj, values)
        encoded = bytes(EdsLib.PackedObject(obj))

        scalars: list[float | int] = []
        for value in values.values():
            scalars += list(value) if isinstance(value, tuple) else [value]
        want = struct.pack(fmt, *scalars)
        if encoded != want:
            print(f"FAIL {container}: encoded {encoded.hex(' ')} != struct.pack {want.hex(' ')}")
            failures += 1
            continue

        back = frame_type(EdsLib.PackedObject(want))
        for name, value in values.items():
            if isinstance(value, tuple):
                got: Any = [back[name][i]() for i in range(len(value))]
                ok = got == list(value)
            else:
                got = back[name]()
                ok = got == value
            if not ok:
                print(f"FAIL {container}.{name}: decoded {got!r}, wrote {value!r}")
                failures += 1
        print(f"ok   {container} ({len(encoded)} B) == struct.pack({fmt!r})")

    print("EDSLINK VERIFY OK" if failures == 0 else f"EDSLINK VERIFY FAILED ({failures})")
    return 0 if failures == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main(Path(sys.argv[1]) if len(sys.argv) > 1 else DIST))
