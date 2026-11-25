# edslink — the mission's EDS, compiled to a runtime schema

The Basilisk side speaks the mission's own message definitions instead of a second
hand-written schema. NASA's EdsLib toolchain turns the EDS XML that
`cfs_mission/cFS/tools/eds` already ships into (a) a runtime database and (b) a
CPython binding, so the simulator encodes and decodes exactly the packets the flight
software and COSMOS describe.

| Artifact | What it is |
| --- | --- |
| `dist/orrery_eds_db.so` | every package this mission is built from: cFE modules, cFS apps, cFS message headers, mission defines, and the `BSK_LINK` link containers |
| `dist/EdsLib.so` | CPython binding: `import EdsLib; EdsLib.Database("orrery")` |

## Build

```bash
uv run python -m codegen.eds_sources --write   # only when the EDS file list changes
./edslink/build.sh                             # check -> prep -> build -> install
```

Needs `cmake`, a C compiler, `make`, and Lua >= 5.3 headers (`apt install
liblua5.4-dev`, which the cFS build image already has). The database must match the
interpreter that imports it, so headers come from the project venv — override the
interpreter with `EDSLINK_PYTHON=/path/to/python`.

`build.sh` finishes by running `verify.py`, which asserts each link frame encodes to
exactly `struct.pack` bytes. `tests/test_eds_link.py` runs the same check.

## Using it from Python

The normal entry point is the thin facade in `sim/eds_link.py`, which knows where the
database lives and hands back bytes ready for ZMQ:

```python
from sim.eds_link import decode_torque_command, encode_sensor_frame

sock.send_multipart([b"SENSOR", bytes(encode_sensor_frame(12.5, (-0.01, 0.0, 0.02), 401))])
command = decode_torque_command(body)  # -> TorqueCommand(sim_time_s, torque_sum_b, frame)
```

Raw binding, if you need a container the facade does not wrap yet:

```python
import EdsLib

db = EdsLib.Database("orrery")
frame = db.Entry("BSK_LINK/SensorFrame")()
frame["SimTimeS"] = 12.5
frame["OmegaBnB"][0] = -0.01
frame["Frame"] = 401
sock.send_multipart([b"SENSOR", bytes(EdsLib.PackedObject(frame))])  # -> cFS

received = db.Entry("BSK_LINK/TorqueCommand")(EdsLib.PackedObject(body))
torque = [received["TorqueSumB"][i]() for i in range(3)]
```

`EdsLib.Database()` takes a bare name and `dlopen()`s `<name>_eds_db.so`; `build.sh`
links the binding with a `$ORIGIN` RUNPATH, so keeping the module and the database
together in `dist/` is all that is needed — no `LD_LIBRARY_PATH` (an older build
required it at interpreter startup, which is why you may still see it in scripts).

Calling a leaf returns a callable: `frame["Frame"]()` is an `int`, not the object.
`db.IsContainer` and `db.keys()` are unreliable; use `db.Entry("PKG/Container")`.

## Frames on the link

| Container | Size | Direction | Notes |
| --- | --- | --- | --- |
| `BSK_LINK/SensorFrame` | 40 B | Basilisk -> cFS | sim time, body rate (rad/s), frame counter — onboard knowledge only |
| `BSK_LINK/TorqueCommand` | 40 B | cFS -> Basilisk | sim time, body torque (N m), frame counter |
| `BSK_LINK/TruthState` | 112 B | Basilisk -> ground | ideal nav state; never routed through cFS (`BSK_TRUTH`) |

NASA's own harness for this pattern lives at
`cfs_mission/cFS/tools/eds/cfecfs/testrunner/src/core/controller_sync.py` — including
topic-to-MsgId lookup and header stamping through the companion `CFE_MissionLib`
binding. Read it before writing anything new against these bindings.

## Why the link types are little-endian

`cfs_mission/mission/orrery_defs/eds/bsk_link.xml` pins `byteOrder="littleEndian"`
on every type. The non-EDS cFE runtime puts CCSDS headers on the wire big-endian and
payload items in target-native (x86 little-endian) order, and `EdsLib` honours
whatever the XML says. Declared little-endian is therefore the only way to get

    EdsLib encode == C struct bytes == the LITTLE_ENDIAN items eds2cosmos emits

for the frames that cross the link. Measured, not assumed: with the upstream
`bigEndian` default, EdsLib byte-swaps every multi-byte payload item against live
packets from the running mission.

## What this database is *not*

Ground still decodes core-service telemetry from `codegen/eds2cosmos` output, because
that generator follows the runtime's mixed byte order (headers big-endian, payloads
little-endian), which no single `DATA_BYTE_ORDER` value expresses. Table-driven
telemetry such as `HK`'s combined packets cannot be described by a static EDS layout
at all; the generator flattens those to a block, EdsLib cannot.
