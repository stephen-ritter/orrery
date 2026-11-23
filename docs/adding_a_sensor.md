# Adding a part to the fleet

A fleet file says what a vehicle is built from. When you add a device to it, the
physics, the flight software and the ground should all follow — no hunt through
five places, no C, no hand-edited definitions. Most of that is already true, and
this file is about the part that is not: **teaching the system a part it has
never carried.**

**The model, in one paragraph.** Two documents own different halves of a part's
meaning. The fleet file (`sim/scenarios/*.toml`) is authoritative for *hardware*:
which devices a vehicle carries and what their physical parameters are. The parts
registry (`registry/parts.toml`) is authoritative for what a part *means*: its
airgap topic, its byte container, its bus identity, the flight-software app that
consumes it, and the acceptance check that must arrive with it. Everything else —
frame tables, headers, topic defines, ground definitions, operator screens, the
wire audit — is generated from those two files. Adding a part that is already
registered means editing the fleet file. Adding a part the project has never
carried means writing the row and its byte contract, and letting the generators
do the rest.

Where a sentence here disagrees with what the tooling does, **the tooling is
right and this file is a bug.**

## Start here: which kind of thing are you adding?

| What you are adding | Is this the right document? | Where it goes instead |
| --- | --- | --- |
| a sensor or modelled instrument whose reading the flight software should see | **yes** | — |
| an actuator — wheels, a torque rod, thrusters | no: that is a *command* path | the control law in flight software and its command definitions; the device itself is declared in the vehicle's `[[effector]]` block |
| a new flight-software application | no | it is compiled in: the app list in `targets.cmake` plus the ES startup table |
| the simulation's own timing — rate knowledge, the lockstep handshake, the torque frame | no, deliberately | those frames are hardcoded on purpose: a control law is only worth writing if the tick it is frame-exact against is not a configuration option |
| data that exists only on the ground, such as the navigator's truth | no, and it must never | the truth gateway. Its container carries no topic, so it has no MsgId, so no flight-software packet for it can exist |

Two rules hold that table together:

- **Knowledge flows one way.** If the world measures it, it becomes a registry
  row. If the ground commands it, it becomes a command definition. A device that
  does both — an actuator that reports its own state — is two entries: the
  actuator on the command path, and a row for whatever the flight software is
  allowed to see.
- **A registered part is not a software change.** Adding or removing a
  registered part costs the fleet file and a world restart; the boot handshake
  then reports the vehicle's new fit. Only a wire that does not exist yet costs
  the procedure below.

## Four rules you cannot work around

1. **The hardware has to be a named type in bskforge's schema.** A fleet file names
   hardware; it does not build it. Hardware that no schema type names is invisible
   to the registry row, to the fit manifest the boot handshake compares, to the
   coverage ledger, and to the acceptance gate — its readings would be a wire
   nothing grades. So a device bskforge cannot model yet is upstream work first,
   with its own tests, and a registry row here afterwards. Every fleet in this
   repository is read through bskforge's audited profile — `sim/fleet_policy.py`
   asks for it — so such a file is refused at load, before anything is built, and
   the message names the fix.
2. **Sizes are computed, never asserted.** `container_bytes` must equal what the
   EDS database computes from the XML. `bash edslink/build.sh all` prints the real
   number and refuses a row that guessed. If it disagrees with you, the XML is
   what you change.
3. **Identity is data; belief is code.** A row can install a part; it cannot make
   the flight software believe anything. Decoding, sanity checks and mode logic
   stay in C, compiled once, driven by the row. That split is what keeps the
   project's central audit possible: what the flight software believes and what
   the world did must stay independently checkable.
4. **Acceptance carries no numbers of its own.** A row stores zero sigmas. It
   names the *field* in the vehicle's device block that carries the sigma, and the
   acceptance gate reads the value out of the fleet file that built the device.
   Change the sigma and the bound moves with it, and a fitted device with a
   missing or zero sigma stops the gate by name — a bound of nothing grades a
   tautology.

## Before you write anything

Two checks, both cheap, and one question you must be able to answer.

```bash
# 1. Does bskforge already expose this hardware as a named type?
uv run python -c "
from typing import get_args
from bskforge.schema.sensors import SensorConfig
print(sorted(v.model_fields['type'].default for v in get_args(get_args(SensorConfig)[0])))"

# 2. Has this project already registered it (or refused it on purpose)?
#    docs/coverage.md is the answer, generated from registry/coverage.toml.
```

**The question: how would you know the part is lying?** That answer becomes the
row's exhibit — the comparison the acceptance gate runs against the truth wire. If
you cannot answer it, register the part as *unlanded* and stop there; landing a
part nobody can judge buys you a wire and no evidence.

## The procedure

The order is not a preference: each step's output is the next step's input. The
example part below, `altimeter`, is invented for this walkthrough — it is not in
the catalog, and `docs/coverage.md` is the catalog.

### 1. Write the row — unlanded

`registry/parts.toml`. About twenty lines, and the only hand-written place where
the part means anything.

```toml
[[part]]
name = "altimeter"
display = "radar altimeter (altitude opinion)"
kind = "opinion"
landed = false                     # register early; land when the circuit flies
bskforge_type = "radarAltimeter"   # must be a real type in bskforge's schema
plant_output = "altimeter"         # which device output this row speaks
zmq_topic = "ALTSTATE"             # the airgap channel name
eds_container = "BSK_LINK/AltitudeState"
container_bytes = 32               # whatever step 4 prints; you do not decide this
topic_offset = 209                 # next free offset; a collision fails the loader
topic_name = "BSK_LINK_ALTITUDE_STATE_TLM_TOPICID"
mid_name = "BSK_LINK_ALTITUDE_STATE_TLM"
evs_info = "BSK_LINK_ALTSTATE_INF_EID"
evs_info_id = 20                   # next free event id in bsk_link_events.h
evs_label = "altimeter altitude"
consumer = "bsk_link"              # the generic consumer: no C per part
subscribers = []                   # only if another app reads it: audited against source

[part.declared]
range_sigma = "range_noise"        # a FIELD NAME in the vehicle's device block, not a number

[part.exhibit]
kind = "truth_pair"
compare = "scalar"                 # must be a comparison the acceptance gate can execute
flight_packet = "BSK_LINK_ALTITUDE_STATE_TLM"
truth_packet = "TRUTH_STATE_TLM"
floor = 0.5                        # under floor x sigma the wire is an oracle, not a sensor
ceiling = "3sqrt3"                 # over it, two worlds share one bus
epoch = "strict"
```

`bskforge_type` (or `bskforge_types` for a bundle) answers one precise question:
**would a vehicle carrying these devices stream this container?** List the
hardware whose *readings* are in the bytes — not the hardware that happens to sit
nearby. A missing name there produces a boot verdict against a part that is
perfectly healthy.

### 2. Write the byte contract

`cfs_mission/mission/orrery_defs/eds/bsk_link.xml`: the payload container, and the
telemetry wrapper that carries it. The rules that bite:

- the cFS telemetry header is 16 bytes and the payload is `littleEndian`;
- alignment is explicit — use `<PaddingEntry>`, the model packs sequentially;
- floats go in `FloatingPoint` entries, never in integer entries with a trick;
- a byte segment is a `uint8` array, and EDS arrays flatten to `_0 … _n` items, so
  a ground `ARRAY` item must have integer elements;
- never write `--` inside an XML comment: it core-dumps the EDS tool.

### 3. Give the consumer a voice

`cfs_mission/mission/apps/bsk_link/fsw/src/bsk_link_events.h`: one `#define` with
the next free event id, matching `evs_info` / `evs_info_id` in the row. This is the
whole per-part cost on the flight-software side — the decode-and-republish
consumer is generic and already compiled.

### 4. Render identity, then measure

```bash
uv run python -m codegen.emit_frames     # frame table, mirror headers, topic defines, digest
bash edslink/build.sh all                # computes the REAL container size and checks it
```

`emit_frames` writes the row onto four generated surfaces and re-renders the
registry digest that the firmware bakes in. `edslink/build.sh` prints each
container's true size and refuses any whose packed length disagrees with its row —
put that number in `container_bytes`.

**Skip `emit_frames` after editing a row and you get the one failure that survives
the offline mirrors on a dirty tree: the mission aborts at boot** (`FIT ABORT`,
below) because the binary was built from a different catalog than the world runs.

### 5. Make the world produce it

`sim/forge_scenario.py`, in two places: a property that asks whether this plant
carries the device, a provider that reads the *device's own* output message, and
one `if` line in `plant_observers` that installs it only when the property is
true.

```python
@property
def has_plant_altimeter(self) -> bool:
    """Whether the plant fitted the device that feeds ALTSTATE."""
    return self._altimeter_sub is not None


def altimeter_state_bytes(self, sim_time_s: float, frame: int) -> bytes:
    """Pack one ``AltitudeState``: this vehicle's own device message, not the harness's numbers."""
    return encode_container(
        "BSK_LINK/AltitudeState",
        {
            "SimTimeS": float(sim_time_s),
            "Frame": int(frame),
            "AltitudeM": float(self._altimeter_sub().altitude),
        },
    )
```

```python
    if body.has_plant_altimeter:  # plant_observers(), in registry order
        observers[PART_ROWS["altimeter"].zmq_topic] = lambda: body.altimeter_state_bytes(*clock())
```

Two properties come with that shape. `encode_container` looks its container up in
the *landed* rows and raises if the name is not there, so a provider cannot invent
a wire the firmware does not carry; and it checks the packed length against the
row, so a provider packing the wrong shape fails at the edge of the world instead
of as a decode error inside the spacecraft. Publish order is insertion order —
the world never shuffles its own publishes, because the trajectory digest hashes
the stream.

Absence is free here, and is the point: the `if` means a vehicle without the
device streams nothing, says so in the gate's own text, and reports a clean boot
with that part's bit clear. Nothing is zero-filled anywhere on this path. Note
what the property asks: whether the plant wired us a message outlet for the
device, not whether somebody remembered to set a flag — the second kind of fact
goes stale on the day it matters.

### 6. Teach the acceptance gate, if it cannot judge you yet

`compare` names a comparison the gate knows how to execute. A landed row whose
`compare` the gate cannot honestly run **stops the run and names the row** — the
exhibit is a capability, so a new one is implemented in `sim/p5_truth_check.py`
alongside the guard that decides what is executable, in the same round the part
lands. Landing before the comparison exists turns the offline gate red on purpose.

### 7. Build, then give the ground its copy

```bash
bash scripts/mission_build.sh                              # the flight binary, and its provenance stamp
bash scripts/regen_defs.sh sim/scenarios/twin_ops.toml     # ground defs, per vehicle
```

The mission build ends by stamping the binary with a content hash of its inputs;
every launcher refuses to boot an unstamped or mismatched one, so a stale build
directory cannot impersonate your tree.

Ground definitions regenerate on the spot. **Only when the plugin's file *set*
changes** — a new packet, a new screen — bump `GEM_VERSION` in
`scripts/cosmos_stack.sh` and reload the stack. A same-version load is a silent
no-op and will cost you an afternoon.

### 8. Land it, and prove the flight did not move

Set `landed = true`. Then classify it in the ledger: the part *type* has an entry
in `registry/coverage.toml` whose `status` becomes `"landed"` — and if bskforge's
schema has a type that no ledger row classifies, the coverage gate is already
red, because completeness is checked against upstream rather than against a copy
of upstream. Regenerate the generated half of `docs/coverage.md`:

```bash
uv run python -m sim.coverage --write
```

Both directions are cross-checked, so landing a row without the ledger entry — or
announcing a part in the ledger whose row never landed — fails the offline gate by
name. Then:

```bash
bash scripts/gates.sh                  # the whole ladder: mirrors, build, and the live fleet
```

Individual commands, when you want one of them:

```bash
uv run pytest -q && uv run ruff check && uv run ruff format --check && uv run ty check
uv run python -m sim.wire_audit 20                    # every stream against its own def
ORRERY_SCENARIO=sim/scenarios/twin_ops.toml uv run python -m sim.p5_truth_check
bash scripts/twin_roundtrip.sh                        # one world, two vehicles, both digests
```

A new wire is allowed to change **nothing inside the trajectory**: the pinned
digests in `tests/baselines/` must stay bit-identical, and the gates compare them
bit-exactly rather than within a tolerance. If a baseline does move, that is a
decision — re-anchor deliberately, in the open, and say so in the commit. The
exception is an actuator: closing a loop is *supposed* to move the trajectory, and
the digest moving on purpose is the evidence that it closed.

## The row, field by field

| Field | Meaning | Who checks it |
| --- | --- | --- |
| `name` / `display` / `evs_label` | identity in logs, screens and boot verdicts | operators |
| `landed` | the phase line: `false` registers a part, `true` enrolls it in every audit at once | the registry audit |
| `bskforge_type` / `bskforge_types` | which devices' readings are in these bytes | bskforge's live schema union |
| `plant_output` | which output of the plant device this row speaks | the provider |
| `zmq_topic` | the airgap channel name | the rig's topic set |
| `eds_container` / `container_bytes` | the byte contract, and its true length | `edslink` computes; the audit refuses a guess |
| `topic_offset` / `topic_name` / `mid_name` | bus identity, rendered onto the generated surfaces | mirror tests, collision-checked at load |
| `evs_info` / `evs_info_id` | the consumer's own voice on the event bus | the event-id audit |
| `consumer` / `subscribers` | which flight-software app reads the frame | audited against the app's source |
| `[part.declared]` | field *names* whose values are the acceptance bounds | read out of the fleet file at run time |
| `[part.exhibit]` | the promise the part arrives with: what is compared, against what, within what | the acceptance gate refuses what it cannot execute |

## What the part arrives with, for free

Once a row is landed and the part is fitted to a vehicle: a flight-software
telemetry packet, ground definitions and operator screens for every vehicle in the
fleet, a slot in the beacon audit that fails if the wire and the definition
disagree byte-for-byte, the acceptance exhibit bounded by the vehicle's own sigma,
and a presence verdict at boot. None of that is authored per part.

## At boot: how a missing or lying part announces itself

Every tick the world sends a short manifest: which parts its fleet file claims,
which frames it actually streamed, and the digest of the registry it was built
against. The flight software compares that with what its own binary contains and
what the bus delivers, and reports a verdict on every change. Masks are hex; each
bit is one row.

| Console line | What it means | Check first |
| --- | --- | --- |
| `FIT MATCHED: declared=… streamed=… registry=…` | all views agree | nothing — though a vehicle legitimately carrying nothing reports `declared=00000000` |
| `FIT SILENT: manifest claims <part> … bus never streamed it` | the fleet file declares the device, the world published no frame | did the provider's `if` fire? is the device's message ever written (a `BSK_WARNING … not properly initialized` in the world log is the plant telling you)? is the row landed but disabled at boot by a size disagreement, which is reported by row name? |
| `FIT DENIED: bus streams <part> … the manifest denies` | the world streams a frame its own manifest does not claim | almost always `bskforge_types`: the list must name the hardware whose readings are in the container. A deliberate injection produces this too — the world log prints the injection, so read that before believing the alarm |
| `FIT ABORT: firmware built from registry A, world runs B` | the mission is **refusing to fly**: bytes decoded under the wrong layout are not knowledge. The world stalls shortly after | you edited a row without running `emit_frames`, or ran it without rebuilding the mission, or you are booting a container from an older tree. Fix: `emit_frames` → `edslink/build.sh` → `mission_build.sh`. The offline mirrors fail in the same situation, before a fleet ever does |
| no `FIT …` line at all | the link never heard a manifest | read the world's log: it died before the first tick |
| `FIT MATCHED` once, then silence | correct | verdicts fire on transitions. A verdict repeating at 1 Hz would drown the console it reports to |

Two behaviours that look like bugs and are not. The handshake frame leads the tick,
but the barrier re-sends its rate frame every half second while a manifest arrives
once a second, so a mission may decode a tick or two before its first manifest
lands and an abort engages. And no verdict is issued during the first few
manifests: agreement measured over an empty bus is not agreement.

To reproduce a verdict deliberately — every run announces that it is injecting, so
a planted lie can never be mistaken for a broken spacecraft:

```bash
# A claim only produces a verdict if it DISAGREES with the fleet file, so for
# FIT SILENT copy the scenario and delete the device's [[spacecraft.sensors]]
# block first; concealment is the mirror image; a false digest needs no edit.
FIT_HIDE=gnss       bash scripts/fleet.sh up sim/scenarios/twin_ops.toml   # streams, manifest denies
FIT_DIGEST=deadbeef bash scripts/fleet.sh up sim/scenarios/twin_ops.toml   # stale firmware: FIT ABORT
PROBES=denied bash scripts/ob_fit_check.sh                                 # one canned probe, while iterating
```

## When it goes wrong

| Symptom | Cause | Fix |
| --- | --- | --- |
| `firmware: THE MISSION ON DISK WAS NOT BUILT FROM THIS TREE` | the provenance stamp on the binary disagrees with the sources it hashed | rebuild (`bash scripts/mission_build.sh`). `ORRERY_ALLOW_STALE=1` flies the old binary anyway and says so on every launch — for reproducing a bug against a binary you already have |
| the mission build fails with a CMake source-directory error | `cfs_mission/build.sh` was run on the host; the cache remembers the container path | `bash scripts/mission_build.sh`, which is the container form |
| a packet decodes as garbage, or the size audit fails | `container_bytes` disagrees with the XML, or the container has no explicit padding | run `bash edslink/build.sh all`, take its number, add `<PaddingEntry>` |
| the EDS tool dies while parsing your XML | `--` inside an XML comment | remove it; there is no escape from this one |
| a ground screen shows an array as nonsense | EDS arrays flatten to `_0 … _n`, and ground `ARRAY` items must be integer elements | expose the array as integer elements, or decode it as separate entries |
| a quaternion comparison is wrong by a sign or a conjugate | Basilisk's `qInrtl2Case` is **scalar-first**; the truth wire's attitude is an MRP | compare representations as a representation-free angle, never componentwise |
| a load into the ground system appears to succeed and changes nothing | loading a plugin version that is already loaded is a no-op | bump `GEM_VERSION` and reload |
| the world stops advancing and prints `LOCKSTEP STALL` | a container refused the tick — usually after `FIT ABORT` | fix the mismatch; the stall is the correct behavior, not the bug |
| telemetry is missing a packet that the flight software is sending | the host's message-queue depth truncates the bus pipes | a host precondition, printed by the fleet scripts; it costs observability, never physics |

## What the registry cannot hold

Three facts a fleet file is structurally unable to carry:

- **Wire identity is per-firmware; hardware presence is per-fleet.** A MsgId is a
  compile-time constant in the ground database and the beacon table. If the fleet
  file declared it, two fleets could disagree about one binary.
- **One device can feed several frames.** A single star tracker line can fill a
  bundled housekeeping frame *and* a dedicated attitude frame. A manifest can say
  "there is a star tracker"; it cannot say which containers that means without
  becoming a second copy of the wire schema.
- **The grading rubric is not a plant fact.** Putting acceptance bounds in the
  world file means a plant edit can loosen its own test. Bounds live with the
  registry, which is why a row names the field that carries a sigma rather than
  storing the sigma.

And one thing the registry deliberately refuses to hold: **device numbers.** Rows
carry no sigmas, so a row cannot go stale when a vehicle's hardware gets better.

## Related documents

- [coverage.md](coverage.md) — what exists, what is refused, what is missing and
  who owns each gap. Generated, and checked against bskforge's live schema, so a
  part added upstream that nobody classified fails the build here.
- [data_provenance.md](data_provenance.md) — how the telemetry wire, the truth
  wire and the command path are kept apart, and why truth never enters the
  flight software.
