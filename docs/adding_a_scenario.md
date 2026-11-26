# Adding a scenario

A scenario file **is** a mission: one world, its vehicles, the hardware each
carries, and — because the file is also what the flight software builds and
boots from — which onboard organs exist at all. The everyday act of working on
this project is writing a new one over known parts in known vehicles, and that
is a single file. Everything else on this page is what the file implies.

Where a sentence here disagrees with what the tooling does, **the tooling is
right and this file is a bug.**

## 1. Start from the closest plant

The plants in `sim/scenarios/` differ in exactly the dimensions a mission
differs in — copy the nearest and edit, don't write from scratch:

| Plant | Shape |
| --- | --- |
| `chief.toml` | one vehicle, the operations plant: wheeled, sensed, no eye |
| `twin_ops.toml` | two vehicles in one world, mounted differently — the heterogeneity witness |
| `camera_ops.toml` | the eye declared: a `[[camera]]` block and the part that brings it |
| `camera_record.toml`, `camera_downlink.toml` | the eye with its file planes: store-to-ground and downlink acceptance |
| `formation_eye.toml` | two vehicles where one's eye carries the other: a 100 m hover, its blob fitted against orbit geometry |

Every plant opens with the same three sections — `[simulation]` (duration, the
world tick), `[world]` (one sun), `[environment]` (one magnetic field) — and
then one `[[spacecraft]]` block per vehicle. Nothing anywhere declares a port
or a ground identity.

## 2. Names are the only identity you write

- The **block name** is the ground identity: `name = "chief"` means the COSMOS
  target `SAT_CHIEF`, the truth stream `TRUTH_CHIEF`, and the landed startup
  table `cfe_es_startup.chief.scr`. Two vehicles may never share one name.
- The **block index** is the port slot: block 0 lands on the legacy block —
  beacon `5050`, uplink `5051`, sensor farm `5580` — and block *N* on `5050+2N`
  and `5580+20N` (`sim/fleet.py` is the single authority). Reordering blocks
  therefore reassigns ports **on purpose**, and the ground maps follow or the
  pin tests refuse the build.
- The **file stem** is the yard's state name — the stem is what `status` and
  `down` speak.

## 3. Mount hardware per vehicle

A device is a `[[spacecraft.sensors]]` or `[[spacecraft.effectors]]` section
whose `type` names a row of `registry/parts.toml` and whose `name` names the
instance; the same type may be mounted many times (four sun sensors are one
part, four instances):

```toml
[[spacecraft.sensors]]
type = "starTracker"     # a registered part
name = "st"              # this vehicle's instance of it
```

Each vehicle mounts its own hardware — a heterogeneous fleet is an ordinary
input, not a special case. How many is law too: a registered part allows one
mount per vehicle unless its registry row fixes more (the sun-sensor slots
are four because the housekeeping container fixes four cosines), and a
scenario that mounts past the law is refused at world build, naming the part
and the count. A vehicle wakes exactly what its own mounts bring:
the baseline organs every vehicle carries, plus the payload authorities whose
parts it mounts (the registry row says which app a part brings; the eye's
`[[camera]]` block, with `parent = "<vehicle>"`, brings the imager app on the
vehicle that declares it). A vehicle that mounts nothing extra boots a plane
whose build never even compiled the organ it lacks — the whole chain from
mount to object file is derived ([what the scenario rules](what_the_scenario_rules.md)).

Taking a mount away is the same door in reverse, and it reaches the compiler:
the organ leaves the vehicle's wake-up table (it never boots), the build list
(its object file is never compiled into the image), and that vehicle's ground
dictionary (ground cannot command what does not exist). The bench has flown
this with the eye: the cut plane streams its beacon with no imager
housekeeping frame at all; the all-parts plane carries it.

Three boundaries on the promise:

- Organs shipped with the core — the scheduler lab and friends — ride every
  image whatever the scenario says, and the core's own packets are nobody's
  to scope away.
- An organ leaves a vehicle only when that vehicle's *last* mount naming it
  goes: four sun sensors are one app and four table rows, so deleting three
  leaves the app awake with three entries.
- The physics frames between the world and a craft stay defined whatever the
  wake list says — what disappears is the onboard organ and its traffic, not
  the message.

And the mirror of removal: dropping an organ another one still entails stops
the door with a named sentence before any compile — nothing boots short
quietly.

## 4. Fly it through the yard door

```bash
scripts/fleet.sh up sim/scenarios/my_mission.toml   # derive, build, launch, wait
scripts/fleet.sh status                             # who is up, and the world's latest line
scripts/fleet.sh down                               # stop politely; digests print at the boundary
```

`up` is the documented door because it is the only path that refuses lies. In
order, it: derives every vehicle's startup table from the file and lands it
beside the core; cuts the build list to the apps this fleet can wake (the
entailment refusal happens here — a mount that asks for a service the fleet
cannot serve stops the door with a sentence, before any compile); builds,
stamps the binary with the list it built from, and launches; then waits for
**each craft's own boot sentence** — a vehicle quietly booting the image's
fallback table is a refused boot, not a coincidence.

`down` sweeps the scenario's build fragment and reinstalls the all-parts plane:
a quiet yard is a complete yard, and the stamp always describes the tree it
sits in.

## 5. Accept it like the bench does

The rule at the yard's edge: at least one witness never comes from the file.
Listening is enough — `sim/ground_probe.py` speaks the beacon raw:

```python
from sim.ground_probe import GroundClient

client = GroundClient()  # 127.0.0.1:5050 / 5051 — the operator's own ports
```

A scenario is accepted when, on the raw beacon, its frames stream (beacon,
housekeeping, and every mounted part's row), the frames of organs it did not
declare stay silent, the world's physics digests print at `down` beside the
anchors they must not move, and `scripts/firmware.sh check` is green.
`scripts/check_startup_config.sh` is that sentence flown for the declared
organ — its `witness.py` is the pattern to borrow.

## 6. The surfaces you configure

The mission is one file. Three registry files stand behind it, each the
extension surface for exactly one kind of question:

| File | You touch it when | One row or block is |
| --- | --- | --- |
| `sim/scenarios/*.toml` | every day — this is the mission | one world, one vehicle, its mounts: the only file whose edit changes what flies |
| `registry/parts.toml` | the fleet carries a device type it never carried | the part's byte contract: the `bskforge` type, the message id, which app a mount wakes, the acceptance exhibit and its bounds — [adding a part](adding_a_sensor.md) walks it |
| `registry/apps.toml` | a craft grows a new onboard organ | the boot record: plumbing or payload, the ground dictionaries the organ owns, what it entails — [what the scenario rules](what_the_scenario_rules.md) says when that belongs in a row instead |
| `registry/coverage.toml` | classifying an asset `bskforge` exposes | landed, refused, or not wired yet, with a named first missing hop; a classification that disagrees with the parts registry fails the battery either way, and `docs/coverage.md` is this file rendered |

The tail that is not TOML, so you know what you are not forgetting: a **new
vehicle name** the yard has never launched needs its service pair in
`compose.yml` — [adding a spacecraft](adding_a_spacecraft.md) walks it; a
**new part's bytes** enter through the EDS contracts in
`cfs_mission/mission/orrery_defs/eds/`; a **new organ** is app source beside
the core; and `cfs.lock` pins the core, which is never edited to bend it.
Every other TOML in the repository is Python's own plumbing
(`pyproject.toml`, `ty.toml`), not mission configuration.

None of the registries names a scenario, and no scenario names a registry —
the generators marry the two, so the doors read the registries and every law
above holds for fleets nobody has written yet.
