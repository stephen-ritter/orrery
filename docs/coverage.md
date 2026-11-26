# What we support — and what we refuse

**The question this file answers:** *does this project support a given Basilisk
asset?* Before the ledger existed, answering it meant grepping the bskforge
schema, four scenario files, `registry/parts.toml`, and the FSW app list — and
whatever you concluded was folklore by the next commit. Now it is one file with a
refusal attached: `registry/coverage.toml`, rendered here.

Two laws make the file worth reading:

- **Completeness is checked against upstream, not against a copy of upstream.**
  The check imports bskforge's live schema unions, so a part added there fails
  *this* project until we say what we do with it. That is the catalog-before-
  factory rule, turned on ourselves.
- **A gap must have a price and an owner.** Every unlanded part names the first
  missing hop (`registry-row`, `provider`, `fsw-command`, …) and carries a
  reason. `refused` is a real classification: drag and solar pressure are
  refused *on evidence* — they decay every pinned trajectory, and the digest
  ledger is the project's argument.

For how a part becomes `landed`, see
[adding_a_sensor.md](adding_a_sensor.md) — why the circuit is generated at all,
the procedure in order, the fields a row can hold, and the routing table for
hardware that is *not* a registry part. The page is generated from the ledger;
`uv run python -m sim.coverage --write` refreshes it after an edit there.

<!-- BEGIN GENERATED (registry/coverage.toml via sim/coverage.py) -->

**7 of 12 part types are supported on the bus** (6 landed, 1 as the barrier tick); **5 of 12 scenario sections are used**. Everything upstream builds; every gap below is ours. The counts are computed from `registry/coverage.toml`, and a hand-edit inside this region fails the coverage check.

## Sensors

| type | status | on the wire | first missing hop | owner |
| --- | --- | --- | --- | --- |
| `imuSensor` | barrier | BSK_LINK/SensorFrame (the tick itself) | — | — |
| `coarseSunSensor` | landed | BSK_LINK/OnboardState -> BSK_LINK_ONBOARD_STATE_TLM | — | — |
| `magnetometer` | landed | BSK_LINK/OnboardState -> BSK_LINK_ONBOARD_STATE_TLM | — | — |
| `starTracker` | landed | BSK_LINK/OnboardState + BSK_LINK/AttitudeState -> 0x08D0 | — | — |
| `gpsReceiver` | landed | BSK_LINK/NavState -> BSK_LINK_NAV_STATE_TLM | — | — |
| `camera` | landed | BSK_LINK/OpNavState -> 0x08D1 | — | — |

## Effectors

An effector's *telemetry* is knowledge and rides a row; its *actuation* is
a command path and never earns a MsgId (docs/adding_a_sensor.md, "Start
here: which kind of thing are you adding?").

| type | status | command path | first missing hop | owner |
| --- | --- | --- | --- | --- |
| `reactionWheels` | landed | COSMOS -> fcs law -> BSK_LINK/TorqueCommand -> the plant | — | — |
| `mtb` | command-path | missing: no FSW dipole law and no command frame to the plant | `fsw-command` | the control-authority round |
| `thrusters` | command-path | partial: sps schedules pulses on the bus; nothing turns them into forces world-side | `fsw-command` | the control-authority round |

## Disturbance models

Not commanded, not sensed: these change the world the plant lives in,
which is exactly why they move every pinned trajectory.

| type | status | how it enters the world | first missing hop | owner |
| --- | --- | --- | --- | --- |
| `drag` | refused | disturbance model, not a command | `decision` | the per-configuration digest round |
| `solarRadiationPressure` | refused | disturbance model, not a command | `decision` | the per-configuration digest round |
| `gravityGradient` | refused | disturbance model, not a command | `decision` | the per-configuration digest round |

## Scenario sections

| section | status | why |
| --- | --- | --- |
| `simulation` | used | Schedule, stop time, seed: the world's clock is a scenario decision, never a code constant. |
| `world` | used | Gravity and ephemeris: the trajectory every baseline pins is a world file's output. |
| `environment` | used | centered_dipole is the magnetic field the magnetometer reads - which is why that sensor's numbers mean something. |
| `spacecraft` | used | The vehicle manifest itself: every [[spacecraft]] block is a mission container, a truth target, and a slot in one world. |
| `comm_links` | unused-with-reason | No inter-satellite link is modelled; the twins are graded as one world through truth, not through each other. Constellation work is where a cross-link becomes a first-class wire. |
| `formations` | unused-with-reason | Relative-motion holding would move the anchored twin digests, and the project would lose the strongest artifact it has - a pair of trajectories that reproduce bit-for-bit. Revisit when relative motion is the mission, and re-found the pair deliberately when it does. |
| `logging` | unused-with-reason | The harness records its own ledgers (trajectory digests, wire audits, truth gateways); bskforge's recorders would be a second, less audited copy of the same story. |
| `visualization` | unused-with-reason | The operator's window is COSMOS, and it is graded: a viz path that no check reads is decoration, and decoration in this repo tends to become a claim. |
| `camera` | unused-with-reason | the circuit landed (row, wire, firmware table, vizard.lock-pinned renderer), but this area measures YARD usage: it flips to `used` when camera_ops.toml has flown the pair check - the LOS exhibit on the live fleet and the anchored digest printing unchanged with and without the trailing camera sections. Declared before flown: the ledger is a map, not a wish. |
| `store` | used | The recorder a store outlet writes into, sized before flight: `capacity_bytes` bounds the volume and `capacity_files` bounds the catalog's reach, because frame size follows the scene (1.6 KB of empty sky, 230 KB with a sister vehicle on the disc) and a byte-only law cannot say when its own index outgrows an honest listing. The flying scenario's rows are compiled into the plane by codegen/emit_store, landed by the yard door and swept when it goes quiet, so changing a capacity moves the firmware stamp rather than drifting from the binary that flies. What does not exist yet: the craft obeying the number - nothing on the wire evicts, refuses a frame, or alarms at a watermark. The scenario says how big the recorder is and the build carries it; the vehicle does not yet act on it. |
| `custom` | banned | Hardware is declared by naming a type, never wired in by hand. Fleets are read through bskforge's audited profile, which sim/fleet_policy.py asks for, so this is refused at load rather than by omission. The reason is mechanical: hardware no schema type names cannot have a registry row, so the fit manifest never declares it, this ledger has no union to classify it under, no defs carry it, and no exhibit could judge its numbers - its readings would be a wire nothing grades. The route for hardware upstream cannot model yet is the one gpsReceiver took: grow the type upstream, with its own tests, and land a row here afterwards. This ledger is what makes that route survivable: it names every gap a hand-wired device would have papered over, and who owns each one. |
| `monte_carlo` | unused-with-reason | Dispersion studies need per-configuration digests (a distribution is meaningless against one pinned trajectory), so this is the per-configuration digest round's customer rather than a gap today. |

## The reasons, in full

- **`imuSensor`** — barrier. Rate knowledge is the lockstep tick, not an observability part: it is the frame every control law is frame-exact against. Giving it a registry row would put the barrier on the same footing as a payload instrument (sim/lockstep.py keeps SENSOR/ACK/CMD hardcoded on purpose).
- **`coarseSunSensor`** — landed. Four sun sensors, cosines in TOML order, inside the housekeeping bundle row.
- **`magnetometer`** — landed. Measured field body-frame, in the bundle row. Its actuator counterpart (mtb) is still command-path, so we sense the field without yet pushing on it.
- **`starTracker`** — landed. the device's own quaternion (scalar-first, C2EP), compared to truth's MRP as the representation-free angle gap at tick-exact epochs. Also the registry's own fan-out specimen - one TOML line, two frames.
- **`gpsReceiver`** — landed. a modelled receiver whose declared sigmas ARE the acceptance bounds, read out of the manifest that built the plant. The first part where the fleet's physics and the check's rubric share one source.
- **`camera`** — landed. Vizard paints the frame, the Basilisk camera module receives it, and the limb/horizon estimators turn pixels into a planet vector - the OPNAV frame carries that KNOWLEDGE (LOS + verdict flags), never the image. The exhibit grades the LOS against truth rotated into body, bounded by the declared FOV; the renderer itself is pinned by vizard.lock and mounts as a compose profile. When the world carries a second vehicle the same capture PAINTS it: scripts/check_formation_eye.sh flies a 100 m hover and fits the blob's crawl to the orbit law. Measured scale law - the default model renders TRUE-size (~2 m, legible 50-500 m, sub-pixel by ~1 km); named model_key meshes are authored for the planet view and render kilometers across up close, so they are forbidden in formation plants.
- **`reactionWheels`** — landed. The closed loop the project is built on: FSW computes, the world applies, the digest records. Every other effector wants this same shape.
- **`mtb`** — command-path · owner: **the control-authority round**. The magnetometer already streams the field, so sensing is done; what is missing is the torque rod and the law that commands it. Momentum management is attitude *supervision*, which is the safe-mode study - declaring the effector before the law exists would give the fleet a coil nobody drives.
- **`thrusters`** — command-path · owner: **the control-authority round**. The cheapest-looking gap here: the SPS app exists and already uploads and runs a schedule, but the pulse terminates at the bus. Closing it needs a force path (and then pulse-width physics inside a 1 Hz lockstep tick, which is the actual hard part - a thruster wants a faster tick than attitude does).
- **`drag`** — refused · owner: **the per-configuration digest round**. Not refused on principle - refused on evidence. Drag decays every pinned trajectory, and the project's whole argument is a digest ledger that means something; until per-configuration baselines exist, adding atmospheric decay would trade the audit for realism. Revisit when digests are keyed to sha256(TOML + registry + app list).
- **`solarRadiationPressure`** — refused · owner: **the per-configuration digest round**. Same ruling as drag, same reason: a real orbit-secularizing disturbance lands with per-configuration baselines, not before them.
- **`gravityGradient`** — refused · owner: **the per-configuration digest round**. Gravity-gradient torque on an extended body would change the attitude trajectory the ledger pins - and unlike drag it is a TORQUE, so it lands in the control loop rather than beside it.

## What the statuses mean

| status | meaning |
| --- | --- |
| `landed` | a landed row in `registry/parts.toml`, and the circuit flies |
| `barrier` | crosses as the sim's own tick; deliberately never a row |
| `catalog-unlanded` | bskforge builds it, we have no row and no wire yet |
| `command-path` | it actuates, so it lands through `fcs`/`sps`, not a MsgId |
| `refused` | a decision with a reason, not a gap |
| `banned` (sections) | refused at load, by profile rather than by omission |

## Reading this file

- **Completeness is enforced.** A new sensor, effector or disturbance type
  upstream, or a new scenario section, fails our build until it appears here.
  The ledger is checked against bskforge's live schema unions, never a copy.
- **`landed` is bidirectional.** A row that lands without a classification, and
  a classification whose row has gone, both fail the offline checks.
- **`banned` has teeth.** Every fleet here is read through bskforge's audited
  profile (`sim/fleet_policy.py` asks for it), so a file that wires hardware in
  by hand is refused at load, before validation; the policy tests prove the
  refusal fires, that no unaudited door exists, and that our fleets pass.
- **Every gap costs something specific.** Each unlanded part names its first
  missing hop from a fixed vocabulary and carries a reason; "eventually" is
  not a classification, and the offline checks say so.
- **Upstream is not the constraint.** bskforge ships a builder for every one of
  the 12 types listed here, so not one gap above is "Basilisk cannot".
  That is why this ledger lives beside the parts registry: the parts exist,
  and what remains is what this project has decided not to wire yet.
<!-- END GENERATED -->
