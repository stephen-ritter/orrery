# Data provenance — the truth/knowledge boundary

This is the written rule the acceptance gate exists to enforce. Where a
sentence here and a test disagree, the test wins and this file is a bug.

## The two sources

| | Onboard knowledge | Truth |
| --- | --- | --- |
| Producer | Basilisk *sensors* (biased IMU, noisy rates) decoded by cFS `bsk_link` | Basilisk *navigator state*, straight from the plant |
| Path | sim → ZMQ link (`:5580+20N`) → cFS bus → `gs_tcp` beacon (`:5050+2N`) | sim → ZMQ truth (`:5590+20N`) → truth gateway → TCP (`:5150+2N`) |
| COSMOS targets | `SAT_*` (flight) | `TRUTH_*` (ground) |
| Packets | CCSDS, cFS-stamped, EDS/TopicId-defined | bare EDS `BSK_LINK/TruthState` under a 4-byte ground header |
| Can be commanded | yes (uplink `:5051+2N`) | **no — no command defs exist** |

```mermaid
flowchart LR
  PLANT["Basilisk plant"]
  PLANT -- "sensor messages<br/>bias + noise applied" --> BL["bsk_link → cFS bus<br/>(flight software decodes)"]
  BL --> BEACON["gs_tcp beacon `:5050+2N`<br/>out-only socket"]
  BEACON --> SAT["`SAT_*` target<br/>= knowledge"]
  GND["ground commands"] -- "uplink `:5051+2N`<br/>the only inbound door" --> BL
  PLANT -- "navigator state,<br/>untouched" --> GW["truth gateway<br/>`:5590+20N` → `:5150+2N`"]
  GW --> TRT["`TRUTH_*` target<br/>= truth"]
```

No arrow leaves the truth side toward the flight side, and none can: the
`TRUTH_*` targets ship no commands, and the beacon refuses inbound bytes.
The gate ladder proves it — an injection attempt on the truth wire is an
acceptance exhibit that asserts *refusal logged*.

## What rides the flight wire

| Kind | Packet(s) | Provenance | Audit |
| --- | --- | --- | --- |
| Rate knowledge (the barrier's witness) | `BSK_LINK_SENSOR_FRAME_TLM` | plant IMU message: bias + seeded random walk | frame-exact law, freshness, gate counters |
| Actuator/environment report (a bundle) | `BSK_LINK_ONBOARD_STATE_TLM`, `OBC_HK_TLM` | the plant's own device messages sampled per tick, mirrored by `obc_hk` | supervisor trips on it; counts vs producers |
| **Sensor opinions** | `BSK_LINK_NAV_STATE_TLM` (0x08CF), `BSK_LINK_ATTITUDE_STATE_TLM` (0x08D0) | a *modelled instrument* in the world — the plant's `gpsReceiver` / `starTracker` device, seeded, with the sigmas the vehicle's own TOML declared | two-sided, registry-driven: differ at device scale, agree as one world |

An opinion is a device output, not a quantity the harness happens to know. Each
one is a row in `registry/parts.toml` (its container, MsgId, EVS voice, exhibit),
and its acceptance bound is read from the manifest line that built the device:
under `floor·σ` the flight wire is reading an oracle (the cheat), over
`3·√3·σ` the two wires describe different universes. A vehicle that declares no
such device announces the channel off — nothing is zero-filled — and a vehicle
that fits one without declaring its sigma stops the gate, because a bound of
nothing grades a tautology.

A **fourth kind** crosses the link and never reaches the bus:
`BSK_LINK/FitManifest`, the world's declared part fit plus the digest of the
registry it was started with. It rides the sensor socket once per tick and
is consumed inside `bsk_link`, which compares three views of one question — what
the TOML claims, what the binary was BUILT with, what the bus actually streams —
and reports the verdict on the event bus (`FIT MATCHED` / `SILENT` / `DENIED`, or
a `FIT ABORT` that refuses the barrier when the digests disagree). It has no
TopicId for the same reason truth has none: a telemetry identity would put a
*ground copy* of the verdict beside the flight one, and the next reader would
grade the wrong number. The absence is the tripwire, and a test pins it
(`tests/test_fit_handshake.py`).

## The rules

1. **The flight software never sees truth.** `TruthState` deliberately has
   no TopicId, so no MsgId exists, so no cFS packet can exist. A tripwire
   test (`tests/test_eds2cosmos.py`) fails the build if any `TRUTH` packet
   appears in generated flight defs; the beacon wire audit would flag it as
   an UNKNOWN storm if a liar ever streamed one.
2. **Truth enters COSMOS on its own interfaces, per vehicle.** Born
   instance-aware: `TRUTH_<vehicle>` target, own gateway, own TCP port,
   `MAP_TARGET` explicit. Flight and truth never share a socket, an
   interface, or a target.
3. **The truth side is telemetry-only by construction, at three layers.**
   No command defs exist (a command fails at the def layer); the gateway
   counts and refuses any inbound byte and says so loudly (the `gs_tcp`
   beacon-refusal idiom, mirrored); there is nothing to command — truth has
   no actuators.
4. **Operators compare, never close loops on truth.** Screens put truth and
   knowledge side by side because the *gap* (gyro bias + noise, ~1e-5..1e-3
   rad/s on this vehicle) is the boundary made visible. Any control path that
   reads truth is a bug by definition — the acceptance gate measures both
   halves: truth and knowledge MUST differ and MUST agree on the same world.
5. **Sim control (not built) is a third channel, never a truth-port write.**
   If and when ground may edit the world itself (fault injection, what-if
   runs), it gets its own port block, interface, and
   audit trail. A truth *downlink* is a window; a sim *uplink* is a hand.

## The machine-checked form

`scripts/check_truth_exhibits.sh` (+ `sim/truth_exhibits.py`) per vehicle: truth and
knowledge interfaces both stream; `TRUTH_STATE_TLM` is fresh by advancing
`SIM_TIME_S`; `|ω_truth|` vs `|ω_knowledge|` differ (>1e-6) yet agree
(<3e-3) while coasting; **every landed registry row with a `truth_pair` exhibit
runs here** — GNSS as componentwise R/V against its declared sigmas, star
tracker as the representation-free angle gap between its quaternion and truth's
MRP at tick-exact epochs, both bounded by the manifest's own numbers
(`docs/adding_a_sensor.md`); enabling FCS **through the flight target** makes
`|ω|` fall **in the truth stream**; a `cmd` at the truth target fails with
"does not exist" at the def layer; raw bytes written to the truth port are
refused and logged; zero UNKNOWN-packet lines across all interfaces.
`sim/wire_audit.py` adds the def-side half: every MsgId on the beacon is
byte-exact against its generated definition.
