# What the scenario rules

One scenario file is one world. This page draws the line precisely: what a
scenario declaration decides today, where the line used to sit, and what it
takes to move the line. It is the companion to
[adding_a_sensor.md](adding_a_sensor.md) (how to add a part) and
[adding_a_spacecraft.md](adding_a_spacecraft.md) (how to add a vehicle); this
page answers the deeper question of what "the TOML is the source of truth"
actually promises, honestly, surface by surface.

## The promise, surface by surface

A scenario makes a vehicle — or takes one away. Every surface that truth
touches:

| Surface | What the scenario decides | How the promise is kept |
| --- | --- | --- |
| The plant | Devices exist or they don't. Delete `[[camera]]` and the world has no eye: no renderer is asked for, the OPNAV provider never joins the observer set. | `sim/forge_scenario.py` builds the Basilisk world from the file alone. |
| The wire | Which knowledge frames cross to flight software. | Rows in `registry/parts.toml` claim topics; `RigChannel` refuses to stream a frame whose part the scenario never mounted. Absence is declared, not zero-filled. |
| The message shapes | The byte contract of every packet, on flight and ground alike. | `codegen/eds2cosmos.py` generates the COSMOS definitions and the `gs_tcp` MsgId table from the cFS EDS; `edslink` builds the simulator side from the same XML list, so the two cannot drift. |
| The spacecraft's wake-up | Which flight-software apps the core starts. | `codegen/emit_startup.py` derives the ES startup table from the scenario per vehicle; the yard door lands it, and a craft boots its own table or refuses to boot. See below. |
| The station's offering | Which commands and packets the operator is offered for a vehicle. | `codegen/eds2cosmos --scope-scenario` cuts the COSMOS dictionary to the same wake list: no button for an app the vehicle never wakes. |
| The image's organs | Which flight-software apps exist as object files at all. | `codegen/emit_applist` lands the scenario's build fragment; `targets.cmake` compiles exactly that list, and the build's own wake-up table can name no organ the build never compiled. |
| The proof | Which acceptance exhibits a part arrives with. | Every `[[part]]` row names its exhibit; the checks read the row, so a part cannot be added without its verdict. |

The registry law covers all of it: generators and checks read
`registry/parts.toml`, and `tests/test_parts.py` pins that every row agrees
with reality surface by surface — a row that lies about a topic, a MsgId or a
container fails like a wrong C struct would.

## Where the line used to sit — and why it moved

Until the startup table was generated, the scenario ruled the world but not
the spacecraft's wake-up list: `cfe_es_startup.scr` named every app — camera
included — for every vehicle, always. A scenario without an eye still booted
the camera app; it sat silent, its counters at their honest zero, and the
acceptance legs proved absence by that silence.

Silence is a weak witness. A table bug that booted *nothing* looked exactly
like a correct no-camera boot, and "present but dormant" is a promise the
operator cannot see. The current law closes the gap at the door: **the
scenario's part declarations select the payload apps each vehicle wakes with,
through the same audited generators everything else enters by.** The baseline
apps — the vehicle's plumbing: `bsk_link`, `gs_tcp`, the labs, the file
service — are fleet-invariant by declaration in `registry/apps.toml`, which
keeps the rule in one visible place instead of in a comment.

What was deliberately *not* collapsed: the binaries still build as one image.
The scenario decides what wakes, not what exists in the flash. A per-scenario
binary is discussed below; the bench's current law is one stamped image, many
worlds, and the digest anchors prove the worlds never move physics.

## Row or app: when a part needs flight software

Most parts never touch flight-software logic. The star tracker proves it: a
plant device, a registry row, `bsk_link` as the generic consumer, generated
shapes on both ends, a truth-pair exhibit — the full circuit is data. A
part needs an app of its own when something must be **decided, owned, or
refuted onboard rather than merely carried**:

1. **Command authority with a duty to refuse.** The ground ordering a part
   around needs a receiver that validates before forwarding. `cam` refusing
   a render rate of zero before the simulator divides by zero is the
   canonical case; the transport app must stay a carrier and cannot own a
   "no". This is also why argument-bearing commands get their own packet and
   interface: codegen loses what no packet declares.
2. **A standing claim the ground must be able to distrust.** The two-view
   law needs an onboard voice: `CAM_HK` (what the craft believes it
   commanded) against `CamState` (what the world reports in use). The
   divergence between the two *is* the defect class; one app holding both
   opinions would grade its own homework. A pure-row part's pair is
   truth-versus-knowledge instead, which needs no app.
3. **The file plane.** Products that live onboard and get requested by name —
   store bookkeeping, manifests — need a keeper: the transport bridges swear
   they have no file knowledge. Any payload producing products, not just
   imagers, inherits this; the file path stays the only data plane, and
   inventory is a file too.
4. **Onboard decision loops.** When knowledge becomes action without the
   ground in the ring, the decision lives in flight software, and flight
   software logic is an app.

Two boundary cases where teams write an app they do not need: a second
camera is not a new app (`CAMERA_ID` already speaks instances — more rows
point at the same authority), and a payload that only needs downlink is not
a new app (the store, manifest and transfer law are generic; only a
payload-specific store *policy* would need a new keeper).

The registry records the answer: a part row that brings an app says so with
`fsw_apps = ["cam"]`, and `registry/apps.toml` inventories every app the core
can start — baseline or declared — with its entry symbol, priority and stack.
The startup generator reads both; a row naming an app the inventory lacks, or
an app whose entailments (`requires`) are unmet, fails the door with a named
sentence, not a silent table. The price cliff is real and now visible: a row
costs an entry, an app costs authority — and authority earns its keep by
being flunkable.

## The ladder up to the full promise

Moving the source-of-truth line further has a known order; each step spends
the discipline the previous one taught:

1. **The spacecraft wakes up from the TOML** (landed, this is `emit_startup`
   above): the startup table becomes a deploy-time artifact generated per
   vehicle and landed by the yard door; `container-start` boots a vehicle's
   own table or refuses, and says on the record which table it booted. The
   image's own build-time table stays as the honest fallback for a hand-run
   `docker compose up`, byte-identical to the all-parts union — a test pins
   the two sources of the grammar to each other.
2. **The ground is cut from the same TOML** (landed): `eds2cosmos --scope-scenario`
   cuts a vehicle's COSMOS dictionary to the apps its scenario wakes — no
   command for an organ the vehicle never grew. The scope is a filter over the
   same EDS pipeline, so a declared-eye vehicle's scoped definitions are
   byte-identical to the catalog a test can pin; `CFE_*` commands are nobody's
   to scope away (the core always boots), and frames riding the generic
   consumer stay defined — they are shared vocabulary, scoped by row-landing
   on the wire, not by the boot record. `scripts/check_ground_scope.sh` flies
   the station itself: a strict plugin variant (versioned below the shipped
   one, so a restart can only wake the catalog) loads, the buttons vanish for
   the cameraless chief while the unscoped deputy keeps them, and the catalog
   restore brings them back. The craft's own timer obeys the same list:
   the schedule table (`orrery_defs/tables/sch_lab_table.c`) is compiled
   into every image and fires onboard, so it may speak only to the core
   and the baseline organs — `tests/test_schedule_law.py` pins the ledger
   against the inventory, and against the built table where one exists.
   A payload app's housekeeping is ground-polled through the scoped
   dictionary, never timer-driven: the timer never fires toward an organ
   a scenario might not have grown, which is exactly what lets a
   per-scenario flight image leave a declared app out without stranding
   a timer that talks to it.
3. **The binary is cut from the TOML** (landed): the manifest feeds the
   mission's app list. The yard door lands a build fragment naming the apps
   the flying fleet can wake (`codegen/emit_applist` beside the startup
   tables), and `targets.cmake` compiles exactly those — a scenario that
   never grew the eye carries no `cam` object file to boot, to link, or to
   lose. The image's own wake-up table follows the same list, so a build-time
   fallback can never promise an organ the build never compiled. No fragment,
   the all-parts union: the same fallback law as the startup table, and a test
   pins the union literal to the inventory. The entailments are the build
   door for free — what the manifest cannot serve, it must not compile.
   Provenance multiplies honestly: the fragment enters the stamp, so an image
   built for one scenario's list fails another's check, and `down` sweeps the
   fragment back to the union. `scripts/check_scenario_image.sh` flies the
   plane itself: cut world with no object file and a beacon that streams
   everything but the eye, whole world with the file and CAM_HK answered,
   union restored.

## The law that keeps it honest

Every step up spends a little of the same coin, so one rule pays it back:
**at least one witness per acceptance leg must not be derived from the
scenario file.** "The camera is absent" is never proven by reading back the
TOML that said so; it is proven by a raw port that hears no `CAM_HK`, by the
boot record naming the table that was booted, by a digest that refuses to
move. Generated expectations checking generated reality is how a framework
grades its own homework — the standing digest anchors, pinned from physics
rather than from any declaration, are what keep this a factory instead of a
hall of mirrors.

`scripts/check_startup_config.sh` is that rule made flesh: the same
command sentence, flown on a vehicle that declared an eye (`CAM_HK` arrives)
and one that never did (the raw beacon stays silent, the booted table is
named in the container's record). `scripts/check_ground_scope.sh` is its
ground-side twin: the verdict is what the station's own dictionary offers,
read back from the station's own API.
