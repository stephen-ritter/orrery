# cfs_mission — cFS mission layer

The core is **fetched, not vendored**: `cfs.lock` pins it (tag `v7.0.1`, one
commit) and `scripts/fetch_cfs.sh` fills in `cfs_mission/cFS/`, which git ignores.
Nothing of orrery's lives inside that directory — the mission sits in `mission/`
and is grafted onto the core at configure time, which is what keeps the rule
"the core is pristine" checkable rather than aspirational:

- `-DMISSION_DEFS=<here>/mission/orrery_defs` (mission defs dir outside the cFS tree)
- `CFS_APP_PATH=<here>/mission/apps` (extra module search path; **entries are
  parents of module dirs**)

Fetch and check:

```bash
scripts/fetch_cfs.sh            # get the pinned core (idempotent)
scripts/fetch_cfs.sh --verify   # is what is here the pinned one? (the build gate runs this)
```

Build: `./build.sh [prep|build|install|all|run]` inside `orrery-cfs:dev`
(repo mounted at `/work`; run the container as your host UID so artifacts are
owned sanely) — or `scripts/mission_build.sh` from anywhere. Executable lands at
`mission/build/exe/cpu1/core-cpu1` (`./container-start` launches it). App `.so`s
+ tables stage to `exe/cpu1/cf/`.

What is here:

| Path | What |
| --- | --- |
| `mission/orrery_defs/` | mission `targets.cmake` (our apps in `APPLIST`) + startup-table hook |
| `mission/orrery_defs/eds/` | the hand-authored link schema: topic defines + one package per mission app (`bsk_link.xml`, `fcs.xml`, `obc.xml`, `sps.xml`) — the source both the flight decode and the ground defs come from |
| `mission/apps/bsk_link/` | in-mission C app: libzmq + the linked-in EdsLib database; decodes `BSK_LINK` frames by EDS name, moves them on/off the SB, and runs the boot handshake that names a fit the bus does not match |
| `mission/apps/gs_tcp/` | the gateway: beacon `:5050` streams every MsgId in the generated table (telemetry out only), uplink `:5051` injects CCSDS commands |
| `mission/apps/fcs/` | the control app: rate damping, wheels and modes; consumes the sensor MsgId, commands torque, ground owns ENABLE/DISABLE/gains |
| `mission/apps/obc_hk/` | OBC health/attitude telemetry packet |
| `mission/apps/sps/` | stored command sequences executed on-board time — the satellite carrying out the future without further contact |

Build happens inside `docker/cfs.Dockerfile` (Linux native PSP, EDS enabled).
