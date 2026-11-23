# Adding a spacecraft

The fleet has two parts, and a new vehicle needs one line in each:

1. **the scenario TOML** — what the *world* simulates;
2. **`compose.yml`** — which *containers* the yard launches.

`scripts/fleet.sh up` launches a service per vehicle named in the scenario,
so both must agree. The stock pair is `chief` + `deputy`
(`sim/scenarios/twin_ops.toml`); this guide adds a third, `relay`.

## 1. The scenario

Copy the deputy block in `sim/scenarios/twin_ops.toml` and rename it. The
block **index** is the port slot and the **name** is the ground identity —
you never write ports anywhere:

```toml
[[spacecraft]]
name = "relay"          # -> COSMOS target SAT_RELAY, truth TRUTH_RELAY
# ... plant bodies: copy the deputy block's [spacecraft.*] sections
```

Ports derive from the slot (`sim/fleet.py`): the third vehicle lands on
beacon `5054`, uplink `5055`, sensor `5620`, command `5621`, truth `5170`.

## 2. compose.yml

Copy the `chief` pair of services (`chief` and `truth-chief`) and rename.
Only five things differ per vehicle — everything else stays as-is:

```yaml
  relay:
    image: orrery-cfs:dev
    container_name: orrery-relay          # the yard sweeps orrery-*
    network_mode: host
    user: "${ORRERY_UID:-0}:${ORRERY_GID:-0}"
    working_dir: /work
    volumes:
      - ${ORRERY_REPO:-.}:/work
    environment:
      HOME: /tmp
      ORRERY_TLM_PORT: "${RELAY_BEACON:-5054}"   # stock defaults for slot 2
      ORRERY_CMD_PORT: "${RELAY_UPLINK:-5055}"
      ORRERY_ZMQ_SENSOR_ENDPOINT: "tcp://127.0.0.1:${RELAY_SENSOR:-5620}"
      ORRERY_ZMQ_CMD_ENDPOINT: "tcp://127.0.0.1:${RELAY_CMD:-5621}"
    command: bash -c 'cd /work/cfs_mission/mission/build/exe/cpu1 && ./container-start'

  truth-relay:
    image: orrery-bsk:dev
    container_name: orrery-truth-relay
    network_mode: host
    user: "${ORRERY_UID:-0}:${ORRERY_GID:-0}"
    working_dir: /work
    volumes: *engine-volumes
    environment: *engine-env
    depends_on:
      sync:
        condition: service_completed_successfully
    command: /opt/engine/venv/bin/python -m sim.truth_gateway --instance 2 --name relay
```

Rules the launch depends on:

- `container_name` starts with `orrery-` (that is how sweeps find the yard);
- mission services get the four `ORRERY_*` port env vars; the `:-defaults`
  are the stock values, and `fleet.sh` overrides them with whatever the
  scenario's slot actually derives — so a `--port-base` launch just works;
- each truth gateway needs `--instance <slot>` matching the scenario block
  index and `depends_on: sync` like its truth siblings.

## 3. Fly it

```bash
scripts/fleet.sh up sim/scenarios/twin_ops.toml    # or your scenario file
scripts/fleet.sh status
scripts/fleet.sh down
```

`up` refuses a scenario naming a vehicle without services and tells you to
edit `compose.yml`. `tests/test_compose_contract.py` pins the rules above —
if your edited file breaks a yard invariant, `uv run pytest` says which one
before anything launches.
