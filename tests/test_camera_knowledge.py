"""The rendered eye wired into the world (CAM): estimators, provider, phase line.

No flight here: these tests BUILD the camera world - the forge's camera
phase plus the harness's estimators - and stop before ``InitializeSimulation``,
because a viz scenario blocks that call waiting for Vizard's handshake and
no renderer is owed one in a unit test. What must hold before any renderer
is dialed:

* the estimators ride the built camera module (image -> limb -> horizon),
  on the Earth index the message stream itself carries;
* the manifest's camera line claims the part in the fit vocabulary;
* the OPNAV provider joins the observer set only for camera plants, and
  before any Vizard frame has arrived it reports SILENCE (valid=0 with a
  zero vector) - a knowledge channel that ships a plausible default would
  be inventing a planet.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
from bskforge import load_scenario

import sim.forge_scenario as fz

REPO = Path(__file__).resolve().parents[1]
CAMERA_SCENARIO = REPO / "sim" / "scenarios" / "camera_ops.toml"
TWIN_SCENARIO = REPO / "sim" / "scenarios" / "twin_ops.toml"


def build_bodies(path: Path) -> list[fz.ForgeBody]:
    """Build every body of a scenario WITHOUT initializing the sim.

    Same construction path ForgeScenario/ForgeFleet take - ScenarioRunner's
    build, then ForgeBody - truncated before InitializeSimulation, which is
    exactly the window the estimators join the graph in.
    """
    config = load_scenario(path)
    built = fz.ScenarioRunner(config).build()
    ctx = built.context
    return [
        fz.ForgeBody(
            ctx,
            sc_cfg,
            seed=7,
            gyro_noise_sigma_rads=0.0,
            gyro_bias_rads=np.zeros(3),
        )
        for sc_cfg in config.spacecraft
    ]


def test_the_camera_world_builds_its_estimators() -> None:
    """[[camera]] rides the full knowledge chain: camera -> limb -> horizon."""
    (body,) = build_bodies(CAMERA_SCENARIO)
    assert body.has_plant_opnav, "[[camera]] on this craft built no estimators"
    assert body._opnav is not None
    camera_id, horizon, _, recorder = body._opnav
    assert camera_id == 1
    assert horizon.planetTarget == fz.OPNAV_PLANET_TARGET == 2, (
        "Earth's celestial index is what the scene's limb findings carry;"
        " another index silently navigates by a different body's radius"
    )
    assert recorder is not None, "the knowledge output was never recorded"
    assert body.sim_time_s == 0.0  # built, never run


def test_the_camera_line_claims_the_part_in_the_fit() -> None:
    """The manifest's [[camera]] joins the fit vocabulary as hardware (OB)."""
    (camera_body,) = build_bodies(CAMERA_SCENARIO)
    assert "camera" in camera_body.fitted_types
    for body in build_bodies(TWIN_SCENARIO):
        assert "camera" not in body.fitted_types, "twin_ops has no rendered eye"


def test_the_opnav_provider_joins_only_the_camera_world() -> None:
    """OPNAV rides the observer set exactly when the plant mounts the eye."""
    clock = lambda: (0.0, 0)  # noqa: E731 - the observers only need a clock shape
    (camera_body,) = build_bodies(CAMERA_SCENARIO)
    observers = fz.plant_observers(camera_body, clock)
    assert "OPNAV" in observers, "the camera plant publishes no knowledge frame"
    for body in build_bodies(TWIN_SCENARIO):
        assert "OPNAV" not in fz.plant_observers(body, clock)


def test_the_frame_packs_and_reports_silence_before_the_first_render() -> None:
    """Row landed, door open: before any Vizard frame the provider says so.

    Valid=0 with a zero vector is the channel declaring its own silence -
    the alternative, a plausible-looking default estimate, would be the
    knowledge channel inventing a planet.
    """
    from sim.eds_link import decode_opnav_state

    (camera_body,) = build_bodies(CAMERA_SCENARIO)
    payload = camera_body.opnav_state_bytes(1.0, 1)
    assert len(payload) == 72, "the provider packs what the row declares"
    frame = decode_opnav_state(payload)
    assert (frame.sim_time_s, frame.frame) == (1.0, 1)
    assert frame.r_bn_b == (0.0, 0.0, 0.0), "silence is zeros, never a fake fix"
    assert frame.planet_id == fz.OPNAV_PLANET_TARGET
    assert frame.camera_id == 1
    assert not frame.valid and not frame.fault_detected


def test_the_rig_keeps_no_startup_window_for_the_renderer() -> None:
    """Startup patience lives in the pinned assembly, so the rig keeps none.

    Stock Vizard closed its socket half a second after the bind - a window
    no service-started world can beat, because the first scene message
    physically needs a full scan. The yard compensated three ways: a timed
    SIM_UPDATE retry window in the rig, a port-probe gate before
    InitializeSimulation, and a sentinel handoff so the renderer started
    only when the world waited. The patience build pinned in vizard.lock
    (tools/vizard-patience/) waits for the first message as long as the
    world needs, so all three are deleted here and their absence is the
    rule. A SIM_UPDATE failure mid-flight now means the renderer died -
    a real fault, raised on the spot. Measured, the alternative is dead
    anyway: a patient player rebooted mid-stream cannot rejoin the
    conversation (it returns to its greeting screen while Basilisk waits
    an image response), so no retry window could ever have saved it.
    """
    import inspect

    import sim.lockstep as ls

    assert "LOAD_GRACE_S" not in inspect.getsource(ls), (
        "the load-window retry is retired: patience is in the assembly, not the rig"
    )
    fz_src = inspect.getsource(fz)
    assert "_await_renderer" not in fz_src, (
        "the renderer starts with the yard; nothing waits on its port"
    )
    assert "BSKFORGE_VIZ_WAIT_FILE" not in fz_src, "no sentinel: the patient player needs no clock"


def test_the_rig_is_wired_before_the_player_clock_starts() -> None:
    """The loop's first tick is the greeting, and nothing may jump the queue.

    The banned greeters stay banned with corrected receipts: a stolen
    step (scan before the rig existed) left that tick out of every rate
    history and MOVED the twin digests - camera-on/off byte equality
    being the one claim those anchors guard - and greeter v2's bare
    WriteProtobuffer died of an arithmetic exception, not a race: its
    render-cadence gate is a modulo by a renderRate that only the first
    scan's ReadBSKMessages copies in, and divide-by-zero is a hardware
    fault (gdb: divq by zero inside WriteProtobuffer). No message before
    the loop can be legal; the loop's first tick is the greeting. That
    law belongs to Basilisk's cadence gate, not to Vizard's startup
    window, so it outlives the patience patch - and both world paths keep
    wiring the rig before InitializeSimulation, so tick zero of run_to is
    the message the renderer waits for.
    """
    import inspect

    src = inspect.getsource(fz)
    assert "_greet_the_renderer" not in src, (
        "no message or step may answer the player outside the loop - "
        "both substitutions were flown and both failed"
    )
    helper = src.split("class ForgeError", 1)[0]
    assert "WriteProtobuffer" not in helper.split('"""')[-1], (
        "no snapshot may fire before the first scan: its cadence gate "
        "divides by a renderRate nothing has copied yet"
    )
    for name in ("ForgeScenario", "ForgeFleet"):
        segment = src.split(f"class {name}", 1)[1].split("\nclass ", 1)[0]
        assert "rig = LockstepRig(" in segment and "ctx.sim.InitializeSimulation()" in segment
        assert segment.index("rig = LockstepRig(") < segment.index(
            "ctx.sim.InitializeSimulation()"
        ), f"{name}: the loop's first tick must be the greeting, nothing before it"
