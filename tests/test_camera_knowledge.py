"""The rendered eye wired into the world (CAM): estimators, provider, phase line.

No flight here: these tests BUILD the camera world - the forge's camera
phase plus the harness's estimators - and stop before ``InitializeSimulation``,
because a viz scenario blocks that call waiting for Vizard's handshake and
no renderer is owed one in a unit test. What must hold before any renderer
is dialed:

* the estimators ride the built camera module (image -> limb -> horizon),
  on the Earth index the message stream itself carries;
* the manifest's camera line claims the part in the fit vocabulary;
* the OPNAV provider joins the observer set only for camera plants;
* the provider's frame stays LOCKED until the registry row lands - the
  encode door is the phase line (the rig's topic validation is the second
  one, and it fires at run wiring, not here).
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest
from bskforge import load_scenario

import sim.forge_scenario as fz
from sim.eds_link import LinkUnavailable

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


def test_the_opnav_frame_is_locked_until_the_row_lands() -> None:
    """The encode door is the registry's phase line: unlanded, unpackable.

    Landing the row (codegen, the C table, the defs, the exhibit - one
    ceremony) is what unlocks this provider. Until then the camera world
    cannot even be flown through the rig: RigChannel refuses providers on
    topics no landed row owns, so the phase line fails LOUD at wiring.
    """
    (camera_body,) = build_bodies(CAMERA_SCENARIO)
    with pytest.raises(LinkUnavailable, match="not a landed registry container"):
        camera_body.opnav_state_bytes(1.0, 1)
