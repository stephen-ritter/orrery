"""KN2: the GNSS receiver is plant hardware, declared in the manifest.

The scenario-layer error model (``sim/gps.py``, KN) retired the way the
Python gyro model retired at P4a: a ``type = "gpsReceiver"`` line now
builds a native ``SimpleNav`` in the world model, and the harness reads
the device's output message. These tests pin the three promises that
graduation made:

1. the fleet manifests fit receivers, and p5 reads its bounds from them;
2. the NAVSTATE bytes ARE the plant message - differ at declared scale,
   inside Basilisk's 3-sigma Gauss-Markov clamp, never recomputed here;
3. rename-proof luck (D10-R1): the receiver's noise seed lives in the
   schema default, so two identically-declared vehicles draw identical
   noise and renaming one re-draws nothing.

A vehicle without the line must stream no NavState at all: the rig
channel carries no NAVSTATE observer (absence declared, not zero-filled).
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest
from bskforge import load_scenario

from codegen.eds_sources import CFS
from sim import forge_scenario as fz
from sim.eds_link import available, decode_nav_state
from sim.truth_exhibits import declared_sigmas

pytestmark = [
    pytest.mark.skipif(not CFS.is_dir(), reason="cFS is not fetched: run scripts/fetch_cfs.sh"),
    pytest.mark.skipif(not available(), reason="run edslink/build.sh to build the EDS database"),
]

SCENARIOS = fz.DEFAULT_SCENARIO.parent
CHIEF_TOML = SCENARIOS / "chief.toml"
TWIN_TOML = SCENARIOS / "twin_ops.toml"

_MINIMAL_TOML = """\
[simulation]
duration = 10.0
[simulation.dt]
dynamics = 1.0
fsw = 1.0
[world]
central_body = "earth"
[[spacecraft]]
name = "{name}"
[spacecraft.orbit]
type = "elements"
[spacecraft.orbit.elements]
a = 7.0e6
{extra}"""

_RECEIVER_TAIL = """[[spacecraft.sensors]]
type = "gpsReceiver"
name = "{sensor}"
"""


def _receiver_toml(name: str, sensor: str = "gnss") -> str:
    return _MINIMAL_TOML.format(name=name, extra=_RECEIVER_TAIL.format(sensor=sensor))


def _write(tmp_path: Path, name: str, body: str) -> Path:
    path = tmp_path / name
    path.write_text(body, encoding="utf-8")
    return path


def test_fleet_manifests_fit_receivers() -> None:
    """Both fleet files carry the lines that build the plant receivers."""
    chief = declared_sigmas(CHIEF_TOML)
    assert chief == {"chief": {"gnss": (15.0, 0.01), "attitude": (1.0e-4,), "opnav": None}}
    twin = declared_sigmas(TWIN_TOML)
    assert twin == {
        "chief": {"gnss": (15.0, 0.01), "attitude": (1.0e-4,), "opnav": None},
        "deputy": {"gnss": (15.0, 0.01), "attitude": (1.0e-4,), "opnav": None},
    }
    # and the sigmas p5 judges by are the schema's own defaults, restated
    # nowhere: a loaded config agrees with the raw-file read.
    config = load_scenario(TWIN_TOML)
    gps = next(s for s in config.spacecraft[0].sensors if s.type == "gpsReceiver")
    assert (gps.pos_sigma_m, gps.vel_sigma_m_s) == twin["chief"]["gnss"]


def test_plant_receiver_is_the_nav_source_when_fitted(tmp_path: Path) -> None:
    """With the line in the TOML, the fix IS the SimpleNav message (KN2)."""
    scenario = fz.ForgeScenario(_write(tmp_path, "rx.toml", _receiver_toml("sc1")))
    assert scenario.body.has_plant_nav
    scenario.run_to(3.0)  # three scans, no control boundary crossed
    truth = scenario.body.truth()
    state = decode_nav_state(scenario.body.nav_state_bytes(scenario.body.sim_time_s, 0))
    dr = np.asarray(state.r_bn_n, dtype=float) - np.asarray(truth.r_bn_n, dtype=float)
    dv = np.asarray(state.v_bn_n, dtype=float) - np.asarray(truth.v_bn_n, dtype=float)
    dr_norm, dv_norm = float(np.linalg.norm(dr)), float(np.linalg.norm(dv))
    pos_sigma, vel_sigma = 15.0, 0.01
    # Basilisk clamps every error component at its 3-sigma walk bound, so
    # the norm cannot leave the cube diagonal; and a live receiver must
    # actually disagree with truth, or it is an oracle in disguise.
    assert dr_norm <= 1.01 * 3.0 * 3.0**0.5 * pos_sigma, f"over the clamp: {dr_norm:.1f} m"
    assert dr_norm >= 0.2 * pos_sigma, f"reads truth, not an instrument: {dr_norm:.2f} m"
    assert dv_norm <= 1.01 * 3.0 * 3.0**0.5 * vel_sigma
    assert dv_norm >= 0.2 * vel_sigma


def test_vehicle_without_receiver_streams_no_nav(tmp_path: Path) -> None:
    """No ``gpsReceiver`` block: the NAVSTATE channel is off by declaration."""
    scenario = fz.ForgeScenario(
        _write(tmp_path, "bare.toml", _MINIMAL_TOML.format(name="sc1", extra=""))
    )
    assert not scenario.body.has_plant_nav
    assert "NAVSTATE" not in scenario._rig.channels[0].observers


def test_rename_cannot_redraw_the_receivers_luck(tmp_path: Path) -> None:
    """Identical declarations, different names: identical noise (D10-R1).

    The seed lives in the schema default, not the ModelTag - so two vehicles
    that declared the same receiver draw the same realization, and renaming
    one re-draws nothing. If a future builder ever seeds from a name, the
    digests of every renamed vehicle would silently move; this test is the
    tripwire, the same watch the harness keeps on its own Python streams.
    """
    one = fz.ForgeScenario(_write(tmp_path, "one.toml", _receiver_toml("alpha")))
    two = fz.ForgeScenario(_write(tmp_path, "two.toml", _receiver_toml("bravo", "radio")))
    one.run_to(2.0)
    two.run_to(2.0)
    fix_one = decode_nav_state(one.body.nav_state_bytes(one.body.sim_time_s, 0))
    fix_two = decode_nav_state(two.body.nav_state_bytes(two.body.sim_time_s, 0))
    assert np.array_equal(fix_one.r_bn_n, fix_two.r_bn_n), (
        "receiver luck depends on the vehicle/sensor NAME - reseeded by a rename"
    )
    assert np.array_equal(fix_one.v_bn_n, fix_two.v_bn_n)
