"""Part landing, pinned offline: a registered part costs one line.

A scenario block that declares a star tracker feeds an ATTSTATE observer -
topic from registry/parts.toml, codec generic, rig untouched. And the
quaternion on that wire is the device's OWN (STSensorMsgPayload
qInrtl2Case), in the scalar-first convention Basilisk's C2EP writes
(pinned here against the device, never against a comment), differing
from truth at the sigma the same line declared - an opinion, not a
relabeled oracle.
"""

from __future__ import annotations

import struct
from pathlib import Path

import pytest

import sim.forge_scenario as fz
from sim.eds_link import available
from sim.truth_exhibits import _angle_gap_rad, _mrp_to_quat

SCENARIOS = fz.DEFAULT_SCENARIO.parent

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

_STAR_TAIL = """\
[[spacecraft.sensors]]
type = "starTracker"
name = "st"
sigma_SB = [0.0, 0.0, 0.0]
white_noise = 1.0e-4
"""

# the same part with its noise model left out: legal TOML, no bound to judge by
_IDEAL_ST = """\
[[spacecraft.sensors]]
type = "starTracker"
name = "st"
"""


def _write(tmp_path: Path, name: str, extra: str) -> Path:
    path = tmp_path / name
    path.write_text(_MINIMAL_TOML.format(name="sc1", extra=extra), encoding="utf-8")
    return path


def test_one_toml_line_adds_the_registered_part(tmp_path: Path) -> None:
    """No device, no frame; one [[spacecraft.sensors]] line, ATTSTATE rides.

    THE exit criterion, as an assertion: nothing outside registry/parts.toml
    and this scenario file knew the part existed. The observer topic arrives
    keyed from the registry row; the rig, the codec and the spacecraft
    consumer are the shared machinery.
    """
    bare = fz.ForgeScenario(_write(tmp_path, "bare.toml", ""))
    assert not bare.body.has_plant_attitude
    assert "ATTSTATE" not in bare._rig.channels[0].observers

    fitted = fz.ForgeScenario(_write(tmp_path, "st.toml", _STAR_TAIL))
    assert fitted.body.has_plant_attitude
    observers = fitted._rig.channels[0].observers
    # the manifest fitted one device; TWO registered frames know it: the
    # The onboard bundle (legacy multi-part frame) and the registry's own opinion
    # frame - publish order fixed as ONBOARD then ATTSTATE, insertion order.
    assert list(observers) == ["ONBOARD", "ATTSTATE"]


def test_an_undeclared_sigma_stops_the_truth_gate(tmp_path: Path) -> None:
    """A fitted device with no declared noise is an oracle - and unjudgeable.

    bskforge's ``white_noise`` is optional and accepts 0.0, so a manifest can
    legally fit a noiseless star tracker. Both values destroy the exhibit:
    its two verdicts are multiples of that sigma, so DIFFER would pass
    vacuously and AGREE would have no bound. The check names the vehicle, the
    part and the field instead of grading a tautology - while a device that
    was never fitted stays plain ``None``: absence is a fact, this is a lie.
    """
    from sim.truth_exhibits import declared_sigmas

    bare = tmp_path / "bare_sigmas.toml"
    bare.write_text(_MINIMAL_TOML.format(name="sc1", extra=""), encoding="utf-8")
    assert declared_sigmas(bare)["sc1"] == {"gnss": None, "attitude": None, "opnav": None}

    for label, extra in (("ideal", _IDEAL_ST), ("zero", _IDEAL_ST + "white_noise = 0.0\n")):
        path = tmp_path / f"st_{label}.toml"
        path.write_text(_MINIMAL_TOML.format(name="sc1", extra=extra), encoding="utf-8")
        with pytest.raises(ValueError, match=r"sc1/attitude.*white_noise"):
            declared_sigmas(path)


@pytest.mark.skipif(not available(), reason="run edslink/build.sh for the EDS runtime")
def test_attitude_frame_is_the_device_opinion(tmp_path: Path) -> None:
    """48 B of plain LE; the quaternion is the device's, scalar-first, opinion-scaled.

    The wire bytes are pinned with struct (the contract: EdsLib packing ==
    C struct == plain LE). The convention claim is pinned against the
    DEVICE: reading the quaternion scalar-first against the navigator's MRP
    leaves noise, reading it scalar-last shows the attitude itself - so the
    coverage angle-gap comparison can never silently compare two different
    rotations. And the gap is NOT zero: this is a 1e-4 rad instrument, not
    truth wearing a costume.
    """
    scenario = fz.ForgeScenario(_write(tmp_path, "st.toml", _STAR_TAIL))
    scenario.run_to(2.0)
    payload = scenario.body.attitude_state_bytes(scenario.body.sim_time_s, 0)
    assert len(payload) == 48, "registry row: SimTimeS + Frame + StqBn[4]"
    _, _, q0, q1, q2, q3 = struct.unpack("<dQ4d", payload)
    q_scalar_first = (q0, q1, q2, q3)

    sigma = tuple(scenario.body.truth().sigma_bn[:3])
    q_truth = _mrp_to_quat(sigma)
    gap = _angle_gap_rad(q_scalar_first, q_truth)
    assert gap < 1.0e-3, f"scalar-first reading does not agree with truth: {gap:.3e} rad"
    assert gap > 1.0e-6, f"an ideal device is an oracle, not a star tracker: {gap:.3e} rad"
    assert gap < 5.0e-4, f"over 3*sqrt(3) sigma of a 1e-4 rad device: {gap:.3e} rad"

    wrong = (q1, q2, q3, q0)  # the scalar-last folklore reading
    assert _angle_gap_rad(wrong, q_truth) > 1.0, (
        "scalar-last accidentally agreed - the pin no longer discriminates"
    )
