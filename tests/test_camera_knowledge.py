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
import pytest
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
    camera_id, module, horizon, _, recorder, image_recorder = body._opnav
    assert camera_id == 1
    assert module is not None, "the config door writes through this handle"
    assert horizon.planetTarget == fz.OPNAV_PLANET_TARGET == 2, (
        "Earth's celestial index is what the scene's limb findings carry;"
        " another index silently navigates by a different body's radius"
    )
    assert recorder is not None, "the knowledge output was never recorded"
    assert image_recorder is not None, "the captured witness was never wired"
    assert body.sim_time_s == 0.0  # built, never run


def test_the_camera_line_claims_the_part_in_the_fit() -> None:
    """The manifest's [[camera]] joins the fit vocabulary as hardware."""
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
    assert "CAMSTATE" in observers, "the camera plant publishes no device-state frame"
    for body in build_bodies(TWIN_SCENARIO):
        twin = fz.plant_observers(body, clock)
        assert "OPNAV" not in twin, "twin_ops has no rendered eye"
        assert "CAMSTATE" not in twin, "absence is declared per frame, not zero-filled"


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


def test_the_camstate_frame_reports_the_device_as_built() -> None:
    """The device frame packs the module's OWN fields, pre-run."""
    from sim.eds_link import decode_cam_state

    (camera_body,) = build_bodies(CAMERA_SCENARIO)
    payload = camera_body.cam_state_bytes(1.0, 1)
    assert len(payload) == 56, "the provider packs what the row declares"
    frame = decode_cam_state(payload)
    assert (frame.sim_time_s, frame.frame) == (1.0, 1)
    assert frame.camera_id == 1
    assert camera_body._opnav is not None
    module = camera_body._opnav[1]
    assert frame.shutter == int(module.cameraIsOn), "the frame is the module, not an opinion"
    assert frame.render_rate_s == pytest.approx(module.renderRate / 1e9)
    assert frame.captured == 0, "nothing rendered yet; a default capture would be a lie"
    assert frame.frames_recorded == 0, "a nav-only camera records nothing"


def test_the_store_outlet_streams_every_nth_capture_and_only_once(tmp_path: Path) -> None:
    """The store outlet chunks every Nth capture's PNG, exactly once.

    A scripted eye (fake recorder + real files on disk) flies the outlet's
    whole law without a renderer: every Nth capture by ordinal is sent, the
    filename is the capture time at 6 decimals (camera.cpp's spelling), a
    > chunk file arrives as ordered parts, the tally counts COMPLETE sends
    only, a second tick re-sends nothing, and a capture whose file has not
    landed yet is retried, never skipped.
    """
    import struct

    from sim.eds_link import decode_cam_state

    class FakeRecorder:
        def __init__(self) -> None:
            self.timeTag: list[float] = []

    (camera_body,) = build_bodies(CAMERA_SCENARIO)
    assert camera_body._opnav is not None
    camera_id, module, horizon, att, knowledge, _ = camera_body._opnav
    fake = FakeRecorder()
    camera_body._opnav = (camera_id, module, horizon, att, knowledge, fake)
    store_dir = tmp_path / "store"
    store_dir.mkdir()
    camera_body._cam_store = {"rate": 2, "dir": store_dir, "sent": [], "count": 0}
    sent: list[bytes] = []
    camera_body.attach_store_emitter(sent.append)

    big = bytes(range(256)) * 80  # 20480 B: two IMG parts, provably
    small = b"\x89PNG tiny"
    # timeTags arrive in NANOSECONDS (the live-flight lesson: a 2 s capture
    # stamps 2e9); the file name is the SECONDS value at 6 decimals, exactly
    # what camera.cpp's std::to_string writes.
    for t, data in ((1.0, small), (2.0, big), (3.0, small), (4.0, small)):
        fake.timeTag.append(t * 1e9)
        (store_dir / f"{t:.6f}.png").write_bytes(data)

    payload = camera_body.cam_state_bytes(5.0, 5)
    frame = decode_cam_state(payload)
    assert frame.frames_recorded == 2, "captures 2 and 4 are the record_rate=2 set"
    headers = [struct.unpack(camera_body.IMG_HEADER, f[:24]) for f in sent]
    assert [h[:2] for h in headers[:3]] == [(1, 2), (1, 2), (1, 4)]
    assert headers[0][2:] == (0, 2) and headers[1][2:] == (1, 2), "ordered parts, correct total"
    assert headers[2][2:] == (0, 1), "capture 4 is one part"
    rebuilt = b"".join(
        f[24:] for f in sent if struct.unpack(camera_body.IMG_HEADER, f[:24])[1] == 2
    )
    assert rebuilt == big, "the parts must rebuild the file byte for byte"
    assert len(sent) == 3, "1 part + 2 parts + 1 part"

    # same tick's state again: the outlet never re-sends a capture
    before = len(sent)
    frame2 = decode_cam_state(camera_body.cam_state_bytes(6.0, 6))
    assert len(sent) == before and frame2.frames_recorded == 2, "exactly-once per capture"

    # patience law: a due capture with no file yet is retried, not lost
    fake.timeTag.append(5.0 * 1e9)
    fake.timeTag.append(6.0 * 1e9)
    frame3 = decode_cam_state(camera_body.cam_state_bytes(7.0, 7))
    assert frame3.frames_recorded == 2 and len(sent) == before, "no file, no record"
    (store_dir / "6.000000.png").write_bytes(small)
    frame4 = decode_cam_state(camera_body.cam_state_bytes(8.0, 8))
    assert frame4.frames_recorded == 3, "the file landed; the capture is finally recorded"


def test_the_store_outlet_is_silent_when_the_scenario_said_no() -> None:
    """record_rate = 0: the outlet does not exist - zero frames, zero tally."""
    (camera_body,) = build_bodies(CAMERA_SCENARIO)
    sent: list[bytes] = []
    camera_body.attach_store_emitter(sent.append)
    assert camera_body.store_active is False
    assert camera_body._pump_store(1, [1.0, 2.0, 3.0]) == 0
    assert sent == []


def test_the_config_door_applies_refuses_and_stays_reversible() -> None:
    """The sim's door: write truthfully, refuse the zero, undo.

    Every law here was flown before it was tested:
    writes take effect at the next update, a refused value must NEVER reach
    the module, and a config is fully reversible mid-flight. The door
    answers nobody - refusals tally, they do not ack - so this test is the
    receipt the airgap does not carry.
    """
    from sim.eds_link import CamConfig

    (camera_body,) = build_bodies(CAMERA_SCENARIO)
    assert camera_body._opnav is not None
    module = camera_body._opnav[1]

    camera_body.apply_cam_config(CamConfig(1.0, 10, 1, 0, 4.0))
    assert module.cameraIsOn == 0 and module.renderRate == 4_000_000_000
    assert camera_body._cam_stats["applied"] == 1

    camera_body.apply_cam_config(CamConfig(2.0, 20, 1, 1, 0.0))  # B1: the zero
    assert module.cameraIsOn == 0, "a refused shutter command changed the device"
    assert module.renderRate == 4_000_000_000, "a refused rate reached the modulo"
    assert camera_body._cam_stats["refused_rate"] == 1

    camera_body.apply_cam_config(CamConfig(3.0, 30, 7, 1, 1.0))  # eye not fitted
    assert camera_body._cam_stats["refused_id"] == 1
    assert module.cameraIsOn == 0, "a command for another eye edited this one"

    camera_body.apply_cam_config(CamConfig(4.0, 40, 1, 1, 2.0))  # reversible
    assert module.cameraIsOn == 1 and module.renderRate == 2_000_000_000


def test_the_cam_config_frame_round_trips_through_the_schema() -> None:
    """The egress frame the FSW builds is the frame the sim decodes."""
    from sim.eds_link import decode_cam_config, encode_cam_config

    payload = bytes(encode_cam_config(3.5, 42, 1, 0, 4.25))
    assert len(payload) == 40, "the CamConfig row declares 40 B"
    cfg = decode_cam_config(payload)
    assert (cfg.sim_time_s, cfg.frame, cfg.camera_id) == (3.5, 42, 1)
    assert cfg.shutter == 0 and cfg.render_rate_s == pytest.approx(4.25)


def test_cam_frames_apply_between_ticks_and_never_open_the_barrier() -> None:
    """The law at the barrier itself: configs apply, the ack is still awaited.

    The v1 law survives the command surface: camera traffic never becomes a
    receipt the physics waits on. The check is driven through its own private
    door (``_await_barrier``) with a scripted fake FSW - the same pre-pushed
    ZeroMQ trick ``test_fake_fsw_closes_the_gate_frame_exact`` uses, minus a
    plant the law does not need.
    """
    import zmq

    from sim.eds_link import encode_cam_config, encode_sensor_frame, encode_step_ack
    from sim.lockstep import LockStalled, LockstepRig, RigChannel

    ctx = zmq.Context.instance()
    pull = ctx.socket(zmq.PULL)
    cmd_port = pull.bind_to_random_port("tcp://127.0.0.1")
    push = ctx.socket(zmq.PUSH)
    push.connect(f"tcp://127.0.0.1:{cmd_port}")
    # PUB like the rig's real sensor feed: a check waiting without a listener
    # must never block on its own retransmits (a PUSH with no peer would).
    sensor = ctx.socket(zmq.PUB)
    sensor.bind_to_random_port("tcp://127.0.0.1")

    seen: list = []
    ch = RigChannel(
        name="camgate",
        gyro=lambda: np.zeros(3),
        apply_torque=lambda torque: None,
        truth_provider=lambda: b"",
        kp=np.zeros(3),
        cam_sink=seen.append,
    )
    ch.sockets = {"cmd": pull, "sensor": sensor}
    rig = LockstepRig(
        None,
        control_period_s=1.0,
        channels=[ch],
        lockstep=True,
        ack_timeout_s=1.0,
        ack_retry_s=0.05,
    )
    sensor_bytes = bytes(encode_sensor_frame(1.0, (0.0, 0.0, 0.0), 1))

    # A config alone must NOT open the check - but it is applied on arrival.
    push.send_multipart([b"CAM", bytes(encode_cam_config(1.0, 1, 1, 0, 4.0))])
    with pytest.raises(LockStalled):
        rig._await_barrier(ch, 1, sensor_bytes)
    assert len(seen) == 1 and ch.cam_configs == 1, "config door opened while the check stayed shut"
    assert ch.bad_frames == 0, "a well-formed CAM frame is not a malformed one"

    # Config + the ack fcs owes: the check answers, and BOTH frames landed.
    push.send_multipart([b"CAM", bytes(encode_cam_config(2.0, 2, 1, 1, 2.0))])
    push.send_multipart([b"ACK", bytes(encode_step_ack(2.0, 2, 1, 0))])
    assert rig._await_barrier(ch, 2, sensor_bytes) is None
    assert len(seen) == 2 and ch.cam_configs == 2

    push.close(linger=0)
    pull.close(linger=0)
    sensor.close(linger=0)


def test_the_rig_keeps_no_startup_window_for_the_renderer() -> None:
    """Startup patience lives in the pinned assembly, so the rig keeps none.

    Stock Vizard closed its socket half a second after the bind - a window
    no service-started world can beat, because the first scene message
    physically needs a full scan. The yard compensated three ways: a timed
    SIM_UPDATE retry window in the rig, a port-probe check before
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
    render-cadence check is a modulo by a renderRate that only the first
    scan's ReadBSKMessages copies in, and divide-by-zero is a hardware
    fault (gdb: divq by zero inside WriteProtobuffer). No message before
    the loop can be legal; the loop's first tick is the greeting. That
    law belongs to Basilisk's cadence check, not to Vizard's startup
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
        "no snapshot may fire before the first scan: its cadence check "
        "divides by a renderRate nothing has copied yet"
    )
    for name in ("ForgeScenario", "ForgeFleet"):
        segment = src.split(f"class {name}", 1)[1].split("\nclass ", 1)[0]
        assert "rig = LockstepRig(" in segment and "ctx.sim.InitializeSimulation()" in segment
        assert segment.index("rig = LockstepRig(") < segment.index(
            "ctx.sim.InitializeSimulation()"
        ), f"{name}: the loop's first tick must be the greeting, nothing before it"
