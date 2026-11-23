"""The link publisher really puts EDS frames on the wire.

Complements tests/test_eds_link.py (which pins the codec) and tests/test_detumble_scenario.py
(which pins the scenario): this one pins ``sim/link.py``, the reusable publisher, so the
byte contract that ``bsk_link`` decodes in flight is covered even when no mission
container is running.
"""

from __future__ import annotations

import time
from collections.abc import Iterator

import pytest
import zmq

from sim.eds_link import available, decode_torque_command, encode_torque_command
from sim.link import LinkPublisher

pytestmark = pytest.mark.skipif(
    not available(), reason="run edslink/build.sh to build the EDS database"
)

# Ephemeral loopback ports so a live mission or scenario can coexist.
SENSOR = "tcp://127.0.0.1:19580"
CMD = "tcp://127.0.0.1:19581"


@pytest.fixture()
def publisher() -> Iterator[LinkPublisher]:
    """A bound publisher, closed whatever the test concludes."""
    link = LinkPublisher(sensor_endpoint=SENSOR, cmd_endpoint=CMD)
    try:
        yield link
    finally:
        link.close()


def _subscribe(endpoint: str, topic: str) -> zmq.Socket:
    sock = zmq.Context.instance().socket(zmq.SUB)
    sock.connect(endpoint)
    sock.setsockopt_string(zmq.SUBSCRIBE, topic)
    return sock


def test_sensor_frame_is_an_eds_container_on_the_wire(publisher: LinkPublisher) -> None:
    """Topic frame + 40-byte BSK_LINK/SensorFrame payload, nothing else."""
    sub = _subscribe(SENSOR.replace("*", "127.0.0.1"), "SENSOR")
    time.sleep(0.5)  # PUB/SUB drops frames published before the subscription settles
    publisher.heartbeat(sim_time_s=7.5, frame=3)
    assert sub.poll(2000, zmq.POLLIN), "no frame reached the subscriber"
    topic, payload = sub.recv_multipart()
    sub.close(linger=0)

    assert topic == b"SENSOR"
    assert len(payload) == 40, "SensorFrame is sim time + 3-axis rate + frame counter"

    # The same codec the scenario uses must read it back unchanged: proof the
    # publisher added no framing of its own and the bytes are the plain EDS form.
    assert payload == _reference_bytes(7.5, 3)


def _reference_bytes(sim_time_s: float, frame: int) -> bytes:
    """What the codec alone produces, to compare the publisher against."""
    from sim.eds_link import encode_sensor_frame

    return bytes(encode_sensor_frame(sim_time_s, (0.0, 0.0, 0.0), frame))


def test_commands_are_decoded_not_just_received(publisher: LinkPublisher) -> None:
    """A PUSHed TorqueCommand arrives as fields, not as bytes to hand-parse."""
    push = zmq.Context.instance().socket(zmq.PUSH)
    push.connect(CMD)
    payload = bytes(encode_torque_command(4.25, (0.001, -0.002, 0.003), 11))
    push.send_multipart([b"CMD", payload])

    command = publisher.recv_command(timeout_ms=2000)
    push.close(linger=0)

    assert command is not None
    assert command.frame == 11
    assert command.sim_time_s == pytest.approx(4.25)
    assert command.torque_sum_b == pytest.approx((0.001, -0.002, 0.003))
    assert decode_torque_command(payload) == command, "decode is not deterministic"


def test_recv_command_returns_none_when_idle(publisher: LinkPublisher) -> None:
    """The polling contract the scenario's free-run loop relies on."""
    assert publisher.recv_command(timeout_ms=10) is None
