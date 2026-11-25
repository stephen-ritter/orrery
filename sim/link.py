"""ZeroMQ publisher side of the orrery link (the Basilisk role).

Frames are EdsLib-packed ``BSK_LINK`` containers, ZMQ multipart with the
channel name as the first frame and the EDS payload as the second. The channel contract
lives in ``edslink/README.md``; the schema lives in the mission EDS XML.
"""

from __future__ import annotations

import zmq

from sim.eds_link import TorqueCommand, decode_torque_command, encode_sensor_frame

# One source of truth for the channel contract: the rig that flies it.
# Re-exported (the historical link.py surface) even where this file omits them.
from sim.lockstep import (  # noqa: F401
    CMD_ENDPOINT,
    SENSOR_ENDPOINT,
    TOPIC_CMD,
    TOPIC_SENSOR,
    TOPIC_TRUTH,
)


class LinkPublisher:
    """Bind-and-broadcast helper for sensor frames plus a command sink."""

    def __init__(
        self,
        sensor_endpoint: str = SENSOR_ENDPOINT,
        cmd_endpoint: str = CMD_ENDPOINT,
    ) -> None:
        """Bind the sensor PUB and command PULL sockets."""
        self._context = zmq.Context.instance()
        self._pub = self._context.socket(zmq.PUB)
        self._pub.bind(sensor_endpoint)
        self._pull = self._context.socket(zmq.PULL)
        self._pull.bind(cmd_endpoint)

    def sensor_frame(
        self, sim_time_s: float, omega_bn_b: tuple[float, float, float], frame: int
    ) -> None:
        """Publish one ``BSK_LINK/SensorFrame`` on the sensor feed."""
        self._pub.send_multipart(
            [TOPIC_SENSOR, bytes(encode_sensor_frame(sim_time_s, omega_bn_b, frame))]
        )

    def heartbeat(self, sim_time_s: float, frame: int) -> None:
        """Publish a sensor frame with zero rate: alive, attitude-quiescent."""
        self.sensor_frame(sim_time_s, (0.0, 0.0, 0.0), frame)

    def recv_command(self, timeout_ms: int = 0) -> TorqueCommand | None:
        """Return the next ``BSK_LINK/TorqueCommand``, or None within the timeout."""
        if not self._pull.poll(timeout_ms, zmq.POLLIN):
            return None
        frames = self._pull.recv_multipart()
        return decode_torque_command(frames[-1])

    def close(self) -> None:
        """Close the bound sockets (context is shared/process-wide)."""
        # Short linger: let queued frames flush to real peers instead of vanishing.
        self._pub.close(linger=250)
        self._pull.close(linger=250)
