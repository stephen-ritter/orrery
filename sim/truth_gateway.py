"""Truth gateway: one per vehicle, the ground-side door for Basilisk truth.

Subscribes the sim's ZMQ TRUTH channel (``:5590+20N``, instance-aware) and
serves it as a length-framed TCP stream on the vehicle's truth port
(``5150+2N``) for COSMOS's truth interface to dial. The payload is
**opaque**: the gateway stamps the ground-owned 4-byte header and forwards
the EDS-encoded 144 B container without decoding it - the generated defs
stay the single source of layout truth, exactly like the beacon path.

The boundary is the link idiom, mirrored: truth is TELEMETRY-ONLY by
construction. Inbound bytes on the truth port are counted, refused, and
loudly logged - the same way ``gs_tcp`` treats commands on the beacon.
There is no truth uplink because truth has no actuators.

The gateway is a pure ZMQ observer: it never acks and never blocks the sim
(a PUB drops frames with no subscriber), so it is physics-neutral by
construction - the sim runs identically with the gateway up or down.
"""

from __future__ import annotations

import argparse
import select
import signal
import socket
import struct
import sys
import time

import zmq

from codegen.eds2cosmos import TRUTH_HEADER_BYTES, TRUTH_PACKET_ID
from sim.fleet import ports
from sim.lockstep import TOPIC_TRUTH

assert TRUTH_HEADER_BYTES == 4  # frame contract below; defs say "field + 4"

HEARTBEAT_S = 30.0


def build_frame(payload: bytes) -> bytes:
    """Ground header + opaque payload: [id][reserved][u16 BE length][payload].

    Length convention mirrors the flight beacon's CCSDS field semantics for
    COSMOS's length protocol: the field counts the octets AFTER the header,
    total = field + 4 (constant_field_length).
    """
    return struct.pack(">BBH", TRUTH_PACKET_ID, 0, len(payload)) + payload


class TruthGateway:
    """One ZMQ subscriber in, N TCP clients out, per vehicle."""

    def __init__(self, instance: int, name: str = "") -> None:
        """Bind the truth TCP port and dial this instance's ZMQ truth PUB."""
        block = ports(instance)
        self.label = name or f"instance{instance}"
        self.frames = 0
        self.stray_bytes = 0
        self.stray_warned = False

        self._context = zmq.Context.instance()
        self._sub = self._context.socket(zmq.SUB)
        self._sub.connect(f"tcp://127.0.0.1:{block.truth}")
        self._sub.setsockopt(zmq.SUBSCRIBE, TOPIC_TRUTH)

        self._server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server.bind(("0.0.0.0", block.truth_tcp))
        self._server.listen(4)
        self._server.setblocking(False)

        self._poller = select.poll()
        self._poller.register(self._server.fileno(), select.POLLIN)
        # NB: the SUB socket is NOT fd-polled (see run()); _drain_sub owns it.
        self._clients: dict[int, socket.socket] = {}
        self._log(
            f"truth gateway up: zmq tcp://127.0.0.1:{block.truth} -> "
            f"tcp 0.0.0.0:{block.truth_tcp} (telemetry-only)"
        )

    def _log(self, message: str) -> None:
        print(f"[truth {self.label}] {message}", flush=True)

    def _accept(self) -> None:
        try:
            client, addr = self._server.accept()
        except OSError:
            return
        client.setblocking(False)
        self._clients[client.fileno()] = client
        self._poller.register(client.fileno(), select.POLLIN)
        self._log(f"truth client connected from {addr} ({len(self._clients)} now)")

    def _drop(self, fd: int) -> None:
        client = self._clients.pop(fd, None)
        if client is None:
            return
        try:
            self._poller.unregister(fd)
        except (KeyError, OSError):
            pass
        try:
            client.close()
        except OSError:
            pass

    def _serve_client_read(self, fd: int) -> None:
        """Inbound bytes on the truth port are refused, never acted on."""
        client = self._clients[fd]
        try:
            data = client.recv(4096)
        except (BlockingIOError, InterruptedError):
            return
        except OSError:
            self._log("truth client disconnected")
            self._drop(fd)
            return
        if data:
            self.stray_bytes += len(data)
            if not self.stray_warned:
                self.stray_warned = True
                self._log(
                    f"REFUSING {len(data)} inbound bytes on the truth port: "
                    "this interface is telemetry-only (no truth uplink exists)"
                )
        elif not data:
            self._log("truth client disconnected")
            self._drop(fd)

    def _broadcast(self, frame: bytes) -> None:
        for fd in list(self._clients):
            try:
                self._clients[fd].sendall(frame)
            except (BlockingIOError, InterruptedError):
                continue  # nonblocking send on tiny frames: would block = dead peer
            except OSError as exc:
                self._log(f"broadcast to fd={fd} failed: {exc}; dropping")
                self._drop(fd)

    def run(self) -> None:
        """Pump until SIGTERM. Frames flow gateway-ward only; nothing checks.

        The ZMQ side is DRAINED nonblocking every iteration, not polled by
        fd: ``select.poll`` on a pyzmq fd demonstrably misses readiness
        (measured in the field here - ``zmq.EVENTS`` said POLLIN while the
        fd poll reported nothing). The socket's own state is the truth;
        that is why pyzmq exposes EVENTS in the first place.
        """
        last_beat = time.monotonic()
        while True:
            for fd, event in self._poller.poll(50):
                if not event & select.POLLIN:
                    continue
                if fd == self._server.fileno():
                    self._accept()
                elif fd in self._clients:
                    self._serve_client_read(fd)
            self._drain_sub()
            if time.monotonic() - last_beat >= HEARTBEAT_S:
                last_beat = time.monotonic()
                if self.frames or self._clients:
                    self._log(
                        f"frames={self.frames} clients={len(self._clients)} "
                        f"stray_inbound_bytes={self.stray_bytes}"
                    )

    def _drain_sub(self) -> None:
        """Take every queued truth frame; never block the pump on ZMQ."""
        while True:
            try:
                _, payload = self._sub.recv_multipart(zmq.NOBLOCK)
            except zmq.Again:
                return
            self.frames += 1
            self._broadcast(build_frame(payload))


def main(argv: list[str] | None = None) -> int:
    """Entry point: fleet.sh starts one gateway per vehicle with --instance."""
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--instance", type=int, default=0, help="fleet port block to serve")
    ap.add_argument("--name", default="", help="vehicle name for log lines")
    args = ap.parse_args(argv)

    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    TruthGateway(args.instance, args.name).run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
