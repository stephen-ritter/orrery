"""Ground-client test driver: a COSMOS-role client against the gs_tcp gateway.

Proves the generic gateway end to end with zero per-message ground code:
streams raw CCSDS telemetry out of cFS on the BEACON port (:5050), injects an
HK NOOP built from the EDS model on the COMMAND port (:5051), and verifies both
the EVS event packet and the HK command counter increment in live telemetry.
Then the negative half: the same NOOP written to the beacon port must change
nothing — no counter move, and an EVS complaint — because the gateway accepts
commands on the uplink port only. Offsets and MsgIds come from
codegen/eds_model, the same source the COSMOS defs and gs_tcp table use.

Wire format note: CCSDS primary header + cFS time header are big-endian
(the message module encodes them); application payload fields are C struct
memory, i.e. target-native = little-endian on the x86 native build.
"""

from __future__ import annotations

import socket
import time
from pathlib import Path

from codegen.eds2cosmos import build_eds

REPO = Path(__file__).resolve().parents[1]
CFS = REPO / "cfs_mission/cFS"
MISSION_DEFS = REPO / "cfs_mission/mission/orrery_defs/eds"

HK_NOOP_EID = 4  # hk_events.h HK_NOOP_INF_EID
HK_NOOP_FC = 0
GS_TCP_STRAY_RX_EID = 8  # gs_tcp_events.h GS_TCP_STRAY_RX_ERR_EID


def cfs_checksum(header_and_body_wo_checksum: bytes) -> int:
    """CFE longitudinal checksum: sum to zero mod 256 across the command."""
    return (-sum(header_and_body_wo_checksum)) & 0xFF


def build_cmd(msgid: int, funccode: int, payload: bytes = b"", seq: int = 0) -> bytes:
    """Assemble a cFS command: CCSDS primary + FC + checksum + payload."""
    body = bytearray([funccode, 0])
    body += payload
    length = len(body) - 1  # octets after primary header, minus 1
    head = msgid.to_bytes(2, "big") + (0x4000 | seq).to_bytes(2, "big") + length.to_bytes(2, "big")
    packet = bytearray(head) + body
    packet[7] = cfs_checksum(bytes(packet[:7]) + bytes(packet[8:]))
    return bytes(packet)


class GroundClient:
    """Length-framed CCSDS over TCP, speaking to the gs_tcp gateway.

    Two connections, mirroring the satellite: telemetry arrives on the beacon
    port, commands leave on the uplink port.
    """

    def __init__(self, host: str = "127.0.0.1", tlm_port: int = 5050, cmd_port: int = 5051) -> None:
        """Connect the downlink and uplink sessions to the gateway."""
        self.tlm = socket.create_connection((host, tlm_port), timeout=5)
        self.tlm.settimeout(1.0)
        self.cmd = socket.create_connection((host, cmd_port), timeout=5)
        self.tlm_port = tlm_port
        self._buf = b""

    def close(self) -> None:
        """Close both TCP sessions."""
        self.tlm.close()
        self.cmd.close()

    def recv_packet(self, deadline: float) -> bytes | None:
        """Return the next complete CCSDS packet, or None at the deadline."""
        while time.monotonic() < deadline:
            if len(self._buf) >= 6:
                total = int.from_bytes(self._buf[4:6], "big") + 7
                if len(self._buf) >= total:
                    pkt, self._buf = self._buf[:total], self._buf[total:]
                    return bytes(pkt)
            try:
                chunk = self.tlm.recv(65536)
            except TimeoutError:
                continue
            if not chunk:
                raise ConnectionError("gs_tcp closed the connection")
            self._buf += chunk
        return None

    def send_packet(self, data: bytes) -> None:
        """Inject one complete CCSDS command packet (uplink port)."""
        self.cmd.sendall(data)

    def send_on_beacon(self, data: bytes) -> None:
        """Write a command frame to the beacon port on purpose: it must not work."""
        self.tlm.sendall(data)


def main() -> int:
    """Run the ground round trip both ways; returns process exit code."""
    eds = build_eds(CFS, MISSION_DEFS)
    pkts = {p.name: p for p in eds.packets()}
    hk_cmd = pkts["HK_CMD"]
    hk_tlm = pkts["HK_HK_TLM"]
    evs_evt = pkts["CFE_EVS_LONG_EVENT_MSG"]

    app_off = evs_evt.byte_offset(".AppName")
    eid_off = evs_evt.byte_offset(".EventID")
    msg_off = evs_evt.byte_offset(".Message")

    client = GroundClient()
    print(f"connected to gs_tcp, HK_CMD msgid=0x{hk_cmd.msgid:04X}")

    seen_msgids: dict[int, int] = {}
    noop_event = False
    counter_before: int | None = None
    counter_after: int | None = None
    sent = False
    # HK_HK_TLM emits every 7.6 s (sch_lab), so both the before and after
    # counter samples need two full output cycles at least.
    deadline = time.monotonic() + 60.0

    while time.monotonic() < deadline and not (noop_event and counter_after):
        pkt = client.recv_packet(min(deadline, time.monotonic() + 1.0))
        if pkt is None:
            continue
        msgid = int.from_bytes(pkt[:2], "big")
        seen_msgids[msgid] = seen_msgids.get(msgid, 0) + 1

        if msgid == hk_tlm.msgid:
            value = hk_tlm.wire_get(pkt, ".CommandCounter")
            if not sent:
                counter_before = value
            # Wait for the counter to actually move: the first packet after
            # sending can predate the injection landing on the bus.
            elif counter_after is None and counter_before is not None and value != counter_before:
                counter_after = value

        if not sent:
            # hold the NOOP until a HK_TLM baseline is on the books (the packet
            # only comes every 7.6 s, and firing blind left counter_before None)
            if counter_before is not None and len(seen_msgids) >= 3:
                client.send_packet(build_cmd(hk_cmd.msgid, HK_NOOP_FC))
                sent = True
                print("sent HK NOOP via uplink :5051")
            continue

        if msgid == evs_evt.msgid:
            # payload bytes are target-native (little-endian on x86)
            app = pkt[app_off : app_off + 20].split(b"\x00")[0].decode(errors="replace")
            eid = int.from_bytes(pkt[eid_off : eid_off + 2], "little")
            text = pkt[msg_off:].split(b"\x00")[0].decode(errors="replace")
            if app == "HK" and eid == HK_NOOP_EID:
                noop_event = True
                print(f"EVS event via TCP: [{app}/{eid}] {text}")

    # Negative half: the beacon port is a beacon. A command written there must
    # not reach the bus - the counter must freeze and EVS must complain once.
    stray_complaint = False
    stray_freeze_checked = False
    if counter_after is not None:
        client.send_on_beacon(build_cmd(hk_cmd.msgid, HK_NOOP_FC, seq=1))
        print("sent HK NOOP via beacon :5050 (must be refused)")
        stray_deadline = time.monotonic() + 30.0
        while time.monotonic() < stray_deadline and not (stray_complaint and stray_freeze_checked):
            pkt = client.recv_packet(min(stray_deadline, time.monotonic() + 1.0))
            if pkt is None:
                continue
            msgid = int.from_bytes(pkt[:2], "big")
            if msgid == hk_tlm.msgid:
                value = hk_tlm.wire_get(pkt, ".CommandCounter")
                stray_freeze_checked = True
                if value > counter_after:
                    print(f"BEACON INJECTED A COMMAND: counter {counter_after} -> {value}")
                    counter_after = value  # fail below
            if msgid == evs_evt.msgid:
                app = pkt[app_off : app_off + 20].split(b"\x00")[0].decode(errors="replace")
                eid = int.from_bytes(pkt[eid_off : eid_off + 2], "little")
                if app == "GS_TCP" and eid == GS_TCP_STRAY_RX_EID:
                    stray_complaint = True
                    text = pkt[msg_off:].split(b"\x00")[0].decode(errors="replace")
                    print(f"EVS complaint via TCP: [{app}/{eid}] {text}")

    client.close()

    distinct = sorted(seen_msgids)
    print(
        f"distinct msgids streamed: {len(distinct)} -> "
        + ", ".join(f"0x{m:04X}(n={seen_msgids[m]})" for m in distinct[:12])
    )
    ok = (
        noop_event
        and counter_after is not None
        and counter_before is not None  # implied by both above being set; explicit for ty
        and len(distinct) >= 3
        and counter_after == counter_before + 1
        and stray_complaint
        and stray_freeze_checked
    )
    print(f"counter {counter_before} -> {counter_after}; noop event packet received: {noop_event}")
    print(f"beacon-port command refused with EVS complaint: {stray_complaint}")
    print("GROUND ROUND TRIP OK" if ok else "GROUND ROUND TRIP FAILED")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
