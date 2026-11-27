"""Truth gateway framing tests: the ground header is a contract.

No sockets here - the gateway's wire shape is what COSMOS's length protocol
and the generated defs were both written against, so it is pinned directly:
[packet id][reserved][u16 BE length = octets after header] + opaque payload.
"""

from __future__ import annotations

import struct

from codegen.eds2cosmos import TRUTH_HEADER_BYTES, TRUTH_PACKET_ID
from sim.fleet import truth_target
from sim.truth_gateway import build_frame

TRUTH_PAYLOAD_BYTES = 144  # BSK_LINK/TruthState: 18 LE float64 (14 + burn ledger)


def test_frame_header_matches_the_generated_def_and_protocol() -> None:
    """The one header every reader agrees on: def, length protocol, wire."""
    payload = bytes(range(TRUTH_PAYLOAD_BYTES))
    frame = build_frame(payload)
    assert len(frame) == TRUTH_HEADER_BYTES + TRUTH_PAYLOAD_BYTES == 148
    packet_id, reserved, length = struct.unpack_from(">BBH", frame)
    assert packet_id == TRUTH_PACKET_ID == 1  # def: APPEND_ID_ITEM PACKET_ID 8 UINT 1
    assert reserved == 0  # def: HEADER_RESERVED "zero on the wire"
    # length_protocol read: total = field + 4  <=>  field = octets after header
    assert length == TRUTH_PAYLOAD_BYTES
    assert frame[TRUTH_HEADER_BYTES:] == payload  # payload forwarded OPAQUE


def test_truth_target_rule_is_stable_across_rename_input() -> None:
    """The derive-rule survives whatever a vehicle's flight target is named."""
    assert truth_target("SAT_CHIEF") == "TRUTH_CHIEF"
    assert truth_target("SAT_DEPUTY") == "TRUTH_DEPUTY"
    # a vehicle whose target never carried the SAT_ prefix keeps its name:
    assert truth_target("OBC3") == "TRUTH_OBC3"
