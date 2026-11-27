"""COSMOS round-trip acceptance: drive the live mission via COSMOS CE JSON-RPC.

Proves the generated defs are good enough for the real ground system with
zero hand-written packet definitions:
  1. interface GS_TCP_INT connected and streaming telemetry,
  2. SAT_CHIEF OBC_CMD NOOP sent through the COSMOS command API,
  3. the NOOP shows up in COSMOS-decoded OBC_HK_TLM.CommandCounter
     (which also proves the generated item endianness is right).

Auth: password from $OPENC3_API_PASSWORD (set via ``openc3.sh cli
setpassword``), exchanged for a session token at /openc3-api/auth/verify.
"""

from __future__ import annotations

import json
import os
import sys
import time
import urllib.error
import urllib.request
from typing import cast

BASE = os.environ.get("OPENC3_URL", "http://localhost:2900")
SCOPE = "DEFAULT"
PASSWORD = os.environ.get("OPENC3_API_PASSWORD", "orrery-dev")


def _post(path: str, body: dict | str | None, headers: dict[str, str] | None = None) -> str:
    if isinstance(body, dict):
        data: bytes | None = json.dumps(body).encode()
        ctype = "application/json"
    else:
        data = body.encode() if body is not None else None
        ctype = "application/x-www-form-urlencoded"
    req = urllib.request.Request(
        f"{BASE}{path}",
        method="POST",
        data=data,
        headers={"Content-Type": ctype, **(headers or {})},
    )
    with urllib.request.urlopen(req, timeout=15) as resp:
        return resp.read().decode()


def login() -> str:
    """Exchange the API password for a session token."""
    return _post("/openc3-api/auth/verify", f"password={PASSWORD}").strip().strip('"')


def rpc(token: str, method: str, *params: object, **keyword_params: object) -> object:
    """One OpenC3 JSON-RPC call; raises on error responses."""
    body = {
        "jsonrpc": "2.0",
        "id": int(time.time() * 1000) % 1_000_000,
        "method": method,
        "params": list(params),
        "keyword_params": {"scope": SCOPE, **keyword_params},
    }
    # CE v6 expects the bare session token in Authorization (no Bearer prefix)
    reply = json.loads(_post("/openc3-api/api", body, {"Authorization": token}))
    if "error" in reply:
        raise RuntimeError(f"{method}: {reply['error']}")
    return reply["result"]


def wait_interface(token: str, timeout: float = 120.0, name: str = "GS_TCP_INT") -> list:
    """Poll until the named interface is connected and streaming packets.

    get_all_interface_info returns rows:
    [name, state, rxcount, txcount, rxbytes, txbytes, ...]
    """
    end = time.monotonic() + timeout
    last: list = []
    while time.monotonic() < end:
        try:
            rows = cast("list[list]", rpc(token, "get_all_interface_info"))
        except (urllib.error.URLError, ConnectionError, TimeoutError, RuntimeError):
            rows = []
        for row in rows:
            if row and row[0] == name:
                last = row
                if row[1] == "CONNECTED" and row[6] > 0:  # [6] = rx bytes
                    return row
        time.sleep(2)
    raise TimeoutError(f"{name} never connected/streamed: {last}")


def main() -> int:
    """Run the acceptance flow; returns process exit code."""
    token = login()
    iface = wait_interface(token)
    print(f"GS_TCP_INT {iface[1]}: rx_bytes={iface[6]} tx_bytes={iface[8]}")

    targets = cast("list[str]", rpc(token, "get_target_names"))
    assert "SAT_CHIEF" in targets, "SAT_CHIEF target not installed"
    print(f"targets installed: {len(targets)} (incl. SAT_CHIEF)")

    def sample() -> tuple[int, int]:
        """(CommandCounter, CCSDS sequence) of the cached OBC_HK_TLM values."""
        rows = cast("list", rpc(token, "get_tlm_packet", "SAT_CHIEF", "OBC_HK_TLM"))
        vals = {name: v for name, v, _ in rows}
        raw, seq = vals.get("COMMAND_COUNTER"), vals.get("SEQUENCE")
        if raw is None or seq is None:
            raise ValueError("no OBC_HK_TLM packet received yet")
        return int(str(raw)), int(str(seq))

    # The TLM server CACHES item values across mission restarts: the first
    # read after this script restarts the mission can still be the PREVIOUS
    # mission's value (that false-failed one round trip: 1 -> 1). Proof of
    # freshness: the CCSDS sequence restarts at boot and strictly increases,
    # so any CHANGE of the pair means a packet of THIS mission landed.
    before: int | None = None
    prev: tuple[int, int] | None = None
    end = time.monotonic() + 30.0
    while before is None and time.monotonic() < end:
        try:
            cur = sample()
        except (RuntimeError, ValueError):
            cur = None
        if cur is not None:
            if prev is not None and cur != prev:
                before = cur[0]
            prev = cur
        if before is None:
            time.sleep(1)
    if before is None:
        print("never received a fresh OBC_HK_TLM packet via COSMOS")
        return 1
    # 2026-11-27 purge: this round trip used to poke the bundle's HK app.
    # The mission's own OBC housekeeping answers instead - a better witness
    # anyway, because the NOOP crosses the command path the operator flies.
    rpc(token, "cmd", "SAT_CHIEF", "OBC_CMD", {})  # FC default 0 == NoopCmd
    print("sent SAT_CHIEF OBC_CMD NOOP via COSMOS cmd API")

    after = before
    end = time.monotonic() + 40.0
    while time.monotonic() < end and after <= before:
        time.sleep(1)
        try:
            after = sample()[0]
        except (RuntimeError, ValueError):
            pass  # HK arrives ~every second; no newer packet sampled yet

    print(f"CommandCounter via COSMOS: {before} -> {after}")
    ok = after > before
    print("COSMOS ROUND TRIP OK" if ok else "COSMOS ROUND TRIP FAILED")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
