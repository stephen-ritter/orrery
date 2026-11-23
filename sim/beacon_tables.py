"""P4 wrap acceptance: the REGENERATED defs work in live COSMOS.

Runs against a live mission + paced lockstep sim (scripts/check_beacon_tables.sh)
and proves through the JSON-RPC API - never a raw socket - that gem 0.1.4's
generated defs decode and command everything P4 put on the wire:

1. target SAT_CHIEF installed with the new commands usable (a missing def
   would fail the cmd API; get_commands RPC itself is broken in this CE),
2. OBC_HK_TLM (0x08CD, P4b-3) streams fresh at 1 Hz and its LITTLE_ENDIAN
   float64/array items decode to physical values (|B| uT-scale, sun seen),
3. BSK_LINK_ONBOARD_STATE_TLM (0x08CC, P4b-1) streams fresh, arrays decoded,
   magnetometer agrees with the OBC's own mirror,
4. NOOP via COSMOS lands: OBC_HK_TLM.CommandCounter increments (uplink path
   through the new packet def),
5. FCS_SET_WHEEL_LIMIT with a WHEEL_LIMIT parameter round-trips: FCS_HK_TLM
   reports the new limit (a parameterised payload through generated defs).

Freshness is SEQUENCE-verified throughout (the TLM server caches values
across mission restarts - the P2 lesson).
"""

from __future__ import annotations

import sys
import time
from typing import cast

from sim.cosmos_api import login, rpc, wait_interface


def sample(token: str, packet: str) -> dict[str, object]:
    """Decoded item dict of the cached packet; raises until it streams."""
    rows = cast("list", rpc(token, "get_tlm_packet", "SAT_CHIEF", packet))
    vals = {str(name): v for name, v, _ in rows}
    if vals.get("SEQUENCE") is None:
        raise ValueError(f"no {packet} packet received yet")
    return vals


def wait_fresh(token: str, packet: str, timeout: float, label: str) -> dict[str, object]:
    """Poll until a genuinely NEW packet of this id lands (SEQUENCE moved)."""
    end = time.monotonic() + timeout
    prev: int | None = None
    while time.monotonic() < end:
        try:
            vals = sample(token, packet)
        except (RuntimeError, ValueError):
            time.sleep(1)
            continue
        seq = int(str(vals["SEQUENCE"]))
        if prev is not None and seq != prev:
            return vals
        prev = seq
        time.sleep(1)
    raise TimeoutError(f"{label}: no fresh {packet} within {timeout}s")


def num(vals: dict[str, object], key: str) -> float:
    """Numeric value of a decoded item."""
    return float(str(vals[key]))


def main() -> int:
    """Run the wrap acceptance; returns process exit code."""
    token = login()
    iface = wait_interface(token)
    print(f"GS_TCP_INT {iface[1]}: rx_bytes={iface[6]} tx_bytes={iface[8]}")

    targets = cast("list[str]", rpc(token, "get_target_names"))
    assert "SAT_CHIEF" in targets, "SAT_CHIEF target not installed"
    # NB: the get_commands RPC answers HTTP 500 in this CE build (settled
    # stack, any plugin state - verified independently against the operator's
    # microservice config and the MinIO bucket objects instead). The command
    # defs' EXISTENCE is proven below by USING them: a missing def would fail
    # the cmd API call outright, which is a stronger proof than a table dump.
    print(f"target SAT_CHIEF installed ({len(targets)} targets)")

    # 2) the OBC's self-report decodes physically (endian/offset proof)
    hk = wait_fresh(token, "OBC_HK_TLM", 90.0, "OBC HK")
    # ONBOARD_FRAMES_SEEN=0 in a FRESH packet is a true report that says
    # "not yet": boot waits here end the moment gs_tcp + FCS init print, and
    # a boot-warm mission is publishing HK before the world's first sensor
    # frame has been decoded and republished onboard. The fact this gate
    # claims is the world's frames ARRIVING, so keep demanding fresh packets
    # until the counter moves - and let the timeout answer "never".
    onboard = int(str(hk["ONBOARD_FRAMES_SEEN"]))
    arm_end = time.monotonic() + 60.0
    while onboard == 0 and time.monotonic() < arm_end:
        time.sleep(2)
        hk = wait_fresh(token, "OBC_HK_TLM", 15.0, "OBC HK")
        onboard = int(str(hk["ONBOARD_FRAMES_SEEN"]))
    field_t = num(hk, "MAG_FIELD_NORM_T")
    sun = num(hk, "SUN_COSINE")
    assert onboard > 0, f"OBC HK saw no onboard frames within the arming window: {onboard}"
    assert 10e-6 < field_t < 60e-6, f"|B| off dipole scale: {field_t}"
    assert -0.001 < sun <= 1.001, f"sun cosine out of range: {sun}"
    assert int(str(hk["FCS_MODE"])) == 0, "fcs mirror should read DISABLED (nobody enabled it)"
    print(f"OBC_HK_TLM fresh: onboard={onboard} frames, |B|={field_t * 1e6:.2f} uT, sun={sun:.2f}")

    # 3) the P4b-1 knowledge frame decodes, and agrees with the OBC mirror
    ob = wait_fresh(token, "BSK_LINK_ONBOARD_STATE_TLM", 90.0, "onboard state")
    b_fields = [num(ob, f"MAG_B_{axis}") for axis in range(3)]
    b_norm = sum(v * v for v in b_fields) ** 0.5
    assert 10e-6 < b_norm < 60e-6, f"ONBOARD |B| off: {b_norm}"
    assert abs(b_norm - field_t) / b_norm < 0.2, (
        f"OBC mirror disagrees with stream: {field_t} vs {b_norm}"
    )
    print(f"ONBOARD state fresh: |B|={b_norm * 1e6:.2f} uT agrees with the OBC's own mirror")

    # 4) NOOP through the NEW command def; OBC counts it in its NEXT report
    before = int(str(sample(token, "OBC_HK_TLM")["COMMAND_COUNTER"]))
    rpc(token, "cmd", "SAT_CHIEF", "OBC_CMD", {})
    print("sent SAT_CHIEF OBC_CMD NOOP via COSMOS cmd API")
    after = before
    end = time.monotonic() + 40.0
    while time.monotonic() < end and after == before:
        time.sleep(1)
        try:
            after = int(str(sample(token, "OBC_HK_TLM")["COMMAND_COUNTER"]))
        except (RuntimeError, ValueError):
            pass
    assert after == before + 1, f"CommandCounter {before} -> {after} (expected exactly +1)"
    print(f"OBC CommandCounter via COSMOS: {before} -> {after}")

    # 5) parameterised uplink: raise the supervisor limit, watch fcs confirm
    rpc(token, "cmd", "SAT_CHIEF", "FCS_SET_WHEEL_LIMIT", {"WHEEL_LIMIT": 610.0})
    print("sent FCS_SET_WHEEL_LIMIT 610.0 via COSMOS cmd API")
    end = time.monotonic() + 60.0  # FCS HK rides its 7.8 s scheduler poll
    limit = 0.0
    while time.monotonic() < end:
        time.sleep(1)
        try:
            limit = num(sample(token, "FCS_HK_TLM"), "WHEEL_LIMIT")
        except (RuntimeError, ValueError, KeyError):
            continue
        if abs(limit - 610.0) < 1e-9:
            break
    assert abs(limit - 610.0) < 1e-9, f"FCS_HK_TLM WHEEL_LIMIT reads {limit}, not 610"
    print(f"FCS_HK_TLM reports WHEEL_LIMIT {limit:.1f} rad/s - uplink through the generated def")

    print("P4 COSMOS WRAP OK")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except AssertionError as exc:
        print(f"P4 COSMOS WRAP FAILED: {exc}")
        sys.exit(1)
    except TimeoutError as exc:
        print(f"P4 COSMOS WRAP FAILED: {exc}")
        sys.exit(1)
