"""Fuel-rung acceptance: the tank is the world's, the claim is the craft's.

Requires the running mission (gs_tcp :5050/:5051) and the fuel plant flying
in lockstep (``forge_scenario --zmq --pace --scenario sim/scenarios/fuel.toml``
: the Rung-1 burn vehicle plus a declared 50 g tank). From a ground seat this
drives, in order:

1. sense: frames flow, the door is idle, and the truth stream opens with the
   tank's declared load - fuel is a WORLD fact from tick one;
2. burn 1 (3 N +Y, 6 s): full service. Both ledgers rise together and the
   tank's books show the honest native drain; nothing throttled, nothing
   starved - fuel changes nothing while it is plentiful, which is exactly
   what the Rung-1 verdicts promised and what this bench must not break;
3. burn 2 (5 N +Y, 60 s): the over-ask. 306 g of propellant cannot be bought
   with 50 g: the world throttles, clamps the stock to EXACTLY zero, and
   starves the rest of the quota - while the craft, which owns no fuel
   sensor and can own no fuel claim, keeps forwarding frames and keeps
   claiming every N s it was given. Claim against applied diverges by
   design; the tank is the only book that was ever complete;
4. burn 3 (3 N +Y, 3 s): dry stays dry. A fresh quota after an empty tank
   adds to the claim and adds NOTHING to applied: the world does not thrust
   on empty credit, and the craft's clock still closes the burn it could not
   fly - the silence of a vehicle that ran out, told by two ledgers.

Same bones as sim/burn_probe.py (evidence PULLED via FCS_SEND_HK; freshness
by CONTENT; every field decoded through the EDS model) - and it imports that
probe's primitives rather than restating them: same wire, same ledgers, one
more chapter. The sim remains the only place both ledgers meet.
"""

from __future__ import annotations

import argparse
import time

from codegen.eds2cosmos import build_eds
from sim.burn_probe import (
    EVS_FCS_BURN_DONE_EID,
    FCS_APP,
    FUEL_REMAINING,
    FUEL_SPENT,
    FUEL_STARVED,
    FUEL_THROTTLED,
    IMPULSE,
    TRUTH_BURN_ID,
    BurnLog,
    TruthWatcher,
    burn_packet,
    hk_field,
)
from sim.fleet import ports
from sim.ground_probe import CFS, MISSION_DEFS, GroundClient, build_cmd

FUEL_LOAD_KG = 0.05  # sim/scenarios/fuel.toml's declared tank
G0 = 9.80665
ISPS = 100.0  # the bench plant's Blank_Thruster factory default, measured

BURN1_N, BURN1_S = 3.0, 6.0
BURN2_N, BURN2_S = 5.0, 60.0  # prices 306 g at factory Isp: six tanks' worth
BURN3_N, BURN3_S = 3.0, 3.0


def main(argv: list[str] | None = None) -> int:
    """Run the fuel acceptance; returns the process exit code."""
    ap = argparse.ArgumentParser(description="fuel-rung acceptance client")
    ap.add_argument("--instance", type=int, default=0, help="fleet port block to address")
    args = ap.parse_args(argv)

    eds = build_eds(CFS, MISSION_DEFS)
    pkts = {p.name: p for p in eds.packets()}

    block = ports(args.instance)
    client = GroundClient(tlm_port=block.beacon, cmd_port=block.uplink)
    log = BurnLog(pkts)
    truth = TruthWatcher(f"tcp://127.0.0.1:{block.truth}")
    send_hk = pkts["FCS_SEND_HK"]
    print(f"connected; tank declared {FUEL_LOAD_KG * 1000:.1f} g, truth stream subscribed")

    deadline = time.monotonic() + 300.0
    stage = "sense"
    fails: list[str] = []
    last_pull = 0.0
    settle_at = 0.0
    applied_before_b3 = 0.0
    claimed_before_b3 = 0.0

    def pull_hk(force: bool = False) -> None:
        """Demand fresh evidence (the burn bench's law: HK is PULLED)."""
        nonlocal last_pull
        now = time.monotonic()
        if force or now - last_pull >= 1.0:
            client.send_packet(build_cmd(send_hk.msgid, 0))
            last_pull = now

    def uplink(data: bytes) -> None:
        client.send_packet(data)
        pull_hk(force=True)

    def require(cond: bool, verdict: str) -> None:
        """Fail LOUD but keep flying: a half-run hides a ledger."""
        if cond:
            print(verdict)
        else:
            fails.append(verdict)
            print(f"FAIL: {verdict}")

    def truth_at(index: int) -> float:
        """One row of the newest truth frame; 0.0 before the first frame."""
        return 0.0 if truth.latest is None else truth.latest[index]

    def fuel_now() -> tuple[float, float, float, float] | None:
        if truth.latest is None:
            return None
        return (
            truth.latest[FUEL_REMAINING],
            truth.latest[FUEL_SPENT],
            truth.latest[FUEL_STARVED],
            truth.latest[FUEL_THROTTLED],
        )

    try:
        while time.monotonic() < deadline and stage != "done":
            pkt = client.recv_packet(min(deadline, time.monotonic() + 0.2))
            truth.poll()
            pull_hk()
            if pkt is not None:
                log.feed(int.from_bytes(pkt[:2], "big"), pkt)
            hk_ready = log.last_hk is not None
            fuel = fuel_now()

            if stage == "sense":
                if len(log.sensor_frames) >= 3 and hk_ready and fuel is not None:
                    remaining = fuel[0]
                    require(
                        int(hk_field(log, "mode")) == 0
                        and int(hk_field(log, "burn_state")) == 0
                        and abs(remaining - FUEL_LOAD_KG) < 1e-9,
                        f"sense: door idle, loop DISABLED, tank OPEN at its declared load "
                        f"({remaining * 1000:.3f} g) on the truth stream",
                    )
                    uplink(burn_packet(pkts, BURN1_N, BURN1_S, (0.0, 1.0, 0.0)))
                    stage = "burn-full"
                    print(f"uplinked burn 1: {BURN1_N} N +Y for {BURN1_S} s (plentiful fuel)")
                continue

            if stage == "burn-full":
                if log.saw(FCS_APP, EVS_FCS_BURN_DONE_EID) and hk_field(log, "last_id") == 1:
                    remaining, spent, starved, throttled = fuel_now() or (float("nan"),) * 4
                    price = BURN1_N * BURN1_S / (ISPS * G0)
                    applied = truth_at(IMPULSE)
                    require(
                        abs(applied - BURN1_N * BURN1_S) <= 1.5 * BURN1_N,
                        f"burn 1 applied in full: truth {applied:.1f} N s "
                        f"(claim {hk_field(log, 'claimed'):.1f})",
                    )
                    require(
                        starved == 0.0 and throttled == 0.0,
                        f"plentiful fuel binds nothing: starved={int(starved)} "
                        f"throttled={int(throttled)}",
                    )
                    require(
                        abs(spent - price) <= 0.25 * price,
                        f"native drain within the pulse-shape band: spent {spent * 1000:.3f} g "
                        f"against the linear price {price * 1000:.3f} g",
                    )
                    require(
                        abs(remaining - (FUEL_LOAD_KG - spent)) < 1e-9,
                        "the tank's books close: remaining == load - spent",
                    )
                    uplink(burn_packet(pkts, BURN2_N, BURN2_S, (0.0, 1.0, 0.0)))
                    stage = "burn-dry"
                    print(
                        f"uplinked burn 2: {BURN2_N} N +Y for {BURN2_S} s (prices "
                        f"~{BURN2_N * BURN2_S / (ISPS * G0) * 1000:.0f} g against "
                        f"{remaining * 1000:.1f} g of stock)"
                    )
                continue

            if stage == "burn-dry":
                if log.count(FCS_APP, EVS_FCS_BURN_DONE_EID) >= 2 and hk_field(log, "last_id") == 2:
                    # Settle: let the world's last starved ticks ride the
                    # truth wire before the books are closed (evidence is
                    # pulled, and pulled AFTER the actors have had their say).
                    settle_at = time.monotonic() + 2.0
                    stage = "read-dry"
                continue

            if stage == "read-dry":
                if time.monotonic() >= settle_at:
                    remaining, spent, starved, throttled = fuel_now() or (float("nan"),) * 4
                    applied = truth_at(IMPULSE)
                    claimed = hk_field(log, "claimed")
                    require(
                        remaining == 0.0,
                        f"the tank is EXACTLY empty: {remaining!r} kg (the model's own clamp)",
                    )
                    require(
                        abs(spent - FUEL_LOAD_KG) <= 1e-9,
                        f"the whole declared load was spent: {spent * 1000:.3f} g",
                    )
                    require(
                        starved >= 20 and 1 <= throttled <= 3,
                        f"the door's life cycle: throttled={int(throttled)} (the squeeze), "
                        f"starved={int(starved)} (the silence)",
                    )
                    require(
                        30.0 < applied <= 70.0,
                        f"fuel bounded the service: applied stopped at {applied:.1f} N s "
                        f"though the asks promised {BURN1_N * BURN1_S + BURN2_N * BURN2_S:.0f}",
                    )
                    require(
                        claimed >= 280.0 and claimed >= 3.0 * applied,
                        f"the divergence is the exhibit: craft CLAIMS {claimed:.1f} N s, "
                        f"world APPLIED {applied:.1f} - fuel is the world's book alone",
                    )
                    require(
                        int(hk_field(log, "burn_state")) == 0 and truth_at(TRUTH_BURN_ID) == 2.0,
                        "the craft closed a burn it could not fly: its clock, not the world's",
                    )
                    uplink(burn_packet(pkts, BURN3_N, BURN3_S, (0.0, 1.0, 0.0)))
                    applied_before_b3 = applied
                    claimed_before_b3 = claimed
                    stage = "stay-dry"
                    print(f"uplinked burn 3: {BURN3_N} N +Y for {BURN3_S} s against an empty tank")
                continue

            if stage == "stay-dry":
                if log.count(FCS_APP, EVS_FCS_BURN_DONE_EID) >= 3 and hk_field(log, "last_id") == 3:
                    settle_at = time.monotonic() + 2.0
                    stage = "read-final"
                continue

            if stage == "read-final":
                if time.monotonic() >= settle_at:
                    remaining, spent, starved, throttled = fuel_now() or (float("nan"),) * 4
                    applied = truth_at(IMPULSE)
                    claimed = hk_field(log, "claimed")
                    require(
                        remaining == 0.0 and abs(spent - FUEL_LOAD_KG) <= 1e-9,
                        f"dry stays dry: {remaining!r} kg, {spent * 1000:.1f} g spent",
                    )
                    require(
                        applied == applied_before_b3,
                        f"the world did not thrust on empty credit: applied frozen at "
                        f"{applied:.3f} N s through a whole fresh quota",
                    )
                    require(
                        starved >= 22,
                        f"a fresh quota met the same dry door: starved={int(starved)}",
                    )
                    require(
                        claimed >= claimed_before_b3 + BURN3_N,
                        f"and the claim kept rising anyway: {claimed:.1f} N s claimed "
                        f"against {applied:.1f} applied",
                    )
                    print(
                        f"FUEL LEDGER CLOSED: claimed {claimed:.1f} N s, applied "
                        f"{applied:.1f}, tank 0.0 kg, {int(starved)} starved ticks"
                    )
                    stage = "done"
                continue

            raise AssertionError(f"unknown stage {stage!r}")
    finally:
        client.close()

    if fails:
        print(f"FUEL CHECK FAILED ({len(fails)}): " + "; ".join(fails))
        return 1
    print("FUEL CHECK PASSED: the tank sells, the world withholds, both ledgers tell it")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
