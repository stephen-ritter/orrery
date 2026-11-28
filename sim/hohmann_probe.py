"""Hohmann-rung acceptance: fuel BUDGETS a transfer, the orbit is the verdict.

Requires the running mission (gs_tcp :5050/:5051) and the hohmann plant
flying UNPACED in lockstep (``forge_scenario --zmq --scenario
sim/scenarios/hohmann.toml`` - the world advances as fast as the containers
ack, ~2x wall: a Hohmann coast is half an orbit and the physics will not
hurry). From a ground seat this drives, in order:

1. sense: frames flow, the door is idle, the 5.5 kg tank opens the truth
   stream - the mission's whole budget, visible from tick one;
2. burn 1 (50 N +Y, 36 s near the born prograde point): full authority,
   full service - and then the bench's new instrument: the probe WATCHES
   THE ORBIT on the truth stream, measuring the raised apoapsis and waiting
   for the radial-velocity sign flip. Nothing is predicted: apoapsis is
   DETECTED from the world's own r/v, so burn-shape and J2 effects are
   absorbed into the measurement rather than argued about;
3. burn 2 (50 N -Y body = inertial prograde at apoapsis, 34 s): the
   circularization leg, fired as the flip is seen. Then the verdict the
   whole rung earns: the ECCENTRICITY COLLAPSES (3.0e-3 -> <5e-4) and a
   240 s confirmation coast holds r to a few hundred metres. The orbit
   itself - not any counter - says the second leg bought a circular orbit;
4. burn 3 (50 N +Y, 240 s): the ask 1.93 surviving kg cannot price (6.12
   would be needed... 12.24 for the whole quota). The world buys ~38 ticks,
   throttles, then starves ~200 while the craft's clock forwards every
   frame: claim 15,500 N s against applied ~5,400 - fuel is the mission
   budget, the tank the only complete book, and the craft (fuel-blind by
   design) keeps claiming what it was given.

Same bones as sim/fuel_probe.py (imported, not restated): pulled HK evidence,
freshness by CONTENT, every field decoded through the EDS model. The sim
remains the only place the two ledgers - and the orbit - meet.
"""

from __future__ import annotations

import argparse
import math
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

MU = 398600.4415e9  # Earth, m^3/s^2 - same constant Basilisk's Earth carries
FUEL_LOAD_KG = 5.5  # sim/scenarios/hohmann.toml's declared tank

BURN1_N, BURN1_S = 50.0, 36.0  # 1800 N s: apoapsis +42.7 km (measured)
BURN2_N, BURN2_S = 50.0, 34.0  # 1700 N s at the detected apoapsis: circularize
BURN3_N, BURN3_S = 50.0, 240.0  # prices 12.24 kg against the ~1.93 kg survivors

APOAPSIS_EXPECT_M = 7_042_659.0  # measured offline; the band below is generous
E_CIRCULAR_MAX = 5.0e-4  # measured 1.43e-4; the transfer ellipse sat at 3.0e-3
DR_CONFIRM_MAX_M = 500.0  # measured 58.5 m over 240 s; half-buys wander ~km


def rv(frame: tuple[float, ...]) -> tuple[list[float], list[float]]:
    """The truth frame's r and v as plain lists (indices 1..6 of TruthState)."""
    return list(frame[1:4]), list(frame[4:7])


def eccentricity(r: list[float], v: list[float]) -> float:
    """Two-body e from an instantaneous state - the bench's orbit meter."""
    rn = math.sqrt(sum(x * x for x in r))
    h = [
        r[1] * v[2] - r[2] * v[1],
        r[2] * v[0] - r[0] * v[2],
        r[0] * v[1] - r[1] * v[0],
    ]
    e_vec = [
        (v[1] * h[2] - v[2] * h[1]) / MU - r[0] / rn,
        (v[2] * h[0] - v[0] * h[2]) / MU - r[1] / rn,
        (v[0] * h[1] - v[1] * h[0]) / MU - r[2] / rn,
    ]
    return math.sqrt(sum(x * x for x in e_vec))


def radius(r: list[float]) -> float:
    """|r| of a truth-frame position."""
    return math.sqrt(sum(x * x for x in r))


def radial_rate(r: list[float], v: list[float]) -> float:
    """Radial velocity r-dot: its sign flip between frames IS an apoapsis."""
    return sum(x * y for x, y in zip(r, v, strict=True)) / radius(r)


def main(argv: list[str] | None = None) -> int:
    """Run the Hohmann acceptance; returns the process exit code."""
    ap = argparse.ArgumentParser(description="Hohmann-rung acceptance client")
    ap.add_argument("--instance", type=int, default=0, help="fleet port block to address")
    args = ap.parse_args(argv)

    eds = build_eds(CFS, MISSION_DEFS)
    pkts = {p.name: p for p in eds.packets()}

    block = ports(args.instance)
    client = GroundClient(tlm_port=block.beacon, cmd_port=block.uplink)
    log = BurnLog(pkts)
    truth = TruthWatcher(f"tcp://127.0.0.1:{block.truth}")
    send_hk = pkts["FCS_SEND_HK"]
    print(f"connected; tank {FUEL_LOAD_KG} kg is the mission budget, truth stream subscribed")

    deadline = time.monotonic() + 3600.0  # half an orbit WILL NOT hurry
    stage = "sense"
    fails: list[str] = []
    last_pull = 0.0
    settle_at = 0.0
    prev_rdot: float | None = None
    r_max = 0.0
    r_after2 = 0.0
    t_after2 = 0.0
    dr_confirm = 0.0
    imp_before3 = 0.0

    def pull_hk(force: bool = False) -> None:
        """Demand fresh evidence (the benches' law: HK is PULLED)."""
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

    try:
        while time.monotonic() < deadline and stage != "done":
            pkt = client.recv_packet(min(deadline, time.monotonic() + 0.2))
            truth.poll()
            pull_hk()
            if pkt is not None:
                log.feed(int.from_bytes(pkt[:2], "big"), pkt)
            latest = truth.latest
            if latest is None:
                continue
            r, v = rv(latest)
            rn = radius(r)
            e_now = eccentricity(r, v)

            if stage == "sense":
                if log.last_hk is not None and abs(latest[FUEL_REMAINING] - FUEL_LOAD_KG) < 1e-9:
                    require(
                        int(hk_field(log, "mode")) == 0
                        and int(hk_field(log, "burn_state")) == 0
                        and e_now < 1.0e-4,
                        f"sense: door idle, born circular (e={e_now:.1e}), tank OPEN at "
                        f"{latest[FUEL_REMAINING]:.3f} kg - the whole mission budget",
                    )
                    uplink(burn_packet(pkts, BURN1_N, BURN1_S, (0.0, 1.0, 0.0)))
                    stage = "burn-1"
                    print(f"uplinked burn 1 (prograde): {BURN1_N} N +Y for {BURN1_S} s")
                continue

            if stage == "burn-1":
                if log.count(FCS_APP, EVS_FCS_BURN_DONE_EID) >= 1 and hk_field(log, "last_id") == 1:
                    settle_at = time.monotonic() + 2.0
                    stage = "read-1"
                continue

            if stage == "read-1":
                if time.monotonic() >= settle_at:
                    require(
                        abs(latest[IMPULSE] - BURN1_N * BURN1_S) < 1.0,
                        f"leg 1 bought full authority: applied {latest[IMPULSE]:.1f} N s of "
                        f"{BURN1_N * BURN1_S:.0f} asked, saturated={int(latest[16])}",
                    )
                    require(
                        latest[FUEL_STARVED] == 0.0 and latest[FUEL_THROTTLED] == 0.0,
                        "plentiful fuel binds the first leg not at all",
                    )
                    prev_rdot = radial_rate(r, v)
                    r_max = rn
                    stage = "coast-apo"
                    print(
                        f"coasting to apoapsis (t={latest[0]:.0f} s sim; watching r and the "
                        f"radial-velocity sign, nothing predicted)"
                    )
                continue

            if stage == "coast-apo":
                r_max = max(r_max, rn)
                rdot = radial_rate(r, v)
                if prev_rdot is not None and prev_rdot > 0.0 >= rdot:
                    require(
                        abs(r_max - APOAPSIS_EXPECT_M) <= 0.01 * APOAPSIS_EXPECT_M,
                        f"apoapsis MEASURED on the truth stream: r_max {r_max:.0f} m "
                        f"(expected {APOAPSIS_EXPECT_M:.0f} within 1%), e={e_now:.2e} "
                        f"on the transfer ellipse",
                    )
                    uplink(burn_packet(pkts, BURN2_N, BURN2_S, (0.0, -1.0, 0.0)))
                    stage = "burn-2"
                    print(
                        f"flip at t={latest[0]:.0f} s: uplinked burn 2 at apoapsis "
                        f"({BURN2_N} N body -Y = inertial prograde, {BURN2_S} s)"
                    )
                prev_rdot = rdot
                continue

            if stage == "burn-2":
                if log.count(FCS_APP, EVS_FCS_BURN_DONE_EID) >= 2 and hk_field(log, "last_id") == 2:
                    settle_at = time.monotonic() + 2.0
                    stage = "confirm"
                continue

            if stage == "confirm":
                if time.monotonic() < settle_at:
                    continue
                if not r_after2:
                    r_after2, t_after2 = rn, latest[0]
                    imp2 = latest[IMPULSE]
                    require(
                        abs(imp2 - BURN1_N * BURN1_S - BURN2_N * BURN2_S) < 1.0,
                        f"leg 2 bought full service too: applied {imp2:.0f} N s total",
                    )
                    require(
                        e_now < E_CIRCULAR_MAX,
                        f"THE TRANSFER LANDED: eccentricity collapsed to {e_now:.2e} "
                        f"(ellipse 3.0e-3, circle band {E_CIRCULAR_MAX:.0e})",
                    )
                    require(
                        abs(latest[FUEL_REMAINING] - (FUEL_LOAD_KG - latest[FUEL_SPENT])) < 1e-9
                        and 3.4 < latest[FUEL_SPENT] < 3.75,
                        f"fuel BUDGETED the transfer: {latest[FUEL_SPENT]:.3f} kg bought both "
                        f"legs, {latest[FUEL_REMAINING]:.3f} kg survive - books close",
                    )
                if latest[0] < t_after2 + 240.0:
                    dr_confirm = max(dr_confirm, abs(rn - r_after2))
                    continue
                require(
                    dr_confirm < DR_CONFIRM_MAX_M,
                    f"confirmation coast: r held to {dr_confirm:.0f} m for 240 s of orbit "
                    f"(an ellipse wanders kilometres; this orbit is CIRCULAR by measurement)",
                )
                uplink(burn_packet(pkts, BURN3_N, BURN3_S, (0.0, 1.0, 0.0)))
                imp_before3 = latest[IMPULSE]
                stage = "burn-3"
                print(
                    f"uplinked burn 3: {BURN3_N} N for {BURN3_S} s - priced "
                    f"{BURN3_N * BURN3_S / (100.0 * 9.80665):.2f} kg against "
                    f"{latest[FUEL_REMAINING]:.2f} kg of survivors"
                )
                continue

            if stage == "burn-3":
                if log.count(FCS_APP, EVS_FCS_BURN_DONE_EID) >= 3 and hk_field(log, "last_id") == 3:
                    settle_at = time.monotonic() + 2.0
                    stage = "read-final"
                continue

            if stage == "read-final":
                if time.monotonic() >= settle_at:
                    remaining, spent = latest[FUEL_REMAINING], latest[FUEL_SPENT]
                    starved, throttled = latest[FUEL_STARVED], latest[FUEL_THROTTLED]
                    applied = latest[IMPULSE]
                    claimed = hk_field(log, "claimed")
                    require(
                        remaining == 0.0 and abs(spent - FUEL_LOAD_KG) <= 1e-9,
                        f"the transfer spent the TANK: {spent:.3f} kg booked, remaining "
                        f"{remaining!r} kg to the model's own clamp",
                    )
                    require(
                        190.0 <= starved <= 215.0 and 1.0 <= throttled <= 3.0,
                        f"fuel priced the third ask down: {int(throttled)} throttled, "
                        f"{int(starved)} starved ticks (measured 201/2 offline)",
                    )
                    require(
                        1750.0 <= applied - imp_before3 <= 2050.0,
                        f"the world delivered {applied - imp_before3:.0f} N s of the "
                        f"{BURN3_N * BURN3_S:.0f} the quota asked - the survivors bought ~38 ticks",
                    )
                    require(
                        claimed >= 15400.0 and claimed >= 2.5 * applied,
                        f"the exhibit stands: craft CLAIMS {claimed:.0f} N s, world APPLIED "
                        f"{applied:.0f}, tank exactly zero - fuel was the budget, the tank "
                        f"the only complete book",
                    )
                    require(
                        latest[TRUTH_BURN_ID] == 3.0 and int(hk_field(log, "burn_state")) == 0,
                        "the craft closed a burn it could not fly, on its own clock, again",
                    )
                    print(
                        f"TRANSFER CLOSED: e {e_now:.1e}, claimed {claimed:.0f} N s, applied "
                        f"{applied:.0f}, tank {remaining!r} kg, {int(starved)} starved ticks"
                    )
                    stage = "done"
                continue

            raise AssertionError(f"unknown stage {stage!r}")
    finally:
        client.close()

    if fails:
        print(f"HOHMANN CHECK FAILED ({len(fails)}): " + "; ".join(f for f in fails if f))
        return 1
    print("HOHMANN CHECK PASSED: fuel budgeted the transfer, the orbit is the verdict")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
