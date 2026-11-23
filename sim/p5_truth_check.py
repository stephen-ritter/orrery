"""P5 acceptance: truth on its own ports, and the boundary MEASURED.

Two vehicles (scripts/check_truth_exhibits.sh flies the fleet); per vehicle: a flight
target on the classic pair and a TRUTH target on the gateway port. Proven
through JSON-RPC:

1. all four interfaces CONNECTED and streaming - truth included;
2. TRUTH_STATE_TLM arrives FRESH (SIM_TIME_S advancing - the truth frame
   has no SEQUENCE by design: it is not CCSDS, it never met a satellite);
3. the boundary, both directions at once: truth ω and onboard-knowledge
   ω (0x08C8) DIFFER (they are not the same wire, vector gap >1e-6 at
   strictly paired epochs) yet AGREE (same world, <3e-3 while coasting) -
   bias/noise scale, the provenance rule as a measurement;
3b. the same rule on every LANDED part whose registry row promises a
   ``truth_pair`` exhibit (KN/KN2, registry-driven since R4): flight
   opinion and navigator truth must DIFFER at device scale and AGREE as
   one world - and the scale is DECLARED: the sigmas named by the row's
   ``declared`` map are read from the vehicle's own device block (0.2
   sigma floor, the Gauss-Markov 3-sigma clamp as ceiling), so acceptance
   judges every vehicle by the manifest that built it, not by folklore; a
   vehicle that declares no device is expected to stream no frame at all.
   A landed row whose exhibit this gate cannot honestly execute stops the
   run loudly - a part never registers quietly ahead of its proof;
4. truth watches physics answer the FLIGHT side: FCS_CMD EnableCmd uplinks
   on the flight target only, and the TRUTH stream shows |ω| fall below
   half - the commanded detumble happened in the world, seen by the side
   that has no idea anyone asked;
5. the truth target cannot be commanded at all: the cmd API fails on a
   TRUTH target because truth has no command defs - absence as tripwire.

Freshness is change-verified throughout (the P2 caching lesson).
"""

from __future__ import annotations

import math
import os
import sys
import time
import urllib.error
from pathlib import Path
from typing import cast

from sim.fleet import flight_interface, scenario_vehicles, truth_interface
from sim.fleet_policy import load_fleet
from sim.p2_cosmos_check import login, rpc, wait_interface
from sim.parts import Part, load_registry

# The scenario file IS the fleet (D10): rows are derived, never declared.
# ORRERY_SCENARIO lets a gate point this client at another fleet than the
# operational one - the REG exit flight needs the SAME client for a vehicle
# before and after one part is fitted (a gate you must edit to change its
# input is two gates).
SCENARIO = Path(
    os.environ.get("ORRERY_SCENARIO")
    or Path(__file__).resolve().parents[1] / "sim" / "scenarios" / "twin_ops.toml"
)


def fleet_rows(path: Path = SCENARIO) -> list[tuple[str, str, str, str, str]]:
    """(name, flight target, flight iface, truth target, truth iface) per block."""
    return [
        (v.name, v.cosmos_target, flight_interface(v), v.truth_target, truth_interface(v))
        for v in scenario_vehicles(path)
    ]


# Landed registry rows whose truth_pair exhibit this gate executes (REG).
# What it can execute HONESTLY today: a one-device opinion part declaring
# pos/vel sigma field names, a 3-sigma-clamped ceiling (Basilisk's
# Gauss-Markov bound per component), and strict epochs. A landed row
# promising anything else STOPS the run - a part may register ahead of its
# circuit, never quietly ahead of its proof.
def truth_pair_rows() -> list[Part]:
    """The exhibit rows, straight from registry/parts.toml."""
    rows: list[Part] = []
    for part in load_registry():
        if not part.landed or part.exhibit is None:
            continue
        ex = part.exhibit
        executable = (
            ex.kind == "truth_pair"
            and len(part.bskforge_types) == 1
            and ex.ceiling == "3sqrt3"
            and ex.epoch == "strict"
        )
        if executable:
            if ex.compare == "vector3":
                executable = {"pos_sigma", "vel_sigma"} <= part.declared.keys()
            elif ex.compare == "rotation_mrp":
                executable = "angle_sigma" in part.declared and bool(ex.flight_pos and ex.truth_pos)
            else:
                executable = False  # e.g. CAM's truth-vs-image: joins at CAM
        if not executable:
            raise NotImplementedError(
                f"{part.name}: landed row promises an exhibit this gate cannot "
                f"honestly execute (kind={ex.kind}, compare={ex.compare}, "
                f"ceiling={ex.ceiling}, epoch={ex.epoch}) - implement it here, never skip it"
            )
        rows.append(part)
    return rows


TRUTH_PAIR_ROWS = truth_pair_rows()


def _sigma(block: object, field: str, vehicle: str, part: str) -> float:
    """One bound, exactly as the manifest declared it - or the run stops.

    ``None`` (the schema's optional noise fields) and ``0.0`` are both legal
    TOML and both destroy the exhibit: its two verdicts are multiples of this
    sigma, so with no declared noise the DIFFER verdict passes vacuously - an
    ideal device IS an oracle - and the AGREE verdict has no bound at all.
    A gate cannot grade an unjudgable manifest by pretending otherwise, so it
    names the vehicle, the part and the field instead (KN2's rule for a device
    that was never fitted stays ``None``: absence is a fact, this is a lie).
    """
    value = getattr(block, field)
    if value is None or float(value) <= 0.0:
        raise ValueError(
            f"{vehicle}/{part}: the device block declares {field} = {value!r}, but the "
            f"registered exhibit bounds both of its verdicts in units of {field} - "
            "declare a positive sigma or the part is not judgeable from this manifest"
        )
    return float(value)


def declared_sigmas(
    path: Path = SCENARIO,
) -> dict[str, dict[str, tuple[float, ...] | None]]:
    """Per-vehicle, per-part sigmas AS DECLARED (KN2 discipline, D13).

    The same bskforge loader that builds the plant reads the manifest here:
    each row's ``declared`` map names the schema fields of its device block
    that carry the sigmas, so acceptance judges every vehicle by the manifest
    that built it - this file restates no number a vehicle could outgrow.
    ``None`` means the device was never fitted: absence is declared, not
    zero-filled.

    A manifest pointed at by ``ORRERY_SCENARIO`` is judged by this gate
    without the gate's owner having edited it (the camera exit flight needs
    exactly that), so it is read through the audited loader too: acceptance
    refuses to grade a world whose fleet file smuggled in wiring, even though
    the plant that would build it already refuses to.
    """
    config = load_fleet(path)
    per_vehicle: dict[str, dict[str, tuple[float, ...] | None]] = {}
    for sc in config.spacecraft:
        per_part: dict[str, tuple[float, ...] | None] = {}
        for row in TRUTH_PAIR_ROWS:
            block = next((s for s in sc.sensors if s.type == row.bskforge_types[0]), None)
            if block is None:
                per_part[row.name] = None  # not fitted: declared absence, no bound needed
            elif row.exhibit is not None and row.exhibit.compare == "rotation_mrp":
                angle = _sigma(block, row.declared["angle_sigma"], sc.name, row.name)
                per_part[row.name] = (angle,)
            else:
                per_part[row.name] = (
                    _sigma(block, row.declared["pos_sigma"], sc.name, row.name),
                    _sigma(block, row.declared["vel_sigma"], sc.name, row.name),
                )
        per_vehicle[sc.name] = per_part
    return per_vehicle


FLEET_ROWS = fleet_rows()
SIGMAS = declared_sigmas()


# The registry word "strict": raw velocity pairs may not cross any epoch
# gap. One frame of orbital acceleration is ~8 m/s - three orders of
# magnitude over this fleet's velocity sigma - so a skew beyond this window
# means the samples are simply not comparable (measured, and the reason KN
# only passed while two caches happened to share a millisecond). Position
# still gets the first-order v*dt ride, whose residual is a*dt^2/2 -
# millimetres at one second.
STRICT_EPOCH_S = 0.5


# Attitude pairing is TICK-strict, not merely "same half second": at the
# rates this fleet tumbles (~0.08 rad/s), ONE second of epoch skew is 4
# degrees - four orders of magnitude over a 1e-4 rad star tracker sigma.
# Both wires stamp the shared world tick, so same-instant pairing is exact.
SAME_TICK_S = 1e-6


def _mrp_to_quat(s: tuple[float, float, float]) -> tuple[float, float, float, float]:
    """MRP -> unit quaternion, SCALAR-FIRST (Basilisk MRP2EP/C2EP order)."""
    s2 = sum(c * c for c in s)
    denom = 1.0 + s2
    sx, sy, sz = (2.0 * c / denom for c in s)
    return ((1.0 - s2) / denom, sx, sy, sz)


def _angle_gap_rad(q_a: tuple[float, ...], q_b: tuple[float, ...]) -> float:
    """Rotation angle between two unit quaternions [rad], sign/shadow-agnostic.

    2*acos(|dot|): the antipodal quaternion (same rotation, negated) and the
    MRP shadow set (which negates the quaternion too) both fall out via |dot|,
    so no shadow-selection logic can be wrong here.
    """
    dot = sum(a * b for a, b in zip(q_a, q_b, strict=True))
    return 2.0 * math.acos(min(1.0, abs(dot)))


def _run_rotation_pair(
    token: str,
    name: str,
    flight: str,
    truth: str,
    row: Part,
    sigma: float,
) -> None:
    """rotation_mrp compare: ST quaternion vs navigator MRP, one ANGLE GAP.

    The two wires describe the same rotation in different algebras, so the
    comparison is representation-free: the principal angle between them.
    Bounds are the row's, in units of the sigma the device DECLARED
    (white_noise per axis): under floor*sigma the ST reads an oracle; over
    the 3*sqrt(3)-sphere diagonal the two wires point at different worlds.
    """
    ex = row.exhibit
    assert ex is not None
    err_max = 0.0
    pairs = 0
    end = time.monotonic() + 60.0
    while pairs < 4 and time.monotonic() < end:
        time.sleep(0.25)
        t = sample(token, truth, ex.truth_packet)
        k = sample(token, flight, ex.flight_packet)
        if abs(num(k, "SIM_TIME_S") - num(t, "SIM_TIME_S")) > SAME_TICK_S:
            continue  # one second apart is 4 degrees of rotation - not comparable
        pairs += 1
        q_k = tuple(num(k, f"{ex.flight_pos}_{axis}") for axis in range(4))
        err_max = max(err_max, _angle_gap_rad(q_k, _mrp_to_quat(vec3(t, ex.truth_pos))))
    assert pairs >= 4, (
        f"{name}: truth/{row.name} caches never shared a world tick "
        f"({pairs}/4 pairs) - one stream is stale, not two views of one attitude"
    )
    ceiling = 3.0 * 3.0**0.5 * sigma
    floor = float(ex.floor)
    assert err_max >= floor * sigma, (
        f"{name}: attitude angle gap {err_max:.2e} rad under {floor} sigma of a "
        f"{sigma:.0e} rad device - the flight side reads an oracle, not a star tracker"
    )
    assert err_max <= 1.01 * ceiling, (
        f"{name}: attitude angle gap {err_max:.2e} rad over the 3*sqrt(3)-sigma "
        f"ceiling ({ceiling:.2e} rad) - two worlds pointing two ways?"
    )
    print(
        f"[{name}] attitude truth-vs-{row.name.upper()} (declared sigma {sigma:.0e} rad): "
        f"|Δφ|<= {err_max:.2e} rad - differs at device scale, agrees as one world"
    )


def run_truth_pair(
    token: str,
    name: str,
    flight: str,
    truth: str,
    row: Part,
    sigmas: tuple[float, ...] | None,
) -> None:
    """Execute one registry row's truth_pair exhibit (the REG acceptance loop).

    Structurally the hand-written KN2 block it replaces, with every name and
    bound read from the row: flight and truth packet, the position/velocity
    field pairs, the sigmas already resolved against the vehicle's manifest,
    the oracle floor, and the Gauss-Markov clamp. Fitting a registered part
    on a vehicle therefore arrives with its exhibit: no edit here.
    """
    ex = row.exhibit
    assert ex is not None  # truth_pair_rows() guarantees it
    if sigmas is None:
        print(
            f"[{name}] no {row.bskforge_types[0]} declared - "
            f"{row.eds_container.rsplit('/', 1)[-1]} channel off by manifest "
            "(absence declared, not zero-filled)"
        )
        return
    wait_stream(token, flight, ex.flight_packet, 90.0, f"{name} {row.name}")
    if ex.compare == "rotation_mrp":
        _run_rotation_pair(token, name, flight, truth, row, float(sigmas[0]))
        return
    (pos_sigma, vel_sigma) = sigmas
    dr_max = dv_max = 0.0
    pairs = 0
    end = time.monotonic() + 60.0
    while pairs < 4 and time.monotonic() < end:
        time.sleep(0.25)
        t = sample(token, truth, ex.truth_packet)
        n = sample(token, flight, ex.flight_packet)
        skew = num(n, "SIM_TIME_S") - num(t, "SIM_TIME_S")
        if abs(skew) > STRICT_EPOCH_S:
            continue  # one frame apart is already 8 m/s of falling - not comparable
        pairs += 1
        r_t, v_t = vec3(t, ex.truth_pos), vec3(t, ex.truth_vel)
        r_n, v_n = vec3(n, ex.flight_pos), vec3(n, ex.flight_vel)
        dr = tuple(rn - (rt + vt * skew) for rn, rt, vt in zip(r_n, r_t, v_t, strict=True))
        dv = tuple(vn - vt for vn, vt in zip(v_n, v_t, strict=True))
        dr_max = max(dr_max, norm3(dr))
        dv_max = max(dv_max, norm3(dv))
    assert pairs >= 4, (
        f"{name}: truth/{row.name} caches never shared a comparable epoch "
        f"({pairs}/4 pairs) - one stream is stale, not two sides of one world"
    )
    dr_ceiling = 3.0 * 3.0**0.5 * pos_sigma
    dv_ceiling = 3.0 * 3.0**0.5 * vel_sigma
    floor = float(ex.floor)
    assert dr_max >= floor * pos_sigma, (
        f"{name}: |R_{row.name} - R_truth| = {dr_max:.1f} m against a declared "
        f"sigma of {pos_sigma:.1f} m - under {floor} sigma the flight side "
        "reads an oracle, not a device"
    )
    assert dr_max <= 1.01 * dr_ceiling, (
        f"{name}: |R_{row.name} - R_truth| = {dr_max:.1f} m exceeds the 3-sigma "
        f"clamp ({dr_ceiling:.1f} m) of a {pos_sigma:.1f} m device - "
        "two worlds on one bus?"
    )
    assert floor * vel_sigma <= dv_max <= 1.01 * dv_ceiling, (
        f"{name}: |V_{row.name} - V_truth| = {dv_max:.4f} m/s against a declared "
        f"sigma of {vel_sigma} m/s - not this device's scale"
    )
    print(
        f"[{name}] truth R vs {row.name.upper()} (declared sigma {pos_sigma:.0f} m / "
        f"{vel_sigma} m/s): |ΔR|<= {dr_max:.1f} m, |ΔV|<= {dv_max * 100:.2f} cm/s - "
        "differs at device scale, agrees as one world"
    )


def sample(token: str, target: str, packet: str) -> dict[str, object]:
    """Decoded item dict of the cached packet; raises until it streams."""
    rows = cast("list", rpc(token, "get_tlm_packet", target, packet))
    vals = {str(name): v for name, v, _ in rows}
    if vals.get("SIM_TIME_S") is None:
        raise ValueError(f"no {target} {packet} received yet")
    return vals


def num(vals: dict[str, object], key: str) -> float:
    """Numeric value of a decoded item."""
    return float(str(vals[key]))


def omega_norm(vals: dict[str, object]) -> float:
    """|omega_BN_B| from any frame that carries the rate triplet."""
    parts = [num(vals, f"OMEGA_BN_B_{axis}") for axis in range(3)]
    return sum(v * v for v in parts) ** 0.5


def vec3(vals: dict[str, object], prefix: str) -> tuple[float, float, float]:
    """One f64 triplet from any frame that carries it (defs flatten _0.._2)."""
    return (num(vals, f"{prefix}_0"), num(vals, f"{prefix}_1"), num(vals, f"{prefix}_2"))


def norm3(v: tuple[float, ...]) -> float:
    """|v| of one decoded triplet (len-3 by construction, not by type)."""
    return sum(c * c for c in v) ** 0.5


def wait_stream(
    token: str, target: str, packet: str, timeout: float, label: str
) -> dict[str, object]:
    """Poll until the packet streams twice in a row with advancing sim time."""
    end = time.monotonic() + timeout
    last: float | None = None
    hits = 0
    while time.monotonic() < end:
        try:
            vals = sample(token, target, packet)
        except (RuntimeError, ValueError):
            time.sleep(1)
            continue
        t = num(vals, "SIM_TIME_S")
        if last is not None and t > last:
            hits += 1
            if hits >= 2:
                return vals
        else:
            hits = 0
        last = t
        time.sleep(1)
    raise TimeoutError(f"{label}: {target} {packet} never streamed advancing sim time")


def check_vehicle(
    token: str,
    name: str,
    flight: str,
    flight_iface: str,
    truth: str,
    truth_iface: str,
    sigmas: dict[str, tuple[float, ...] | None],
) -> None:
    """Run the full per-vehicle proof."""
    for iface in (flight_iface, truth_iface):
        row = wait_interface(token, name=iface)
        assert row[1] == "CONNECTED" and row[6] > 0, f"{iface} not streaming: {row}"
    print(f"[{name}] flight {flight_iface} + truth {truth_iface}: both CONNECTED, both streaming")

    # truth is fresh, physical, and orbit-scaled
    t0 = wait_stream(token, truth, "TRUTH_STATE_TLM", 90.0, f"{name} truth")
    w_truth = omega_norm(t0)
    r_norm = sum(num(t0, f"R_BN_N_{a}") ** 2 for a in range(3)) ** 0.5
    # sanity fence, not a vehicle spec: the twin's deputy tumbles hotter than
    # chief by design (0.076 initial), so the old 0.075 ceiling was one
    # plant's folklore, not a boundary rule. Garbage/NaN/zero streams still
    # die here; the REAL per-vehicle proofs are the truth-vs-knowledge gap
    # and the commanded decay below.
    assert 0.001 < w_truth < 0.5, f"{name}: truth |omega| implausible: {w_truth}"
    assert r_norm > 1e5, f"{name}: truth |r| not orbit-scale: {r_norm}"

    # the boundary, measured: differ (two wires) yet agree (one world).
    # R4 hygiene, the KN2 lesson applied to the rate pair too: strictly
    # shared epochs AND a vector difference. knowledge = truth + bias +
    # noise, so |Δω| at one epoch IS the sensor-error scale - while a
    # difference of NORMS over unpaired caches could geometrically cancel
    # (measured: one unlucky cache race squeezed deputy's norm gap to 6.4e-7,
    # under the floor, while every honest gap sits at 4e-6 and up).
    k0 = wait_stream(token, flight, "BSK_LINK_SENSOR_FRAME_TLM", 90.0, f"{name} knowledge")
    gaps: list[float] = []
    pairs = 0
    end = time.monotonic() + 60.0
    while pairs < 4 and time.monotonic() < end:
        time.sleep(0.25)
        t = sample(token, truth, "TRUTH_STATE_TLM")
        k = sample(token, flight, "BSK_LINK_SENSOR_FRAME_TLM")
        if abs(num(k, "SIM_TIME_S") - num(t, "SIM_TIME_S")) > STRICT_EPOCH_S:
            continue  # caches a frame apart drift by their own bias - not comparable
        pairs += 1
        w_t, w_k = vec3(t, "OMEGA_BN_B"), vec3(k, "OMEGA_BN_B")
        gaps.append(norm3(tuple(kn - kt for kn, kt in zip(w_k, w_t, strict=True))))
    assert pairs >= 4, (
        f"{name}: truth/knowledge caches never shared a comparable epoch "
        f"({pairs}/4 pairs) - one stream is stale, not two sides of one world"
    )
    gap = max(gaps)
    assert gap > 1e-6, f"{name}: knowledge == truth on the wire ({gap}) - the boundary leaked"
    assert gap < 3e-3, f"{name}: truth vs knowledge disagree by {gap} - two different worlds?"
    print(
        f"[{name}] truth |ω|={w_truth:.5f} vs knowledge |ω|={omega_norm(k0):.5f}: "
        f"differ by {gap:.2e} rad/s - biased+noisy, same world"
    )

    # REG: every landed part that promised a truth_pair exhibit gets it
    # right here, bounded by the sigmas this vehicle's own manifest declared.
    # The KN2 rule, generalized: DIFFER at device scale (the flight wire is
    # no oracle), AGREE as one world (not two universes), on strictly shared
    # epochs. Rows arrive from registry/parts.toml - fitting the exit demo's
    # star tracker will run this very loop with no edit to this file.
    for row in TRUTH_PAIR_ROWS:
        run_truth_pair(token, name, flight, truth, row, sigmas.get(row.name))

    # truth watches the world answer a command routed ONLY through flight.
    # (FC selection is {"FUNCCODE": n}: the STATE names are labels on the
    # ID_PARAMETER, not item names - "EnableCmd" as a key is unknown item.)
    rpc(token, "cmd", flight, "FCS_CMD", {"FUNCCODE": 1})
    start = omega_norm(sample(token, truth, "TRUTH_STATE_TLM"))
    end = time.monotonic() + 90.0
    now = start
    while time.monotonic() < end and now > 0.5 * start:
        time.sleep(2)
        now = omega_norm(sample(token, truth, "TRUTH_STATE_TLM"))
    assert now <= 0.5 * start, (
        f"{name}: truth |ω| {start:.5f} -> {now:.5f}, commanded damp did not happen"
    )
    print(f"[{name}] FCS via {flight}: TRUTH |ω| {start:.5f} -> {now:.5f} - physics obeyed")

    # the truth target cannot be commanded: no defs, no wire, no way
    try:
        rpc(token, "cmd", truth, "TRUTH_STATE_CMD", {})
        raise AssertionError(f"{truth} accepted a command - truth has no command defs!")
    except (RuntimeError, urllib.error.HTTPError) as exc:
        # HTTP 500 + JSON error body is this CE's cmd-failure idiom (same
        # family as the get_commands trap), so read the body if there is one.
        # Must fail at the DEF layer ("does not exist"), not later on the
        # interface - a connected-but-commandless truth interface would also
        # raise, and would prove nothing about the defs.
        detail = str(exc)
        if isinstance(exc, urllib.error.HTTPError):
            detail += exc.read().decode(errors="replace")
        assert "exist" in detail.lower(), f"{truth} cmd failed for the wrong reason: {detail[:160]}"
        print(f"[{name}] cmd {truth}: refused at the def layer ({str(exc)[:50]}...)")


def main() -> int:
    """Run the two-vehicle truth acceptance; returns process exit code."""
    token = login()
    targets = cast("list[str]", rpc(token, "get_target_names"))
    for _, flight, _, truth, _ in FLEET_ROWS:
        for t in (flight, truth):
            assert t in targets, f"{t} target not installed"
    print(f"targets installed: {len(targets)} (incl. all flight + truth pairs)")

    for name, flight, flight_iface, truth, truth_iface in FLEET_ROWS:
        check_vehicle(token, name, flight, flight_iface, truth, truth_iface, SIGMAS[name])

    print("P5 TRUTH CHECK OK")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except AssertionError as exc:
        print(f"P5 TRUTH CHECK FAILED: {exc}")
        sys.exit(1)
    except TimeoutError as exc:
        print(f"P5 TRUTH CHECK FAILED: {exc}")
        sys.exit(1)
