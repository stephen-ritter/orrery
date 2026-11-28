"""Rung-1 burn allocation: a body-frame force quota into per-thruster on-times.

The law this module implements is option A (quota): the ground commands a
force along an axis, the craft forwards that quota once per control tick, and
Basilisk's ``ThrusterDynamicEffector`` owns the pulse pattern WITHIN the tick.
The only language the effector understands is
``THRArrayOnTimeCmdMsgPayload.OnTimeRequest[i]`` - seconds thruster ``i``
fires per command period - so the whole world-side job is this one translation::

    on_time_i = tick_s * m_i / u_max_i        sum_i m_i * d_i  ~=  force_B

with ``m_i`` the magnitude allocated to thruster ``i`` and ``d_i`` its unit
direction.  ``ComputeThrusterFire`` then does the real on/off with the
remainder carried across sub-steps; nothing here pulses anything.

The allocation walks the plant's nozzle axes against the force that remains
unallocated: each axis group (devices sharing a direction - a cross-X has one
device per axis, a clustered RCS may have several) may spend the positive
projection of that residual on its axis, capped at the group's combined
rating and split pro-rata by the devices' remaining room.  For the
axis-aligned cross-X sets this bench flies the first pass is exact; residual
iterations keep non-orthogonal geometries honest.  A demand outside the
plant's feasible cone SATURATES - the shortfall is reported, never solved
away: what the hardware cannot deliver is a fact about the plant, not a
debt the physics layer quietly writes off.  (A min-norm pseudo-inverse with
a clip, the tempting first try, fails exactly this test: opposed devices
each take half the demand and cancel it.)
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class BurnAllocation:
    """What the plant will actually do with one tick's force quota.

    ``achieved_force_B`` is what the allocated magnitudes deliver - the
    world's applied ledger counts THIS, never the requested force, so a
    saturated tick is truth-visible as a shortfall rather than a phantom
    impulse.
    """

    on_times_s: np.ndarray
    achieved_force_B: np.ndarray
    saturated: bool

    def scaled(self, ratio: float) -> BurnAllocation:
        """Proportionally shrink this plan: same shape, ratio of the burn.

        Fuel cost is linear in on-time, so scaling a plan by an
        affordability ratio stays a consistent plan: the force, the
        on-times and the fuel they cost all shrink by exactly `ratio`.
        Saturation is a shape, not a size: a shortfall stays true.
        """
        return BurnAllocation(
            on_times_s=self.on_times_s * ratio,
            achieved_force_B=self.achieved_force_B * ratio,
            saturated=self.saturated,
        )


def allocate_on_times(
    directions_B: np.ndarray,
    max_thrusts_N: np.ndarray,
    force_B: np.ndarray,
    tick_s: float,
    *,
    tol_N: float = 1e-9,
) -> BurnAllocation:
    """Translate one tick's force quota into per-thruster on-times.

    Args:
        directions_B: (n, 3) unit thrust directions in the body frame.
        max_thrusts_N: (n,) per-thruster peak force, all strictly positive.
        force_B: (3,) requested body-frame force vector.
        tick_s: the control tick the quota covers; on-times are capped at it.
        tol_N: how far the achieved force may sit from the request before the
            tick counts as saturated.

    Raises:
        ValueError: shapes disagree, a rating is not positive, a direction is
            not a unit vector, or the tick is not a positive span of time.
    """
    dirs = np.asarray(directions_B, dtype=float)
    umax = np.asarray(max_thrusts_N, dtype=float)
    force = np.asarray(force_B, dtype=float)
    if dirs.ndim != 2 or dirs.shape[1] != 3:
        raise ValueError(f"directions must be (n, 3), got shape {dirs.shape}")
    if umax.shape != (dirs.shape[0],):
        raise ValueError(f"max_thrusts must be ({dirs.shape[0]},), got shape {umax.shape}")
    if force.shape != (3,):
        raise ValueError(f"force must be a 3-vector, got shape {force.shape}")
    if np.any(umax <= 0.0):
        raise ValueError("every thruster rating must be strictly positive")
    unit_slack = np.abs(np.linalg.norm(dirs, axis=1) - 1.0)
    if np.any(unit_slack > 1e-6):
        raise ValueError(
            f"thrust directions must be unit vectors (max |d|-1 = {unit_slack.max():.2e})"
        )
    if not tick_s > 0.0:
        raise ValueError(f"tick must be positive, got {tick_s}")

    # Nozzle axes, grouped: devices that share a direction spend that axis's
    # demand pro-rata, so two 5 N devices on +X share +X's bill of 5 N each,
    # never 5 N each against the same Newton. Group order is declaration
    # order: deterministic, digest-safe, plant-declared.
    groups: list[list[int]] = []
    seen: dict[tuple[float, float, float], int] = {}
    for index, direction in enumerate(dirs):
        key = (
            round(float(direction[0]), 9),
            round(float(direction[1]), 9),
            round(float(direction[2]), 9),
        )
        if key in seen:
            groups[seen[key]].append(index)
        else:
            seen[key] = len(groups)
            groups.append([index])

    magnitudes = np.zeros(dirs.shape[0])
    remaining = force.copy()
    for _ in range(len(groups) + 1):
        spent = False
        for group in groups:
            room = umax[group] - magnitudes[group]
            if not np.any(room > tol_N):
                continue
            demand = float(dirs[group[0]] @ remaining)
            if demand <= tol_N:
                continue  # this axis cannot help against the current residual
            delivered = min(demand, float(room.sum()))
            magnitudes[group] += delivered * room / room.sum()
            remaining = force - magnitudes @ dirs
            spent = True
        if np.linalg.norm(remaining) <= tol_N or not spent:
            break

    achieved = magnitudes @ dirs
    on_times = tick_s * magnitudes / umax
    saturated = bool(np.linalg.norm(force - achieved) > tol_N)
    return BurnAllocation(on_times_s=on_times, achieved_force_B=achieved, saturated=saturated)
