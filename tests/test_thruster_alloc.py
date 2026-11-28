"""The allocation math: a tick's force quota into honest on-times, pure Python.

No Basilisk, no wire, no clock: these are the geometry facts the burn bench
relies on, checked where they are cheap. The physics side (a burn that really
raises an apoapsis) lives in tests/test_burn_plant.py.
"""

from __future__ import annotations

import numpy as np
import pytest

from sim.thruster_alloc import allocate_on_times

# The burn bench plant's set: four 5 N devices on +/-X and +/-Y.
CROSS_X_DIRS = np.array([[1.0, 0.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, -1.0, 0.0]])
CROSS_X_UMAX = np.full(4, 5.0)
TICK = 1.0


def test_axis_aligned_demand_is_exact() -> None:
    """A demand along a nozzle axis lands whole on that axis, nothing elsewhere."""
    alloc = allocate_on_times(CROSS_X_DIRS, CROSS_X_UMAX, np.array([0.0, 3.0, 0.0]), TICK)
    assert alloc.on_times_s == pytest.approx([0.0, 0.0, 3.0 / 5.0, 0.0])
    assert alloc.achieved_force_B == pytest.approx([0.0, 3.0, 0.0])
    assert not alloc.saturated


def test_in_cone_diagonal_demand_is_exact() -> None:
    """Two opposed pairs decompose a diagonal without the pairs cancelling it.

    This is the property the tempting pseudo-inverse+clip loses: the min-norm
    fit spreads +X's demand across BOTH +X and -X devices, and clipping the
    negative half leaves half the force on the table.
    """
    alloc = allocate_on_times(CROSS_X_DIRS, CROSS_X_UMAX, np.array([2.0, 3.0, 0.0]), TICK)
    assert alloc.achieved_force_B == pytest.approx([2.0, 3.0, 0.0])
    assert alloc.on_times_s == pytest.approx([2.0 / 5.0, 0.0, 3.0 / 5.0, 0.0])
    assert not alloc.saturated


def test_negative_axis_uses_the_opposing_device() -> None:
    """A -X demand fires the -X device alone: direction is decided by the demand."""
    alloc = allocate_on_times(CROSS_X_DIRS, CROSS_X_UMAX, np.array([-4.0, 0.0, 0.0]), TICK)
    assert alloc.on_times_s == pytest.approx([0.0, 4.0 / 5.0, 0.0, 0.0])
    assert not alloc.saturated


def test_rating_is_the_ceiling_and_says_so() -> None:
    """9 N on a 5 N axis buys 5 N: the shortfall is reported, not invented."""
    alloc = allocate_on_times(CROSS_X_DIRS, CROSS_X_UMAX, np.array([9.0, 0.0, 0.0]), TICK)
    assert alloc.achieved_force_B == pytest.approx([5.0, 0.0, 0.0])
    assert alloc.on_times_s.max() == pytest.approx(TICK)  # never more than the tick
    assert alloc.saturated


def test_unreachable_component_shortfalls_alone() -> None:
    """A cross-X plant cannot push along Z; X and Y still land whole."""
    alloc = allocate_on_times(CROSS_X_DIRS, CROSS_X_UMAX, np.array([3.0, 4.0, 2.0]), TICK)
    assert alloc.achieved_force_B == pytest.approx([3.0, 4.0, 0.0])
    assert alloc.saturated


def test_collinear_devices_share_the_bill() -> None:
    """Two devices on one axis split that axis's demand pro-rata by their room."""
    dirs = np.array([[1.0, 0.0, 0.0], [1.0, 0.0, 0.0], [-1.0, 0.0, 0.0]])
    umax = np.array([2.0, 8.0, 5.0])
    alloc = allocate_on_times(dirs, umax, np.array([5.0, 0.0, 0.0]), TICK)
    assert alloc.achieved_force_B == pytest.approx([5.0, 0.0, 0.0])
    assert alloc.on_times_s == pytest.approx([1.0 / 2.0, 4.0 / 8.0, 0.0])  # 20% + 80%
    assert not alloc.saturated
    # Past the group's combined rating the whole group saturates, full room used.
    alloc = allocate_on_times(dirs, umax, np.array([12.0, 0.0, 0.0]), TICK)
    assert alloc.achieved_force_B == pytest.approx([10.0, 0.0, 0.0])
    assert alloc.on_times_s[:2] == pytest.approx([1.0, 1.0])
    assert alloc.saturated


def test_on_times_never_exceed_the_tick() -> None:
    """Whatever the demand, one tick buys at most one tick of firing."""
    rng = np.random.default_rng(7)
    for _ in range(50):
        demand = rng.normal(0.0, 4.0, 3)
        alloc = allocate_on_times(CROSS_X_DIRS, CROSS_X_UMAX, demand, 0.5)
        assert np.all(alloc.on_times_s >= 0.0)
        assert np.all(alloc.on_times_s <= 0.5 + 1e-12)


def test_allocation_is_deterministic() -> None:
    """Same plant, same quota: bit-identical on-times (digest law)."""
    demand = np.array([1.25, -0.75, 0.0])
    first = allocate_on_times(CROSS_X_DIRS, CROSS_X_UMAX, demand, TICK)
    second = allocate_on_times(CROSS_X_DIRS, CROSS_X_UMAX, demand, TICK)
    assert first.on_times_s.tobytes() == second.on_times_s.tobytes()
    assert first.achieved_force_B.tobytes() == second.achieved_force_B.tobytes()


def test_honest_inputs_or_nothing() -> None:
    """Bad plants and bad asks raise out loud; the allocator never guesses."""
    with pytest.raises(ValueError, match="unit vectors"):
        allocate_on_times(np.array([[2.0, 0.0, 0.0]]), np.array([5.0]), np.zeros(3), TICK)
    with pytest.raises(ValueError, match="strictly positive"):
        allocate_on_times(CROSS_X_DIRS, np.array([5.0, 0.0, 5.0, 5.0]), np.zeros(3), TICK)
    with pytest.raises(ValueError, match="tick"):
        allocate_on_times(CROSS_X_DIRS, CROSS_X_UMAX, np.zeros(3), 0.0)
    with pytest.raises(ValueError, match="3-vector"):
        allocate_on_times(CROSS_X_DIRS, CROSS_X_UMAX, np.array([1.0, 1.0]), TICK)
    with pytest.raises(ValueError, match="max_thrusts"):
        allocate_on_times(CROSS_X_DIRS, np.full(3, 5.0), np.zeros(3), TICK)


def test_scaling_a_plan_is_linear_in_every_book() -> None:
    """BurnAllocation.scaled is the door's affordability shrink (fuel rung).

    One ask, one squeeze: force, on-times and (linearly) fuel cost all move
    by the ratio; direction survives; saturation is a shape, so a shortfall
    stays true and a clean plan stays clean.
    """
    demand = np.array([4.0, 0.0, 0.0])
    plan = allocate_on_times(CROSS_X_DIRS, CROSS_X_UMAX, demand, TICK)
    half = plan.scaled(0.5)
    assert np.allclose(half.on_times_s, 0.5 * plan.on_times_s)
    assert np.allclose(half.achieved_force_B, 0.5 * plan.achieved_force_B)
    assert half.saturated == plan.saturated
    over = allocate_on_times(CROSS_X_DIRS, CROSS_X_UMAX, 100.0 * demand, TICK)
    assert over.scaled(0.5).saturated, "a clipped plan stays clipped when shrunk"
