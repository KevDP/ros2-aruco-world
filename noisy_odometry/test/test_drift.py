"""The drift model on synthetic motion, where the exact path is known."""

from __future__ import annotations

import math

import numpy as np
import pytest
from noisy_odometry.drift import (
    DriftingOdometry,
    DriftModel,
    Pose2D,
    advance,
    increment,
    step_variances,
    wrap,
)

EXACT = DriftModel(distance_var_per_m=0.0, heading_var_per_rad=0.0, heading_var_per_m=0.0)
MODEL = DriftModel(distance_var_per_m=9e-4, heading_var_per_rad=4.4e-4, heading_var_per_m=3e-4)


def _straight(length: float, steps: int) -> list[Pose2D]:
    return [Pose2D(length * i / steps, 0.0, 0.0) for i in range(steps + 1)]


def _turn(angle: float, steps: int) -> list[Pose2D]:
    return [Pose2D(0.0, 0.0, wrap(angle * i / steps)) for i in range(steps + 1)]


def _final(model: DriftModel, path: list[Pose2D], seed: int) -> Pose2D:
    odometry = DriftingOdometry(model, seed=seed)
    for pose in path:
        odometry.update(pose)
    return odometry.pose


# ---------------------------------------------------------------------------
# Geometry
# ---------------------------------------------------------------------------


def test_increment_is_measured_along_the_heading() -> None:
    assert increment(Pose2D(0, 0, math.pi / 2), Pose2D(0, 0.1, math.pi / 2)) == pytest.approx(
        (0.1, 0.0)
    )
    assert increment(Pose2D(1, 1, 0.0), Pose2D(1, 1, 0.3)) == pytest.approx((0.0, 0.3))


def test_increment_turns_through_the_shorter_way_across_pi() -> None:
    _, turn = increment(Pose2D(0, 0, 3.1), Pose2D(0, 0, -3.1))
    assert turn == pytest.approx(2 * math.pi - 6.2)


def test_advance_moves_along_the_mid_step_heading() -> None:
    pose = advance(Pose2D(0, 0, 0), distance=1.0, turn=math.pi / 2)
    assert (pose.x, pose.y) == pytest.approx((math.cos(math.pi / 4), math.sin(math.pi / 4)))
    assert pose.yaw == pytest.approx(math.pi / 2)


def test_step_variances_grow_with_the_step() -> None:
    assert step_variances(0.0, 0.0, MODEL) == (0.0, 0.0)
    distance_var, turn_var = step_variances(0.5, 0.2, MODEL)
    assert distance_var == pytest.approx(9e-4 * 0.5)
    assert turn_var == pytest.approx(4.4e-4 * 0.2 + 3e-4 * 0.5)


# ---------------------------------------------------------------------------
# Drift
# ---------------------------------------------------------------------------


def test_without_error_the_path_is_reproduced() -> None:
    path = _straight(1.0, 50) + [Pose2D(1.0, 0.0, wrap(a)) for a in np.linspace(0, 1.5, 30)]
    pose = _final(EXACT, path, seed=0)
    assert (pose.x, pose.y, pose.yaw) == pytest.approx((1.0, 0.0, 1.5), abs=1e-9)


def test_a_robot_standing_still_does_not_drift() -> None:
    pose = _final(MODEL, [Pose2D(0, 0, 0)] * 100, seed=1)
    assert (pose.x, pose.y, pose.yaw) == (0.0, 0.0, 0.0)


def test_the_same_seed_repeats_the_same_drift() -> None:
    path = _straight(2.0, 100)
    assert _final(MODEL, path, seed=7) == _final(MODEL, path, seed=7)


@pytest.mark.parametrize("steps", [10, 200])
def test_distance_drift_depends_on_the_distance_not_the_rate(steps: int) -> None:
    # After 1 m the distance error should spread by sqrt(variance per metre * 1 m), whether
    # odometry came in 10 steps or 200. With 1000 runs the spread is known to about 2%.
    finals = [_final(MODEL, _straight(1.0, steps), seed=s).x for s in range(1000)]
    assert np.std(finals) == pytest.approx(math.sqrt(9e-4), rel=0.1)
    assert np.mean(finals) == pytest.approx(1.0, abs=0.005)


def test_turning_spreads_the_heading_by_the_angle_turned() -> None:
    model = DriftModel(distance_var_per_m=0.0, heading_var_per_rad=4.4e-4, heading_var_per_m=0.0)
    finals = [_final(model, _turn(math.pi, 200), seed=s).yaw for s in range(1000)]
    errors = [wrap(yaw - math.pi) for yaw in finals]
    assert np.std(errors) == pytest.approx(math.sqrt(4.4e-4 * math.pi), rel=0.1)


def test_gathered_variance_matches_the_model() -> None:
    odometry = DriftingOdometry(MODEL, seed=3)
    for pose in _straight(2.0, 80):
        odometry.update(pose)
    assert odometry.distance_var == pytest.approx(9e-4 * 2.0)
    assert odometry.heading_var == pytest.approx(3e-4 * 2.0)


def test_negative_variance_is_rejected() -> None:
    with pytest.raises(ValueError, match="heading_var_per_m"):
        DriftModel(distance_var_per_m=1e-4, heading_var_per_rad=1e-4, heading_var_per_m=-1.0)
