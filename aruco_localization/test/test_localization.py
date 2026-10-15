"""Fixes from synthetic detections, where the robot's true pose is known."""

from __future__ import annotations

import math

import numpy as np
import pytest
from aruco_localization.localization import (
    Fix,
    Uncertainty,
    build_marker_map,
    covariance,
    invert,
    locate,
    matrix_from_quaternion,
    pose_matrix,
    yaw_quaternion,
)

# Marker 0 of the simulated room: on a post at 0.6 m, facing back along -x.
MARKER_0 = [0.5, 0.0, 0.6, 0.5, -0.5, -0.5, 0.5]
# The simulated camera: 0.17 m ahead of base_footprint, 0.25 m up, optical frame (z forward).
OPTICAL = (-0.5, 0.5, -0.5, 0.5)  # rpy (-pi/2, 0, -pi/2)
BASE_FROM_CAMERA = pose_matrix((0.17, 0.0, 0.25), OPTICAL)
UNCERTAINTY = Uncertainty(range_per_m=0.01, angle_per_m=0.02)


def _robot(x: float, y: float, yaw: float, pitch: float = 0.0) -> np.ndarray:
    """map_from_base for a robot at (x, y) with a heading, optionally tipped forward."""
    cy, sy, cp, sp = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch)
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    transform = np.eye(4)
    transform[:3, :3] = rz @ ry
    transform[:3, 3] = (x, y, 0.0)
    return transform


def _seen(map_from_marker: np.ndarray, map_from_base: np.ndarray) -> np.ndarray:
    """What the camera would detect: the marker's pose in the camera's optical frame."""
    return invert(map_from_base @ BASE_FROM_CAMERA) @ map_from_marker


def _fix(sight: float, distance: float = 2.0) -> Fix:
    return Fix(x=0.0, y=0.0, yaw=0.0, distance=distance, sight=sight)


# ---------------------------------------------------------------------------
# Transforms
# ---------------------------------------------------------------------------


def test_quaternion_of_a_quarter_turn_about_z() -> None:
    rotation = matrix_from_quaternion(0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
    assert rotation == pytest.approx(np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]))


def test_a_rounded_quaternion_still_gives_a_rotation() -> None:
    rotation = matrix_from_quaternion(0.5, -0.5, -0.5, 0.5001)
    assert rotation @ rotation.T == pytest.approx(np.eye(3))


def test_invert_undoes_a_transform() -> None:
    transform = pose_matrix((1.0, -2.0, 0.5), (0.1, 0.2, 0.3, 0.9))
    assert invert(transform) @ transform == pytest.approx(np.eye(4))


@pytest.mark.parametrize("yaw", [0.0, 1.0, -2.5])
def test_yaw_quaternion_turns_about_the_vertical(yaw: float) -> None:
    rotation = matrix_from_quaternion(*yaw_quaternion(yaw))
    assert math.atan2(rotation[1, 0], rotation[0, 0]) == pytest.approx(yaw)


# ---------------------------------------------------------------------------
# The marker map
# ---------------------------------------------------------------------------


def test_marker_map_builds_one_transform_per_marker() -> None:
    markers = build_marker_map({0: MARKER_0})
    # The marker's z axis, out of the printed side, faces back along -x toward the room.
    assert markers[0][:3, 2] == pytest.approx([-1.0, 0.0, 0.0])
    assert markers[0][:3, 3] == pytest.approx([0.5, 0.0, 0.6])


@pytest.mark.parametrize(
    ("values", "message"),
    [([0.5, 0.0, 0.6, 0.0, 0.0, 1.0], "expected"), ([0.5, 0.0, 0.6, 0.0, 0.0, 0.0, 0.0], "zero")],
)
def test_marker_map_rejects_bad_entries(values: list[float], message: str) -> None:
    with pytest.raises(ValueError, match=message):
        build_marker_map({7: values})


# ---------------------------------------------------------------------------
# Fixes
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    ("x", "y", "yaw"), [(-1.5, 0.0, 0.0), (-0.5, 0.8, -0.4), (-2.0, -1.0, 0.5)]
)
def test_locate_recovers_the_true_pose(x: float, y: float, yaw: float) -> None:
    marker = build_marker_map({0: MARKER_0})[0]
    fix = locate(marker, _seen(marker, _robot(x, y, yaw)), BASE_FROM_CAMERA)
    assert (fix.x, fix.y, fix.yaw) == pytest.approx((x, y, yaw), abs=1e-9)


def test_locate_reports_distance_and_line_of_sight() -> None:
    marker = build_marker_map({0: MARKER_0})[0]
    fix = locate(marker, _seen(marker, _robot(-1.5, 0.0, 0.0)), BASE_FROM_CAMERA)
    # Camera at (-1.33, 0, 0.25), marker at (0.5, 0, 0.6): straight ahead along +x.
    assert fix.distance == pytest.approx(math.hypot(1.83, 0.35))
    assert fix.sight == pytest.approx(0.0, abs=1e-9)


def test_a_tilted_estimate_keeps_its_heading() -> None:
    marker = build_marker_map({0: MARKER_0})[0]
    fix = locate(marker, _seen(marker, _robot(-1.5, 0.2, 0.3, pitch=0.05)), BASE_FROM_CAMERA)
    assert fix.yaw == pytest.approx(0.3, abs=1e-9)


def test_an_orientation_error_moves_the_fix_across_the_sight() -> None:
    # The detected marker turned by 1 degree about the vertical swings the camera on an arc:
    # the fix moves sideways by about distance x angle, and barely along the line of sight.
    marker = build_marker_map({0: MARKER_0})[0]
    seen = _seen(marker, _robot(-1.5, 0.0, 0.0))
    angle = math.radians(1.0)
    turn = np.eye(4)
    turn[:3, :3] = matrix_from_quaternion(0.0, math.sin(angle / 2), 0.0, math.cos(angle / 2))
    fix = locate(marker, seen @ turn, BASE_FROM_CAMERA)  # about the marker's own vertical axis
    distance = math.hypot(1.83, 0.35)
    assert abs(fix.y) == pytest.approx(distance * angle, rel=0.25)
    assert abs(fix.x + 1.5) < 0.1 * abs(fix.y)


# ---------------------------------------------------------------------------
# Covariance
# ---------------------------------------------------------------------------


def test_covariance_lies_along_and_across_the_sight() -> None:
    cov = covariance(_fix(sight=0.0, distance=2.0), UNCERTAINTY)
    along, across, angle = 0.01 * 2.0, 2.0 * (0.02 * 2.0), 0.02 * 2.0
    assert np.diag(cov) == pytest.approx([along**2, across**2, angle**2])
    assert cov[0, 1] == pytest.approx(0.0)


def test_covariance_turns_with_the_sight() -> None:
    ahead = covariance(_fix(sight=0.0), UNCERTAINTY)
    left = covariance(_fix(sight=math.pi / 2), UNCERTAINTY)
    assert left[0, 0] == pytest.approx(ahead[1, 1])
    assert left[1, 1] == pytest.approx(ahead[0, 0])


def test_covariance_grows_faster_across_than_along() -> None:
    near = covariance(_fix(sight=0.0, distance=1.0), UNCERTAINTY)
    far = covariance(_fix(sight=0.0, distance=3.0), UNCERTAINTY)
    assert far[0, 0] / near[0, 0] == pytest.approx(9.0)  # along: distance squared
    assert far[1, 1] / near[1, 1] == pytest.approx(81.0)  # across: distance to the fourth


@pytest.mark.parametrize("sight", [0.3, 1.7, -2.9])
def test_covariance_is_symmetric_and_positive_definite(sight: float) -> None:
    cov = covariance(_fix(sight=sight), UNCERTAINTY)
    assert cov == pytest.approx(cov.T)
    assert np.all(np.linalg.eigvalsh(cov) > 0)


def test_uncertainty_must_be_positive() -> None:
    with pytest.raises(ValueError, match="angle_per_m"):
        Uncertainty(range_per_m=0.01, angle_per_m=0.0)
