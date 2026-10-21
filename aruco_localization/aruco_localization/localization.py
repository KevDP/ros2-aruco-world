"""Where the robot is, according to one marker it sees, without ROS.

A marker whose pose in the map is known, seen by the camera, places the camera and so the robot:

    map_from_base = map_from_marker @ marker_from_camera @ camera_from_base

Each ``a_from_b`` is a 4x4 rigid transform that takes points from frame b to frame a, which is also
the pose of frame b in frame a. ``marker_from_camera`` is the inverse of the detected marker pose,
and ``camera_from_base`` the inverse of where the camera sits on the robot.

The result is projected onto the floor the robot drives on, keeping x, y and heading and dropping
height, roll and pitch.

A fix's uncertainty grows with distance, and more across the line of sight than along it. The
distance to a marker is measured well. An orientation error of theta moves the camera along an arc
around the marker, about distance * theta sideways.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass

import numpy as np

# ---------------------------------------------------------------------------
# Rigid transforms
# ---------------------------------------------------------------------------


def matrix_from_quaternion(x: float, y: float, z: float, w: float) -> np.ndarray:
    """3x3 rotation of a quaternion, normalised first so a rounded one still gives a rotation."""
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-9:
        raise ValueError("a zero quaternion is not a rotation")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


def pose_matrix(position: Sequence[float], quaternion: Sequence[float]) -> np.ndarray:
    """4x4 transform from a position (x, y, z) and a quaternion (x, y, z, w)."""
    transform = np.eye(4)
    transform[:3, :3] = matrix_from_quaternion(*quaternion)
    transform[:3, 3] = position
    return transform


def invert(transform: np.ndarray) -> np.ndarray:
    """Inverse of a rigid transform, using the rotation's transpose instead of a general inverse."""
    rotation = transform[:3, :3]
    inverse = np.eye(4)
    inverse[:3, :3] = rotation.T
    inverse[:3, 3] = -rotation.T @ transform[:3, 3]
    return inverse


def yaw_quaternion(yaw: float) -> tuple[float, float, float, float]:
    """(x, y, z, w) of a turn of ``yaw`` radians about the vertical."""
    return 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)


# ---------------------------------------------------------------------------
# The marker map
# ---------------------------------------------------------------------------


def build_marker_map(entries: Mapping[int, Sequence[float]]) -> dict[int, np.ndarray]:
    """Each marker's pose in the map as a transform, from 7 numbers per marker.

    Args:
        entries: Marker id to [x, y, z, qx, qy, qz, qw], the centre in metres and the orientation,
            with the marker's z axis out of its printed side.

    Returns:
        Marker id to ``map_from_marker``.
    """
    markers = {}
    for marker_id, values in entries.items():
        if len(values) != 7:
            raise ValueError(
                f"marker {marker_id}: expected x, y, z, qx, qy, qz, qw, got {len(values)} numbers"
            )
        try:
            markers[marker_id] = pose_matrix(values[:3], values[3:])
        except ValueError as e:
            raise ValueError(f"marker {marker_id}: {e}") from e
    return markers


# ---------------------------------------------------------------------------
# One marker's fix
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Fix:
    """The robot's place on the floor according to one marker.

    Attributes:
        x: Position of the robot's base in the map, metres.
        y: Same, metres.
        yaw: Heading of the robot's x axis on the floor, radians counter-clockwise from map x.
        distance: Metres from the camera to the marker.
        sight: Heading of the line of sight, from the camera toward the marker, in radians.
    """

    x: float
    y: float
    yaw: float
    distance: float
    sight: float


def locate(
    map_from_marker: np.ndarray, camera_from_marker: np.ndarray, base_from_camera: np.ndarray
) -> Fix:
    """The robot's pose on the floor from one detected marker.

    Args:
        map_from_marker: The marker's pose in the map.
        camera_from_marker: The detected pose, in the camera's optical frame.
        base_from_camera: Where the camera sits on the robot.

    Returns:
        The fix, projected onto the floor.
    """
    map_from_camera = map_from_marker @ invert(camera_from_marker)
    map_from_base = map_from_camera @ invert(base_from_camera)
    # The heading of the robot's x axis seen from above survives any small tilt in the estimate.
    yaw = math.atan2(map_from_base[1, 0], map_from_base[0, 0])
    to_marker = map_from_marker[:3, 3] - map_from_camera[:3, 3]
    return Fix(
        x=float(map_from_base[0, 3]),
        y=float(map_from_base[1, 3]),
        yaw=yaw,
        distance=float(np.linalg.norm(camera_from_marker[:3, 3])),
        sight=math.atan2(to_marker[1], to_marker[0]),
    )


@dataclass(frozen=True)
class Uncertainty:
    """How a single marker's fix spreads with distance.

    Attributes:
        range_per_m: Standard deviation of the measured distance, per metre of distance.
        angle_per_m: Standard deviation of the marker's measured orientation in radians, per metre
            of distance. Across the line of sight the fix spreads by distance times this angle.
    """

    range_per_m: float
    angle_per_m: float

    def __post_init__(self) -> None:
        for name in ("range_per_m", "angle_per_m"):
            value = getattr(self, name)
            if value <= 0:
                raise ValueError(f"{name} must be positive, got {value}")


def covariance(fix: Fix, uncertainty: Uncertainty) -> np.ndarray:
    """3x3 covariance of a fix over x, y and yaw.

    Along the line of sight the spread is the distance error, and across it the distance times the
    orientation error. The heading takes the orientation error as it is. Both orientation terms come
    from one error and are correlated. Leaving that out counts one marker as a little more
    informative than it is.
    """
    d = fix.distance
    along = uncertainty.range_per_m * d
    angle = uncertainty.angle_per_m * d
    across = d * angle
    c, s = math.cos(fix.sight), math.sin(fix.sight)
    rotation = np.array([[c, -s], [s, c]])
    cov = np.zeros((3, 3))
    cov[:2, :2] = rotation @ np.diag([along**2, across**2]) @ rotation.T
    cov[2, 2] = angle**2
    return cov
