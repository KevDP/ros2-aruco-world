"""Keep a robot from driving into what its 2D lidar sees, without ROS.

The node in node.py feeds this module each scan and each velocity command; everything that decides
lives here, so it can be tested on synthetic scans.

Two zones lie past the robot's edge in the direction it drives, as wide as the robot plus a margin:

- **stop zone:** an obstacle in it stops any motion that way, and the command is dropped, so the
  robot does not set off again by itself when the obstacle leaves;
- **slow zone:** an obstacle in it caps the speed that way.

Turning in place is never limited. Frames follow REP 103: x forward, y left, angles
counter-clockwise from x.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Footprint:
    """The robot's outline, a rectangle in its base frame, in metres.

    Attributes:
        front: From the base frame's origin to the front edge.
        rear: From the origin to the rear edge.
        half_width: From the centre line to the widest point on either side.
    """

    front: float
    rear: float
    half_width: float

    def __post_init__(self) -> None:
        for name in ("front", "rear", "half_width"):
            value = getattr(self, name)
            if value <= 0:
                raise ValueError(f"{name} must be positive, got {value}")


@dataclass(frozen=True)
class Zones:
    """Where obstacles slow or stop the robot.

    Attributes:
        stop_distance: An obstacle this close to the leading edge, or closer, stops the robot (m).
        slow_distance: An obstacle this close, or closer, caps the speed at ``slow_speed`` (m).
        slow_speed: Speed cap inside the slow zone (m/s).
        side_margin: Added to each side of the robot, so the zones are wider than it (m).
        min_points: Returns an obstacle needs before it counts; 2 ignores a lone stray return.
    """

    stop_distance: float
    slow_distance: float
    slow_speed: float
    side_margin: float = 0.05
    min_points: int = 2

    def __post_init__(self) -> None:
        if not 0 < self.stop_distance < self.slow_distance:
            raise ValueError(
                "need 0 < stop_distance < slow_distance, "
                f"got {self.stop_distance} and {self.slow_distance}"
            )
        if self.slow_speed <= 0:
            raise ValueError(f"slow_speed must be positive, got {self.slow_speed}")
        if self.side_margin < 0:
            raise ValueError(f"side_margin cannot be negative, got {self.side_margin}")
        if self.min_points < 1:
            raise ValueError(f"min_points must be at least 1, got {self.min_points}")


@dataclass(frozen=True)
class SensorPose:
    """Where the lidar sits in the robot's base frame: position in metres, heading in radians."""

    x: float
    y: float
    yaw: float


# ---------------------------------------------------------------------------
# Geometry
# ---------------------------------------------------------------------------


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Heading of a rotation about the vertical, from its quaternion.

    A lidar mounted level has no roll or pitch, so its heading is all that places its scan.
    """
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def scan_points(
    ranges: np.ndarray | list[float],
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    sensor: SensorPose,
) -> np.ndarray:
    """The scan's valid returns as points in the robot's base frame.

    A return is valid when it is finite and within ``[range_min, range_max]``: below the minimum
    the sensor cannot measure, and lidars report "nothing hit" as inf or as a value past the
    maximum. Ray ``i`` points at ``angle_min + i * angle_increment`` in the sensor's frame.

    Returns:
        Array of shape (n, 2) with x and y in metres.
    """
    r = np.asarray(ranges, dtype=float)
    angles = angle_min + angle_increment * np.arange(r.size) + sensor.yaw
    valid = np.isfinite(r) & (r >= range_min) & (r <= range_max)
    r, angles = r[valid], angles[valid]
    return np.column_stack((sensor.x + r * np.cos(angles), sensor.y + r * np.sin(angles)))


def clearance(points: np.ndarray, footprint: Footprint, zones: Zones, direction: int) -> float:
    """Free distance from the robot's leading edge to the nearest obstacle in its path.

    The path is a corridor as wide as the robot plus the side margin on each side, past the front
    edge when driving forward and past the rear edge when reversing. Points beside the robot do not
    count: they are not in the way.

    Args:
        points: Obstacle points in the base frame, shape (n, 2).
        footprint: The robot's outline.
        zones: Supplies the side margin and how many returns an obstacle needs.
        direction: 1 forward, -1 backward.

    Returns:
        Metres from the edge to the ``min_points``-th closest return in the path, so a single stray
        return is ignored; inf if the path is clear.
    """
    if direction not in (1, -1):
        raise ValueError(f"direction must be 1 or -1, got {direction}")
    edge = footprint.front if direction == 1 else footprint.rear
    past_edge = direction * points[:, 0] - edge
    in_path = (past_edge > 0) & (np.abs(points[:, 1]) <= footprint.half_width + zones.side_margin)
    gaps = np.sort(past_edge[in_path])
    if gaps.size < zones.min_points:
        return math.inf
    return float(gaps[zones.min_points - 1])


def speed_limit(gap: float, zones: Zones) -> float:
    """Highest speed allowed toward an obstacle ``gap`` metres away: 0, the slow speed, or inf."""
    if gap <= zones.stop_distance:
        return 0.0
    if gap <= zones.slow_distance:
        return zones.slow_speed
    return math.inf


# ---------------------------------------------------------------------------
# Decision
# ---------------------------------------------------------------------------


class CommandGate:
    """Holds the last velocity command and passes on what the zones allow of it.

    A stop drops the held command's linear part, so a robot stopped by an obstacle stays stopped
    after the obstacle leaves, until it is commanded again. The slow zone only caps the speed.
    """

    def __init__(self, zones: Zones) -> None:
        self.zones = zones
        self.linear = 0.0  # m/s, forward positive
        self.angular = 0.0  # rad/s, counter-clockwise positive

    def command(self, linear: float, angular: float) -> None:
        """Hold a new command."""
        self.linear, self.angular = linear, angular

    def halt(self) -> None:
        """Drop the held command entirely, as when the lidar goes quiet."""
        self.linear = self.angular = 0.0

    def output(self, gap_front: float, gap_rear: float) -> tuple[float, float]:
        """The command to send now, given the free distance ahead and behind.

        Returns:
            Linear (m/s) and angular (rad/s) velocity.
        """
        if self.linear == 0.0:
            return 0.0, self.angular
        limit = speed_limit(gap_front if self.linear > 0 else gap_rear, self.zones)
        if limit == 0.0:
            self.linear = 0.0  # stopped: wait for a new command
            return 0.0, self.angular
        return math.copysign(min(abs(self.linear), limit), self.linear), self.angular
