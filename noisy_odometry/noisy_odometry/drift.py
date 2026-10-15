"""Wheel odometry that drifts like a real robot's, made from the simulation's exact one, without ROS.

Gazebo's wheel odometry barely drifts; a real robot's wanders off. This module adds a random,
zero-mean error to every step of motion and integrates it, so the odometry drifts slowly and in no
particular direction.

The error is scaled to the motion, not to the message: each step's error variance is proportional to
the step itself, so the errors add up as a random walk. After travelling L metres the position has
spread by about sqrt(variance per metre * L), however often odometry is published.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class DriftModel:
    """How much error the odometry gathers per unit of motion, as variances.

    Attributes:
        distance_var_per_m: Variance of the distance travelled, per metre (m^2/m).
        heading_var_per_rad: Variance of the heading, per radian turned (rad^2/rad).
        heading_var_per_m: Variance of the heading, per metre travelled (rad^2/m).
    """

    distance_var_per_m: float
    heading_var_per_rad: float
    heading_var_per_m: float

    def __post_init__(self) -> None:
        for name in ("distance_var_per_m", "heading_var_per_rad", "heading_var_per_m"):
            value = getattr(self, name)
            if value < 0:
                raise ValueError(f"{name} cannot be negative, got {value}")


@dataclass(frozen=True)
class Pose2D:
    """A pose on the floor: position in metres, heading in radians counter-clockwise from x."""

    x: float
    y: float
    yaw: float


def wrap(angle: float) -> float:
    """The same angle in [-pi, pi)."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


# ---------------------------------------------------------------------------
# One step
# ---------------------------------------------------------------------------


def increment(previous: Pose2D, current: Pose2D) -> tuple[float, float]:
    """Forward distance and heading change from one pose to the next.

    The distance is measured along the robot's heading at the first pose. A differential drive
    cannot move sideways, so any sideways part, which the simulation only produces by rounding, is
    left out.
    """
    dx, dy = current.x - previous.x, current.y - previous.y
    forward = math.cos(previous.yaw) * dx + math.sin(previous.yaw) * dy
    return forward, wrap(current.yaw - previous.yaw)


def step_variances(distance: float, turn: float, model: DriftModel) -> tuple[float, float]:
    """Variances of one step's distance and heading errors, proportional to the step."""
    return (
        model.distance_var_per_m * abs(distance),
        model.heading_var_per_rad * abs(turn) + model.heading_var_per_m * abs(distance),
    )


def advance(pose: Pose2D, distance: float, turn: float) -> Pose2D:
    """The pose after one step: forward along the mid-step heading, while turning."""
    heading = pose.yaw + turn / 2
    return Pose2D(
        x=pose.x + distance * math.cos(heading),
        y=pose.y + distance * math.sin(heading),
        yaw=wrap(pose.yaw + turn),
    )


# ---------------------------------------------------------------------------
# The drifting odometry
# ---------------------------------------------------------------------------


class DriftingOdometry:
    """Follows the exact odometry step by step, with each step's error added and kept.

    Attributes:
        pose: The drifting pose, starting at the origin like the exact odometry.
        distance_var: Variance of the distance error gathered so far (m^2).
        heading_var: Variance of the heading error gathered so far (rad^2).
    """

    def __init__(self, model: DriftModel, seed: int | None = None) -> None:
        self.model = model
        self.rng = np.random.default_rng(seed)
        self.pose = Pose2D(0.0, 0.0, 0.0)
        self.distance_var = 0.0
        self.heading_var = 0.0
        self._last_exact: Pose2D | None = None

    def update(self, exact: Pose2D) -> tuple[float, float, float, float]:
        """Take the next exact pose and move the drifting one by the same step, with its error.

        Returns:
            The step as the drifting odometry saw it: distance (m) and turn (rad), and the
            variances of their errors.
        """
        if self._last_exact is None:
            self._last_exact = exact
            return 0.0, 0.0, 0.0, 0.0
        distance, turn = increment(self._last_exact, exact)
        self._last_exact = exact
        distance_var, turn_var = step_variances(distance, turn, self.model)
        noisy_distance = distance + self.rng.normal(0.0, math.sqrt(distance_var))
        noisy_turn = turn + self.rng.normal(0.0, math.sqrt(turn_var))
        self.pose = advance(self.pose, noisy_distance, noisy_turn)
        self.distance_var += distance_var
        self.heading_var += turn_var
        return noisy_distance, noisy_turn, distance_var, turn_var
