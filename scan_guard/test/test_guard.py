"""The guard's decisions on synthetic scans, where every obstacle's position is known."""

from __future__ import annotations

import math

import numpy as np
import pytest
from scan_guard.guard import (
    CommandGate,
    Footprint,
    SensorPose,
    Zones,
    clearance,
    scan_points,
    speed_limit,
    yaw_from_quaternion,
)

# A full turn in 500 rays, like the simulated lidar (5,000 samples per second at 10 Hz).
SAMPLES = 500
ANGLE_MIN = -math.pi
ANGLE_INCREMENT = 2 * math.pi / SAMPLES
RANGE_MIN, RANGE_MAX = 0.05, 12.0

ROBOT = Footprint(front=0.18, rear=0.18, half_width=0.16)
ZONES = Zones(stop_distance=0.3, slow_distance=0.8, slow_speed=0.15, side_margin=0.05, min_points=2)
CENTRED = SensorPose(0.0, 0.0, 0.0)


def _scan(obstacles: list[tuple[float, float]], rays: int = 3) -> np.ndarray:
    """A scan of open space with small obstacles at the given (x, y) in the sensor's frame.

    Each obstacle answers ``rays`` neighbouring rays at its distance, like a thin post.
    """
    ranges = np.full(SAMPLES, math.inf)
    for x, y in obstacles:
        centre = round((math.atan2(y, x) - ANGLE_MIN) / ANGLE_INCREMENT)
        for k in range(centre - rays // 2, centre - rays // 2 + rays):
            ranges[k % SAMPLES] = math.hypot(x, y)
    return ranges


def _points(ranges: np.ndarray, sensor: SensorPose = CENTRED) -> np.ndarray:
    return scan_points(ranges, ANGLE_MIN, ANGLE_INCREMENT, RANGE_MIN, RANGE_MAX, sensor)


def _gaps(ranges: np.ndarray) -> tuple[float, float]:
    points = _points(ranges)
    return clearance(points, ROBOT, ZONES, 1), clearance(points, ROBOT, ZONES, -1)


# ---------------------------------------------------------------------------
# Points and clearance
# ---------------------------------------------------------------------------


def test_points_land_where_the_rays_hit() -> None:
    points = _points(_scan([(1.0, 0.0)], rays=1))
    assert points.shape == (1, 2)
    assert points[0, 0] == pytest.approx(1.0)
    assert points[0, 1] == pytest.approx(0.0, abs=1e-9)


def test_points_follow_the_sensor_pose() -> None:
    # A lidar 0.1 m ahead of the centre and turned to face left: its forward ray points along +y.
    sensor = SensorPose(0.1, 0.0, math.pi / 2)
    points = _points(_scan([(1.0, 0.0)], rays=1), sensor)
    assert points[0, 0] == pytest.approx(0.1)
    assert points[0, 1] == pytest.approx(1.0)


def test_invalid_returns_are_dropped() -> None:
    ranges = np.full(SAMPLES, math.inf)
    ranges[:4] = [math.nan, -math.inf, RANGE_MIN / 2, RANGE_MAX + 1]
    assert _points(ranges).shape == (0, 2)


def test_clearance_is_measured_from_the_leading_edge() -> None:
    # The neighbouring rays meet the obstacle a hair closer than the centre one: within 1 mm.
    front, rear = _gaps(_scan([(1.0, 0.0), (-2.0, 0.0)]))
    assert front == pytest.approx(1.0 - ROBOT.front, abs=1e-3)
    assert rear == pytest.approx(2.0 - ROBOT.rear, abs=1e-3)


def test_open_space_is_clear_both_ways() -> None:
    assert _gaps(_scan([])) == (math.inf, math.inf)


def test_obstacles_beside_the_path_do_not_count() -> None:
    beside = ROBOT.half_width + ZONES.side_margin + 0.05
    assert _gaps(_scan([(0.5, beside), (0.5, -beside)]))[0] == math.inf


def test_obstacles_inside_the_margin_count() -> None:
    inside = ROBOT.half_width + ZONES.side_margin - 0.02
    assert _gaps(_scan([(0.5, inside)]))[0] < math.inf


def test_a_lone_stray_return_is_ignored() -> None:
    assert _gaps(_scan([(0.4, 0.0)], rays=1))[0] == math.inf
    assert _gaps(_scan([(0.4, 0.0)], rays=2))[0] == pytest.approx(0.4 - ROBOT.front)


def test_clearance_rejects_a_bad_direction() -> None:
    with pytest.raises(ValueError, match="direction"):
        clearance(_points(_scan([])), ROBOT, ZONES, 0)


# ---------------------------------------------------------------------------
# Speed limits and the gate
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    ("gap", "limit"),
    [(0.0, 0.0), (0.3, 0.0), (0.31, 0.15), (0.8, 0.15), (0.81, math.inf), (math.inf, math.inf)],
)
def test_speed_limit_by_zone(gap: float, limit: float) -> None:
    assert speed_limit(gap, ZONES) == limit


def test_a_clear_path_passes_the_command_unchanged() -> None:
    gate = CommandGate(ZONES)
    gate.command(0.5, 0.3)
    assert gate.output(math.inf, math.inf, 0.0) == (0.5, 0.3)


def test_the_slow_zone_caps_the_speed_only_when_faster() -> None:
    gate = CommandGate(ZONES)
    gate.command(0.5, 0.0)
    assert gate.output(0.5, math.inf, 0.0) == (0.15, 0.0)
    gate.command(0.1, 0.0)
    assert gate.output(0.5, math.inf, 0.0) == (0.1, 0.0)


def test_the_stop_zone_stops_driving_but_not_turning() -> None:
    gate = CommandGate(ZONES)
    gate.command(0.5, 0.4)
    assert gate.output(0.2, math.inf, 0.0) == (0.0, 0.4)


def test_a_stopped_robot_moves_on_once_the_way_stays_clear() -> None:
    gate = CommandGate(ZONES)
    gate.command(0.5, 0.0)
    assert gate.output(0.2, math.inf, 0.0) == (0.0, 0.0)
    assert gate.output(math.inf, math.inf, 0.1) == (0.0, 0.0)  # clear from here on
    assert gate.output(math.inf, math.inf, 0.1 + ZONES.resume_delay - 0.01) == (0.0, 0.0)
    assert gate.output(math.inf, math.inf, 0.1 + ZONES.resume_delay) == (0.5, 0.0)


def test_noise_at_the_edge_of_the_stop_zone_does_not_release_a_stop() -> None:
    gate = CommandGate(ZONES)
    gate.command(0.5, 0.0)
    gate.output(0.28, math.inf, 0.0)
    edge = ZONES.stop_distance + ZONES.resume_margin
    for t in np.arange(0.05, 5.0, 0.05):
        assert gate.output(edge - 0.001, math.inf, t) == (0.0, 0.0)


def test_the_wait_starts_over_when_the_way_closes_again() -> None:
    gate = CommandGate(ZONES)
    gate.command(0.5, 0.0)
    gate.output(0.2, math.inf, 0.0)
    gate.output(math.inf, math.inf, 0.1)
    gate.output(0.2, math.inf, 0.5)  # something steps back in
    gate.output(math.inf, math.inf, 0.6)
    assert gate.output(math.inf, math.inf, 0.1 + ZONES.resume_delay) == (0.0, 0.0)
    assert gate.output(math.inf, math.inf, 0.6 + ZONES.resume_delay) == (0.5, 0.0)


def test_a_stop_does_not_hold_back_driving_away() -> None:
    gate = CommandGate(ZONES)
    gate.command(0.5, 0.0)
    gate.output(0.2, math.inf, 0.0)
    gate.command(-0.3, 0.0)
    assert gate.output(0.2, math.inf, 0.05) == (-0.3, 0.0)


def test_only_the_direction_of_travel_is_checked() -> None:
    gate = CommandGate(ZONES)
    gate.command(0.5, 0.0)
    assert gate.output(math.inf, 0.1, 0.0) == (0.5, 0.0)  # blocked behind, driving forward
    gate.command(-0.5, 0.0)
    assert gate.output(math.inf, 0.1, 0.0) == (0.0, 0.0)
    gate.command(-0.5, 0.0)
    assert gate.output(math.inf, 0.5, 0.0) == (0.0, 0.0)  # clear, but only just
    assert gate.output(math.inf, 0.5, ZONES.resume_delay) == (-0.15, 0.0)


def test_halt_drops_the_whole_command() -> None:
    gate = CommandGate(ZONES)
    gate.command(0.5, 0.4)
    gate.output(0.2, math.inf, 0.0)
    gate.halt()
    assert gate.output(math.inf, math.inf, 5.0) == (0.0, 0.0)


def test_from_scan_to_command() -> None:
    gate = CommandGate(ZONES)
    gate.command(0.5, 0.0)
    assert gate.output(*_gaps(_scan([(ROBOT.front + 0.5, 0.0)])), 0.0) == (0.15, 0.0)
    assert gate.output(*_gaps(_scan([(ROBOT.front + 0.2, 0.0)])), 0.05) == (0.0, 0.0)


# ---------------------------------------------------------------------------
# Configuration and helpers
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "kwargs",
    [
        {"stop_distance": 0.8, "slow_distance": 0.3, "slow_speed": 0.15},
        {"stop_distance": 0.0, "slow_distance": 0.3, "slow_speed": 0.15},
        {"stop_distance": 0.3, "slow_distance": 0.8, "slow_speed": 0.0},
        {"stop_distance": 0.3, "slow_distance": 0.8, "slow_speed": 0.15, "side_margin": -0.1},
        {"stop_distance": 0.3, "slow_distance": 0.8, "slow_speed": 0.15, "min_points": 0},
        {"stop_distance": 0.3, "slow_distance": 0.8, "slow_speed": 0.15, "resume_margin": -0.1},
        {"stop_distance": 0.3, "slow_distance": 0.8, "slow_speed": 0.15, "resume_delay": -1.0},
    ],
)
def test_bad_zones_are_rejected(kwargs: dict[str, float]) -> None:
    with pytest.raises(ValueError):
        Zones(**kwargs)


def test_bad_footprint_is_rejected() -> None:
    with pytest.raises(ValueError, match="half_width"):
        Footprint(front=0.18, rear=0.18, half_width=0.0)


@pytest.mark.parametrize("yaw", [0.0, math.pi / 2, -math.pi / 3, math.pi - 0.01])
def test_yaw_from_quaternion(yaw: float) -> None:
    assert yaw_from_quaternion(0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)) == pytest.approx(yaw)
