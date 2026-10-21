"""Pieces the experiments share that need ROS and Gazebo: placing the robot, message conversions.

What only reads the world's files lives in world.py, which needs neither.
"""

from __future__ import annotations

import math

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Quaternion
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.pose_pb2 import Pose
from gz.transport13 import Node as GzNode

WORLD_NAME = "aruco_room"  # <world name> in the world build_world.py writes
ROBOT_NAME = "camera_bot"  # the name the launch spawns the robot with

# ---------------------------------------------------------------------------
# Placing the robot
# ---------------------------------------------------------------------------


class RobotPlacer:
    """Moves the robot to a pose through Gazebo's own service, as a person would drag it.

    It talks to Gazebo directly rather than through ROS: no node the robot runs can do this.
    """

    def __init__(self, timeout_ms: int = 2000) -> None:
        self._node = GzNode()
        self._service = f"/world/{WORLD_NAME}/set_pose"
        self._timeout_ms = timeout_ms

    def place(self, x: float, y: float, yaw: float) -> None:
        """Put the robot on the floor at (x, y), facing ``yaw`` radians.

        Raises:
            RuntimeError: if Gazebo does not accept the move.
        """
        request = Pose()
        request.name = ROBOT_NAME
        request.position.x, request.position.y, request.position.z = x, y, 0.0
        request.orientation.z, request.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        ok, reply = self._node.request(self._service, request, Pose, Boolean, self._timeout_ms)
        if not (ok and reply.data):
            raise RuntimeError(f"Gazebo did not place {ROBOT_NAME} at ({x}, {y}, {yaw})")


# ---------------------------------------------------------------------------
# Conversions
# ---------------------------------------------------------------------------


def yaw_of(q: Quaternion) -> float:
    """Heading in radians of a quaternion, for a robot that stays level."""
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(angle: float) -> float:
    """The same angle in (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def seconds(stamp: Time) -> float:
    """A message stamp in seconds."""
    return stamp.sec + stamp.nanosec * 1e-9
