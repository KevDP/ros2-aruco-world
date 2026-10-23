"""Pieces the experiments share that need ROS and Gazebo: placing the robot, message conversions.

What only reads the world's files lives in world.py, which needs neither.
"""

from __future__ import annotations

import math

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Quaternion
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.pose_pb2 import Pose
from gz.transport13 import Node as GzNode
from rclpy.node import Node
from rclpy.parameter import parameter_value_to_python
from rclpy.parameter_client import AsyncParameterClient

WORLD_NAME = "aruco_room"  # <world name> in the world build_world.py writes
ROBOT_NAME = "camera_bot"  # the name the launch spawns the robot with
GUARD_PARAMETERS = (
    "front", "rear", "half_width", "stop_distance", "slow_distance", "slow_speed",
    "side_margin", "min_points", "resume_margin", "resume_delay", "scan_timeout", "rate",
)  # fmt: skip

# ---------------------------------------------------------------------------
# Placing the robot
# ---------------------------------------------------------------------------


class RobotPlacer:
    """Moves the robot to a pose through Gazebo's own service, as a person would drag it.

    It talks to Gazebo directly. No node the robot runs can do this.
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


class Obstacle:
    """A box Gazebo puts on the floor and takes away again, through its own services.

    It is static: the robot can run into it but not push it.
    """

    NAME = "obstacle"

    def __init__(self, size: tuple[float, float, float], timeout_ms: int = 2000) -> None:
        self._node = GzNode()
        self._size = size
        self._timeout_ms = timeout_ms
        sx, sy, sz = size
        box = f"<geometry><box><size>{sx} {sy} {sz}</size></box></geometry>"
        self._sdf = (
            f'<sdf version="1.9"><model name="{self.NAME}"><static>true</static><link name="link">'
            f'<collision name="collision">{box}</collision><visual name="visual">{box}</visual>'
            "</link></model></sdf>"
        )

    def put(self, x: float, y: float) -> None:
        """Stand the box on the floor, centred on (x, y) and square to the room.

        Raises:
            RuntimeError: if Gazebo does not create it.
        """
        request = EntityFactory()
        request.sdf = self._sdf
        request.pose.position.x, request.pose.position.y = x, y
        request.pose.position.z = self._size[2] / 2
        service = f"/world/{WORLD_NAME}/create"
        ok, reply = self._node.request(service, request, EntityFactory, Boolean, self._timeout_ms)
        if not (ok and reply.data):
            raise RuntimeError(f"Gazebo did not create the obstacle at ({x}, {y})")

    def take_away(self) -> None:
        """Remove the box from the world.

        Raises:
            RuntimeError: if Gazebo does not remove it.
        """
        request = Entity()
        request.name, request.type = self.NAME, Entity.MODEL
        service = f"/world/{WORLD_NAME}/remove"
        ok, reply = self._node.request(service, request, Entity, Boolean, self._timeout_ms)
        if not (ok and reply.data):
            raise RuntimeError("Gazebo did not remove the obstacle")


# ---------------------------------------------------------------------------
# The guard
# ---------------------------------------------------------------------------


def guard_settings(node: Node, timeout_s: float = 10.0) -> dict[str, float]:
    """The parameters the running scan_guard uses, as it reports them.

    Raises:
        RuntimeError: if no guard answers, as when the launch ran with guard:=false.
    """
    client = AsyncParameterClient(node, "scan_guard")
    if not client.wait_for_services(timeout_sec=timeout_s):
        raise RuntimeError("no scan_guard parameters: run with guard:=true")
    future = client.get_parameters(list(GUARD_PARAMETERS))
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_s)
    if future.result() is None:
        raise RuntimeError("scan_guard did not report its parameters")
    values = future.result().values
    return {
        name: parameter_value_to_python(v) for name, v in zip(GUARD_PARAMETERS, values, strict=True)
    }


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
