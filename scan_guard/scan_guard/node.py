"""ROS 2 node that keeps the robot from driving into what its 2D lidar sees.

Subscribes:
    scan (sensor_msgs/LaserScan): the lidar.
    cmd_vel_in (geometry_msgs/Twist): the command to guard, from teleop or a planner.

Publishes:
    cmd_vel_out (geometry_msgs/Twist): what the zones allow of the command, at a fixed rate.
    scan_guard/zones (visualization_msgs/MarkerArray): the stop and slow zones ahead and behind,
        solid while an obstacle is in them, for RViz.

Parameters:
    front, rear, half_width (double, required): the robot's outline in base_frame, in metres. A
        default outline, meant for some other robot, could quietly shrink the zones.
    stop_distance, slow_distance (double, required): zone lengths past the robot's edge, in metres.
    slow_speed (double, required): speed cap in the slow zone, in m/s.
    side_margin (double, default 0.05): extra width on each side of the robot, in metres.
    min_points (int, default 2): returns an obstacle needs before it counts.
    resume_margin (double, default 0.05): free distance past the stop zone, in metres, needed to
        release a stop.
    resume_delay (double, default 1.0): seconds that distance must stay free before the robot
        moves on.
    scan_timeout (double, default 0.5): seconds without a scan before the robot is held still.
    rate (double, default 20.0): how often the command is sent, in Hz.
    base_frame (string, default base_footprint): the frame the outline is given in.

The node sends zero velocity until it has a recent scan and the lidar's pose from TF. Without them
it cannot tell whether the path is clear.
"""

from __future__ import annotations

import math
import signal
import time
from typing import Any

import rclpy
from geometry_msgs.msg import Twist
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from scan_guard.guard import (
    CommandGate,
    Footprint,
    SensorPose,
    Zones,
    clearance,
    scan_points,
    yaw_from_quaternion,
)

# How the zones look in RViz: thin slabs on the floor, faint while clear and solid while occupied
SLOW_RGB = (0.95, 0.75, 0.1)
STOP_RGB = (0.9, 0.2, 0.2)
IDLE_ALPHA = 0.15
OCCUPIED_ALPHA = 0.6
SLAB_THICKNESS_M = 0.01


class ScanGuardNode(Node):
    """Turns each scan into the free distance ahead and behind, and gates the command with it."""

    def __init__(self) -> None:
        super().__init__("scan_guard")

        # The first six have no default. The node stops at start-up if any of them is missing.
        for name in ("front", "rear", "half_width", "stop_distance", "slow_distance", "slow_speed"):
            self.declare_parameter(name, Parameter.Type.DOUBLE)
        self.declare_parameter("side_margin", 0.05)
        self.declare_parameter("min_points", 2)
        self.declare_parameter("resume_margin", 0.05)
        self.declare_parameter("resume_delay", 1.0)
        self.declare_parameter("scan_timeout", 0.5)
        self.declare_parameter("rate", 20.0)
        self.declare_parameter("base_frame", "base_footprint")

        self.footprint = Footprint(
            front=self._param("front"), rear=self._param("rear"), half_width=self._param("half_width")
        )
        self.zones = Zones(
            stop_distance=self._param("stop_distance"),
            slow_distance=self._param("slow_distance"),
            slow_speed=self._param("slow_speed"),
            side_margin=self._param("side_margin"),
            min_points=self._param("min_points"),
            resume_margin=self._param("resume_margin"),
            resume_delay=self._param("resume_delay"),
        )
        self.scan_timeout: float = self._param("scan_timeout")
        self.base_frame: str = self._param("base_frame")

        self.gate = CommandGate(self.zones)
        self.sensor: SensorPose | None = None  # the lidar's pose in base_frame, looked up once
        self.gaps = (0.0, 0.0)  # free metres ahead and behind, zero until a scan arrives
        self.last_scan = -math.inf  # time.monotonic() of the last scan used
        self.holding = False  # whether the last cycle held the robot for lack of scans

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Best-effort sensor QoS, as for the detector's camera. Relative names, for the launch file
        # to remap.
        self.create_subscription(LaserScan, "scan", self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Twist, "cmd_vel_in", self.on_command, 10)
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel_out", 10)
        self.zones_pub = self.create_publisher(MarkerArray, "scan_guard/zones", 10)
        # On the steady clock, which only moves forward. A timer on the wall clock goes quiet each
        # time that clock is set back, and the wheels keep their last command until it catches up.
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1.0 / self._param("rate"), self.on_timer, clock=steady)

        # Logs the remapped names: a teleop pointed at the wrong topic shows up here.
        self.get_logger().info(
            f"guarding {self.resolve_topic_name('cmd_vel_in')} -> "
            f"{self.resolve_topic_name('cmd_vel_out')} with {self.resolve_topic_name('scan')}"
        )

    def _param(self, name: str) -> Any:
        return self.get_parameter(name).value

    def on_scan(self, msg: LaserScan) -> None:
        if self.sensor is None:
            self.sensor = self.lookup_sensor(msg.header.frame_id)
            if self.sensor is None:
                return
        points = scan_points(
            msg.ranges,
            msg.angle_min,
            msg.angle_increment,
            msg.range_min,
            msg.range_max,
            self.sensor,
        )
        self.gaps = (
            clearance(points, self.footprint, self.zones, 1),
            clearance(points, self.footprint, self.zones, -1),
        )
        self.last_scan = time.monotonic()
        self.zones_pub.publish(self.zones_msg())

    def lookup_sensor(self, frame: str) -> SensorPose | None:
        """Where the lidar sits on the robot. It is bolted on, and one lookup serves the run."""
        try:
            t = self.tf_buffer.lookup_transform(self.base_frame, frame, Time())
        except TransformException as e:
            self.get_logger().warn(f"lidar frame not in TF yet: {e}", throttle_duration_sec=2.0)
            return None
        q = t.transform.rotation
        pose = SensorPose(
            x=t.transform.translation.x,
            y=t.transform.translation.y,
            yaw=yaw_from_quaternion(q.x, q.y, q.z, q.w),
        )
        self.get_logger().info(
            f"{frame} at x={pose.x:.3f} y={pose.y:.3f} yaw={pose.yaw:.3f} in {self.base_frame}"
        )
        return pose

    def on_command(self, msg: Twist) -> None:
        self.gate.command(msg.linear.x, msg.angular.z)

    def on_timer(self) -> None:
        # Real time on purpose: a paused or stalled simulation stops the scans, and that is exactly
        # when the robot must not move.
        now = time.monotonic()
        holding = now - self.last_scan > self.scan_timeout
        if holding:
            self.gate.halt()
            linear, angular = 0.0, 0.0
        else:
            was_stopped = self.gate.stopped
            linear, angular = self.gate.output(*self.gaps, now)
            if self.gate.stopped and not was_stopped:
                gap = self.gaps[0] if self.gate.stopped > 0 else self.gaps[1]
                way = "ahead" if self.gate.stopped > 0 else "behind"
                self.get_logger().info(f"stopped: obstacle {gap:.2f} m {way}")
            elif was_stopped and not self.gate.stopped and linear != 0.0:
                self.get_logger().info("way clear: moving on")
        if holding != self.holding:
            if not holding:
                self.get_logger().info("scans arriving: waiting for a command")
            elif math.isinf(self.last_scan):
                self.get_logger().info("waiting for the first scan: holding the robot still")
            else:
                self.get_logger().warn("no recent scan: holding the robot still")
            self.holding = holding

        out = Twist()
        out.linear.x = linear
        out.angular.z = angular
        self.cmd_pub.publish(out)

    def zones_msg(self) -> MarkerArray:
        """Slow and stop zones ahead and behind, as slabs on the floor of base_frame.

        The stamp stays zero, which RViz reads as "the latest transform". The slabs then follow the
        robot without waiting for TF at the scan's time.
        """
        width = 2 * (self.footprint.half_width + self.zones.side_margin)
        sides = (
            ("ahead", 1, self.footprint.front, self.gaps[0]),
            ("behind", -1, self.footprint.rear, self.gaps[1]),
        )
        markers = []
        for ns, direction, edge, gap in sides:
            zones = ((self.zones.slow_distance, SLOW_RGB), (self.zones.stop_distance, STOP_RGB))
            for i, (length, rgb) in enumerate(zones):
                slab = Marker(ns=ns, id=i, type=Marker.CUBE, action=Marker.ADD)
                slab.header.frame_id = self.base_frame
                slab.pose.position.x = direction * (edge + length / 2)
                slab.pose.position.z = (i + 0.5) * SLAB_THICKNESS_M  # stop zone drawn on top
                slab.pose.orientation.w = 1.0
                slab.scale.x, slab.scale.y, slab.scale.z = length, width, SLAB_THICKNESS_M
                alpha = OCCUPIED_ALPHA if gap <= length else IDLE_ALPHA
                slab.color = ColorRGBA(r=rgb[0], g=rgb[1], b=rgb[2], a=alpha)
                markers.append(slab)
        return MarkerArray(markers=markers)


def main(args: list[str] | None = None) -> None:
    # Starts and shuts down like aruco_detector's main(), whose comments explain both guards.
    try:
        rclpy.init(args=args)
        node = ScanGuardNode()
        try:
            rclpy.spin(node)
        except RuntimeError:
            if rclpy.ok():
                raise
        finally:
            signal.signal(signal.SIGINT, signal.SIG_IGN)
            node.destroy_node()
            rclpy.try_shutdown()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
