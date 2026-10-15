"""ROS 2 node that turns the simulation's exact wheel odometry into odometry that drifts.

Subscribes:
    odom_exact (nav_msgs/Odometry): the simulation's wheel odometry, which barely drifts.

Publishes:
    odom (nav_msgs/Odometry): the same motion with a random error added at every step and kept, in
        the odom frame, with the speeds a filter fuses and their variances. It publishes no TF: the
        local filter owns odom -> base_footprint.

Parameters:
    distance_var_per_m, heading_var_per_rad, heading_var_per_m (double, required): how much error
        gathers per unit of motion; see drift.DriftModel.
    seed (int, default -1): a fixed seed repeats the same drift; -1 draws a new one at every start.
    odom_frame (string, default odom), base_frame (string, default base_footprint).
"""

from __future__ import annotations

import math
import signal
from typing import Any

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter

from noisy_odometry.drift import DriftingOdometry, DriftModel, Pose2D

# A step with no motion has no error, but a filter cannot take a zero variance: this is the floor
# for a stopped robot's speeds, (m/s)^2 and (rad/s)^2.
MIN_SPEED_VARIANCE = 1e-6
# The robot cannot move sideways, and the filter is told so with a tight variance, (m/s)^2.
SIDEWAYS_VARIANCE = 1e-6
# Height, roll and pitch are fixed by the floor.
FLOOR_VARIANCE = 1e-6


class NoisyOdometryNode(Node):
    """Republishes the exact odometry with drift added."""

    def __init__(self) -> None:
        super().__init__("noisy_odometry")

        # Declared with a type and no default: get_parameter stops the node here if one is missing.
        for name in ("distance_var_per_m", "heading_var_per_rad", "heading_var_per_m"):
            self.declare_parameter(name, Parameter.Type.DOUBLE)
        self.declare_parameter("seed", -1)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")

        model = DriftModel(
            distance_var_per_m=self._param("distance_var_per_m"),
            heading_var_per_rad=self._param("heading_var_per_rad"),
            heading_var_per_m=self._param("heading_var_per_m"),
        )
        seed: int = self._param("seed")
        self.odometry = DriftingOdometry(model, seed=None if seed < 0 else seed)
        self.odom_frame: str = self._param("odom_frame")
        self.base_frame: str = self._param("base_frame")
        self.last_stamp: float | None = None

        self.create_subscription(Odometry, "odom_exact", self.on_exact, 10)
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)

    def _param(self, name: str) -> Any:
        return self.get_parameter(name).value

    def on_exact(self, msg: Odometry) -> None:
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        distance, turn, distance_var, turn_var = self.odometry.update(Pose2D(p.x, p.y, yaw))

        stamp = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        dt = stamp - self.last_stamp if self.last_stamp is not None else 0.0
        self.last_stamp = stamp
        if dt <= 0:
            return  # the first message, or a repeated stamp: no speed to report

        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.odom_frame
        out.child_frame_id = self.base_frame
        pose = self.odometry.pose
        out.pose.pose.position.x, out.pose.pose.position.y = pose.x, pose.y
        out.pose.pose.orientation.z = math.sin(pose.yaw / 2)
        out.pose.pose.orientation.w = math.cos(pose.yaw / 2)
        # Row-major 6x6 over x, y, z, roll, pitch, yaw: the error gathered so far.
        pose_cov = [0.0] * 36
        pose_cov[0] = pose_cov[7] = self.odometry.distance_var
        pose_cov[14] = pose_cov[21] = pose_cov[28] = FLOOR_VARIANCE
        pose_cov[35] = self.odometry.heading_var
        out.pose.covariance = pose_cov

        # Speeds over this step, in the robot's frame, with this step's error as their variance.
        out.twist.twist.linear.x = distance / dt
        out.twist.twist.angular.z = turn / dt
        twist_cov = [0.0] * 36
        twist_cov[0] = max(distance_var / dt**2, MIN_SPEED_VARIANCE)
        twist_cov[7] = SIDEWAYS_VARIANCE
        twist_cov[14] = twist_cov[21] = twist_cov[28] = FLOOR_VARIANCE
        twist_cov[35] = max(turn_var / dt**2, MIN_SPEED_VARIANCE)
        out.twist.covariance = twist_cov
        self.odom_pub.publish(out)


def main(args: list[str] | None = None) -> None:
    try:
        rclpy.init(args=args)
        node = NoisyOdometryNode()
        try:
            rclpy.spin(node)
        except RuntimeError:
            # rclpy can fail to take a message that arrives while it shuts down; that is not an
            # error. While it is still running, it is.
            if rclpy.ok():
                raise
        finally:
            # Ctrl+C under ros2 launch reaches a node twice; once shutting down, the repeat is
            # ignored so it cannot cut the cleanup (same as aruco_detector).
            signal.signal(signal.SIGINT, signal.SIG_IGN)
            node.destroy_node()
            rclpy.try_shutdown()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
