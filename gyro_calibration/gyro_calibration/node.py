"""ROS 2 node that measures the gyro's bias at start-up and republishes the IMU without it.

Subscribes:
    imu (sensor_msgs/Imu): the IMU as the driver publishes it.

Publishes:
    imu_calibrated (sensor_msgs/Imu): the same messages with the bias taken out of the angular
        velocity. Nothing is published until the bias is measured, and the filters run on the
        wheels alone until then.

Parameters:
    duration (double, required): seconds at rest to average, by the messages' stamps.
    max_rate (double, required): rad/s on any axis past which the robot counts as moving and the
        average starts over (see bias.Calibration).

The robot has to stand still for a moment after start-up, as when it is spawned or switched on.
"""

from __future__ import annotations

import signal
from typing import Any

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from gyro_calibration.bias import BiasEstimator, Calibration, corrected


class GyroCalibrationNode(Node):
    """Holds the IMU back until the bias is measured, then passes it on corrected."""

    def __init__(self) -> None:
        super().__init__("gyro_calibration")
        # Both are required: get_parameter raises at start-up if one is unset.
        self.declare_parameter("duration", Parameter.Type.DOUBLE)
        self.declare_parameter("max_rate", Parameter.Type.DOUBLE)
        calibration = Calibration(
            duration=self._param("duration"), max_rate=self._param("max_rate")
        )
        self.estimator = BiasEstimator(calibration)
        self.create_subscription(Imu, "imu", self.on_imu, qos_profile_sensor_data)
        self.imu_pub = self.create_publisher(Imu, "imu_calibrated", qos_profile_sensor_data)

    def _param(self, name: str) -> Any:
        return self.get_parameter(name).value

    def on_imu(self, msg: Imu) -> None:
        w = msg.angular_velocity
        rates = (w.x, w.y, w.z)
        if self.estimator.bias is None:
            stamp = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
            if not self.estimator.add(stamp, rates):
                return
            bx, by, bz = self.estimator.bias
            self.get_logger().info(
                f"gyro bias {bx:+.5f} {by:+.5f} {bz:+.5f} rad/s about x, y, z, after "
                f"{self.estimator.restarts} restarts"
            )
        w.x, w.y, w.z = corrected(rates, self.estimator.bias)
        self.imu_pub.publish(msg)


def main(args: list[str] | None = None) -> None:
    # Starts and shuts down like aruco_detector's main(), whose comments explain both guards.
    try:
        rclpy.init(args=args)
        node = GyroCalibrationNode()
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
