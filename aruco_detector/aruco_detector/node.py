"""ROS 2 node that detects ArUco markers in a camera stream.

Subscribes:
    image (sensor_msgs/Image): the camera stream.
    camera_info (sensor_msgs/CameraInfo): the calibration; poses need it, outlines do not.

Publishes:
    aruco/image (sensor_msgs/Image): every frame with each marker outlined, its axes drawn and a
        label with its id, distance and viewing angle.
    aruco/detections (aruco_interfaces/MarkerDetectionArray): id, pose, distance and viewing angle
        of every marker in the frame, with the image's header.

Parameters, both required: a wrong marker size would silently scale every distance, so there is no
default to fall back on.
    dictionary (string): OpenCV ArUco dictionary, such as DICT_4X4_50.
    marker_length (double): side of the black square in metres.
"""

from __future__ import annotations

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header

from aruco_detector.detection import (
    Detection,
    MarkerDetector,
    draw_detections,
    quaternion_from_rvec,
)
from aruco_interfaces.msg import MarkerDetection, MarkerDetectionArray


class ArucoDetectorNode(Node):
    """Detects the markers in every frame and publishes the annotated image and the detections."""

    def __init__(self) -> None:
        super().__init__("aruco_detector")

        # Declared with a type and no default. Declaring accepts an unset parameter; reading it with
        # get_parameter does not, so a missing value stops the node here, at start-up.
        self.declare_parameter("marker_length", Parameter.Type.DOUBLE)
        self.declare_parameter("dictionary", Parameter.Type.STRING)
        self.marker_length: float = self.get_parameter("marker_length").value
        dictionary: str = self.get_parameter("dictionary").value

        self.detector = MarkerDetector(dictionary)
        self.bridge = CvBridge()
        self.camera_matrix: np.ndarray | None = None
        self.dist_coeffs: np.ndarray | None = None
        self.ids_in_view: list[int] = []

        # Sensor QoS drops late frames instead of queueing them. The names stay relative so a
        # launch file can remap them to any camera.
        self.create_subscription(Image, "image", self.on_image, qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, "camera_info", self.on_camera_info, qos_profile_sensor_data
        )
        # Default QoS is reliable, which every subscriber accepts.
        self.image_pub = self.create_publisher(Image, "aruco/image", 10)
        self.detections_pub = self.create_publisher(MarkerDetectionArray, "aruco/detections", 10)

    def on_camera_info(self, msg: CameraInfo) -> None:
        # Fixed-size array fields such as k (float64[9]) arrive as numpy arrays.
        self.camera_matrix = np.asarray(msg.k, dtype=float).reshape(3, 3)
        self.dist_coeffs = np.asarray(msg.d, dtype=float)

    def on_image(self, msg: Image) -> None:
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        detections = self.detector.detect(
            frame, self.marker_length, self.camera_matrix, self.dist_coeffs
        )

        annotated = draw_detections(
            frame, detections, self.camera_matrix, self.dist_coeffs, self.marker_length
        )
        out = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        out.header = msg.header  # same stamp and frame as the camera image
        self.image_pub.publish(out)

        if self.camera_matrix is None:
            self.get_logger().warn("no camera_info yet: outlines only, no poses", once=True)
        else:
            self.detections_pub.publish(self.detections_msg(msg.header, detections))
        self.log_changes(detections)

    def detections_msg(self, header: Header, detections: list[Detection]) -> MarkerDetectionArray:
        """The detections of one frame as a message, stamped like the frame."""
        array = MarkerDetectionArray(header=header)
        for d in detections:
            marker = MarkerDetection()
            marker.id = d.marker_id
            position = marker.pose.position
            position.x, position.y, position.z = (float(v) for v in d.tvec)
            orientation = marker.pose.orientation
            orientation.x, orientation.y, orientation.z, orientation.w = quaternion_from_rvec(
                d.rvec
            )
            marker.distance = d.distance
            marker.viewing_angle = d.viewing_angle
            array.markers.append(marker)
        return array

    def log_changes(self, detections: list[Detection]) -> None:
        """Log the ids in view only when they change, so the log reads as markers coming and going."""
        ids = [d.marker_id for d in detections]
        if ids != self.ids_in_view:
            self.get_logger().info(f"markers in view: {ids}")
            self.ids_in_view = ids


def main(args: list[str] | None = None) -> None:
    # Ctrl+C under ros2 launch reaches a node twice: from the terminal and forwarded by launch. The
    # second one can land during the cleanup, so the outer try covers the cleanup too.
    try:
        rclpy.init(args=args)
        node = ArucoDetectorNode()
        try:
            rclpy.spin(node)
        except RuntimeError:
            # rclpy can fail to take a message that arrives while it shuts down; that is not an
            # error. While it is still running, it is.
            if rclpy.ok():
                raise
        finally:
            node.destroy_node()
            rclpy.try_shutdown()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
