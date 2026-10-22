"""ROS 2 node that detects ArUco markers in a camera stream.

Subscribes:
    image (sensor_msgs/Image): the camera stream.
    camera_info (sensor_msgs/CameraInfo): the calibration, needed for poses but not for outlines.

Publishes:
    aruco/image (sensor_msgs/Image): every frame with each marker outlined, its axes drawn and a
        label with its id, distance and viewing angle.
    aruco/detections (aruco_interfaces/MarkerDetectionArray): id, pose, distance and viewing angle
        of every marker in the frame, with the image's header.
    aruco/markers (visualization_msgs/MarkerArray): each detected marker as a panel with its id,
        for RViz. Each message replaces the previous one.

Parameters, both required. With a default, a wrong marker size would quietly scale every distance.
    dictionary (string): OpenCV ArUco dictionary, such as DICT_4X4_50.
    marker_length (double): side of the black square in metres.
"""

from __future__ import annotations

import signal

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray

from aruco_detector.detection import (
    Detection,
    MarkerDetector,
    draw_detections,
    quaternion_from_rvec,
)
from aruco_interfaces.msg import MarkerDetection, MarkerDetectionArray

# How a detected marker looks in RViz
PANEL_THICKNESS_M = 0.01
PANEL_COLOR = ColorRGBA(r=0.2, g=0.85, b=0.35, a=0.9)
LABEL_COLOR = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)


class ArucoDetectorNode(Node):
    """Detects the markers in every frame and publishes the annotated image and the detections."""

    def __init__(self) -> None:
        super().__init__("aruco_detector")

        # Declared with a type and no default. get_parameter raises on one that was never set, which
        # stops the node at start-up.
        self.declare_parameter("marker_length", Parameter.Type.DOUBLE)
        self.declare_parameter("dictionary", Parameter.Type.STRING)
        self.marker_length: float = self.get_parameter("marker_length").value
        dictionary: str = self.get_parameter("dictionary").value

        self.detector = MarkerDetector(dictionary)
        self.bridge = CvBridge()
        self.camera_matrix: np.ndarray | None = None
        self.dist_coeffs: np.ndarray | None = None
        self.ids_in_view: list[int] = []

        # Sensor QoS drops late frames. The names stay relative so a launch file can remap them to
        # any camera.
        self.create_subscription(Image, "image", self.on_image, qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, "camera_info", self.on_camera_info, qos_profile_sensor_data
        )
        # Default QoS is reliable, which every subscriber accepts.
        self.image_pub = self.create_publisher(Image, "aruco/image", 10)
        self.detections_pub = self.create_publisher(MarkerDetectionArray, "aruco/detections", 10)
        self.markers_pub = self.create_publisher(MarkerArray, "aruco/markers", 10)

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
            self.markers_pub.publish(self.markers_msg(msg.header, detections))
        self.log_changes(detections)

    def detections_msg(self, header: Header, detections: list[Detection]) -> MarkerDetectionArray:
        """The detections of one frame as a message, stamped like the frame."""
        array = MarkerDetectionArray(header=header)
        for d in detections:
            marker = MarkerDetection()
            marker.id = d.marker_id
            marker.pose = self.pose_msg(d)
            marker.distance = d.distance
            marker.viewing_angle = d.viewing_angle
            array.markers.append(marker)
        return array

    def markers_msg(self, header: Header, detections: list[Detection]) -> MarkerArray:
        """RViz markers for one frame: each detected marker as a panel, with its id above it.

        The first entry deletes the previous frame's markers. RViz then shows only what is in view,
        with no lifetime to guess.
        """
        markers = [Marker(header=header, action=Marker.DELETEALL)]
        for d in detections:
            pose = self.pose_msg(d)
            panel = Marker(header=header, ns="panels", id=d.marker_id, type=Marker.CUBE, pose=pose)
            panel.scale.x = panel.scale.y = self.marker_length
            panel.scale.z = PANEL_THICKNESS_M
            panel.color = PANEL_COLOR

            label = Marker(header=header, ns="ids", id=d.marker_id, type=Marker.TEXT_VIEW_FACING)
            label.text = str(d.marker_id)
            label.pose.position.x = pose.position.x
            # One marker side above the panel: up in the image is -y in the optical frame.
            label.pose.position.y = pose.position.y - self.marker_length
            label.pose.position.z = pose.position.z
            label.scale.z = self.marker_length  # text height
            label.color = LABEL_COLOR
            markers += [panel, label]
        return MarkerArray(markers=markers)

    @staticmethod
    def pose_msg(d: Detection) -> Pose:
        """A detection's pose as a message: position from tvec, orientation from rvec."""
        pose = Pose()
        position = pose.position
        position.x, position.y, position.z = (float(v) for v in d.tvec)
        orientation = pose.orientation
        orientation.x, orientation.y, orientation.z, orientation.w = quaternion_from_rvec(d.rvec)
        return pose

    def log_changes(self, detections: list[Detection]) -> None:
        """Log the ids in view each time the set changes."""
        ids = [d.marker_id for d in detections]
        if ids != self.ids_in_view:
            self.get_logger().info(f"markers in view: {ids}")
            self.ids_in_view = ids


def main(args: list[str] | None = None) -> None:
    try:
        rclpy.init(args=args)
        node = ArucoDetectorNode()
        try:
            rclpy.spin(node)
        except RuntimeError:
            # rclpy can fail to take a message that arrives during shutdown. That is expected then,
            # and a real error at any other time.
            if rclpy.ok():
                raise
        finally:
            # Under ros2 launch, Ctrl+C reaches a node twice, from the terminal and again from
            # launch. The second one is ignored once the node is shutting down, or it could cut
            # the cleanup or the interpreter's exit short. launch can still end a stuck node with
            # SIGTERM.
            signal.signal(signal.SIGINT, signal.SIG_IGN)
            node.destroy_node()
            rclpy.try_shutdown()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
