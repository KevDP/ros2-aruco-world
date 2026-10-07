#!/usr/bin/env python3
"""Save one frame from the robot's camera, and the same frame with the markers detected in it.

Writes <out>/snapshot.png (what the camera sees) and <out>/snapshot_detected.png (the same frame
annotated the way the live detector does it), and prints each marker's id, distance and viewing
angle. Dictionary and marker size come from the world's generated detector parameters, and the
detection code from aruco_detector, so the snapshot and the live node always agree.

Usage:
    ros2 run aruco_world camera_snapshot.py --out /tmp/aruco
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from aruco_detector.detection import MarkerDetector, draw_detections
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


def marker_settings() -> tuple[str, float]:
    """Dictionary name and marker size from the parameters build_world.py generated."""
    path = Path(get_package_share_directory("aruco_world")) / "config" / "aruco_markers.yaml"
    params = yaml.safe_load(path.read_text())["aruco_detector"]["ros__parameters"]
    return params["dictionary"], float(params["marker_length"])


class Snapshot(Node):
    def __init__(self, out: Path) -> None:
        super().__init__("camera_snapshot")
        dictionary, self.marker_length = marker_settings()
        self.detector = MarkerDetector(dictionary)
        self.out = out
        self.bridge = CvBridge()
        self.camera_matrix: np.ndarray | None = None
        self.dist_coeffs: np.ndarray | None = None
        self.done = False
        self.create_subscription(CameraInfo, "/camera/camera_info", self.on_camera_info, 1)
        self.create_subscription(Image, "/camera/image", self.on_image, 1)

    def on_camera_info(self, msg: CameraInfo) -> None:
        self.camera_matrix = np.asarray(msg.k, dtype=float).reshape(3, 3)
        self.dist_coeffs = np.asarray(msg.d, dtype=float)

    def on_image(self, msg: Image) -> None:
        if self.done or self.camera_matrix is None:  # wait for the calibration first
            return
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        detections = self.detector.detect(
            frame, self.marker_length, self.camera_matrix, self.dist_coeffs
        )
        annotated = draw_detections(
            frame, detections, self.camera_matrix, self.dist_coeffs, self.marker_length
        )
        self.out.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(self.out / "snapshot.png"), frame)
        cv2.imwrite(str(self.out / "snapshot_detected.png"), annotated)
        print(f"frame {msg.width}x{msg.height}, {len(detections)} markers detected")
        for d in detections:
            print(
                f"  id {d.marker_id}: {d.distance:.2f} m, "
                f"viewing angle {math.degrees(d.viewing_angle):.0f} deg"
            )
        self.done = True


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--out", type=Path, default=Path("/tmp/aruco"), help="output folder")
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = Snapshot(args.out)
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.5)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
