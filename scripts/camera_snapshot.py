#!/usr/bin/env python3
"""Save one frame from the robot's camera and mark the ArUco markers it detects in it.

Writes <out>/snapshot.png (what the camera sees) and <out>/snapshot_detected.png (the same frame
with each detected marker outlined and its id), and prints the ids.

Usage:
    ros2 run aruco_world camera_snapshot.py --out /tmp/aruco
"""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

DICTIONARY = cv2.aruco.DICT_4X4_50


def detect(gray):
    """Run the ArUco detector with default parameters on either OpenCV API."""
    dictionary = cv2.aruco.getPredefinedDictionary(DICTIONARY)
    if hasattr(cv2.aruco, "ArucoDetector"):  # OpenCV 4.7 and later
        detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
        corners, ids, _rejected = detector.detectMarkers(gray)
    else:  # OpenCV 4.6, the version in Ubuntu 24.04
        corners, ids, _rejected = cv2.aruco.detectMarkers(
            gray, dictionary, parameters=cv2.aruco.DetectorParameters_create()
        )
    return corners, ids


class Snapshot(Node):
    def __init__(self, out: Path) -> None:
        super().__init__("camera_snapshot")
        self.out = out
        self.bridge = CvBridge()
        self.done = False
        self.create_subscription(Image, "/camera/image", self.on_image, 1)

    def on_image(self, msg: Image) -> None:
        if self.done:
            return
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        corners, ids = detect(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
        self.out.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(self.out / "snapshot.png"), frame)
        marked = frame.copy()
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(marked, corners, ids)
        cv2.imwrite(str(self.out / "snapshot_detected.png"), marked)
        found = sorted(int(i) for i in ids.flatten()) if ids is not None else []
        print(f"frame {msg.width}x{msg.height}, detected ids: {found}")
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
