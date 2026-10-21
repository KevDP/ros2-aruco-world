#!/usr/bin/env python3
"""Which markers the robot reads from each spot in the room.

Gazebo places the robot (it never drives) at every point of a grid over the floor, facing each of
a set of headings. At each pose the script waits for a camera frame taken after the move, runs the
same detector the live node runs, and records the robot's true pose and every marker read, with
its distance and viewing angle.

Run from the repository root, with the workspace built and sourced (see experiments/README.md):
    experiments/sim.sh -- python3 experiments/visibility.py
Then build the tables and figures with experiments/visibility_report.py.
"""

from __future__ import annotations

import argparse
import csv
import math
import time
from collections.abc import Callable
from pathlib import Path

import numpy as np
import rclpy
from aruco_detector.detection import MarkerDetector
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from sim_tools import RobotPlacer, seconds, yaw_of
from world import RESULTS, load_world, marker_settings

GRID_STEP_M = 0.5
HEADINGS = 12  # 30 degrees apart, so with a 60-degree camera every direction is seen twice
CLEARANCE_M = 0.30  # robot centre to any wall, post or box: the robot reaches ~0.22 m from it
SETTLE_S = 0.3  # simulated time between a move and the frame that is used
WAIT_TIMEOUT_S = 10.0  # wall time to wait for that frame before giving up


# ---------------------------------------------------------------------------
# Poses
# ---------------------------------------------------------------------------


def _distance_to_box(x: float, y: float, centre: tuple[float, float], size: tuple) -> float:
    dx = max(abs(x - centre[0]) - size[0] / 2, 0.0)
    dy = max(abs(y - centre[1]) - size[1] / 2, 0.0)
    return math.hypot(dx, dy)


def grid_positions(step: float) -> list[tuple[float, float]]:
    """Grid points on the free floor: away from the walls, the posts and the box."""
    world = load_world()
    limit = world.ROOM_HALF - CLEARANCE_M
    posts = [p.position[:2] for p in world.PLACEMENTS if p.mount == "post"]
    axis = np.arange(-limit, limit + 1e-9, step)
    axis = axis - (axis[0] + axis[-1]) / 2  # centred on the room
    positions = []
    for x in axis:
        for y in axis:
            if any(math.hypot(x - px, y - py) < CLEARANCE_M for px, py in posts):
                continue
            if _distance_to_box(x, y, world.BOX_CENTRE, world.BOX_SIZE) < CLEARANCE_M:
                continue
            positions.append((round(float(x), 3), round(float(y), 3)))
    return positions


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------


class Visibility(Node):
    """Collects one camera frame and the true pose after each move, and detects the markers."""

    def __init__(self) -> None:
        super().__init__("visibility_experiment")
        dictionary, self.marker_length = marker_settings()
        self.detector = MarkerDetector(dictionary)
        self.bridge = CvBridge()
        self.camera_matrix: np.ndarray | None = None
        self.dist_coeffs: np.ndarray | None = None
        self.image: Image | None = None
        self.truth: Odometry | None = None
        self.create_subscription(CameraInfo, "/camera/camera_info", self._on_camera_info, 1)
        self.create_subscription(Image, "/camera/image", self._on_image, 1)
        self.create_subscription(Odometry, "/ground_truth", self._on_truth, 1)

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self.camera_matrix = np.asarray(msg.k, dtype=float).reshape(3, 3)
        self.dist_coeffs = np.asarray(msg.d, dtype=float)

    def _on_image(self, msg: Image) -> None:
        self.image = msg

    def _on_truth(self, msg: Odometry) -> None:
        self.truth = msg

    def _wait(self, ready: Callable[[], bool]) -> None:
        deadline = time.monotonic() + WAIT_TIMEOUT_S
        while not ready():
            if time.monotonic() > deadline:
                raise RuntimeError("no camera frame or true pose: is the simulation running?")
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_until_live(self) -> None:
        self._wait(lambda: all(v is not None for v in (self.camera_matrix, self.image, self.truth)))

    def measure(self, placer: RobotPlacer, x: float, y: float, yaw: float) -> tuple[tuple, list]:
        """Place the robot, then detect on the first frame taken ``SETTLE_S`` after the move.

        Returns:
            ((true x, true y, true yaw), detections) for that frame.
        """
        before = seconds(self.image.header.stamp)
        placer.place(x, y, yaw)
        self._wait(
            lambda: (
                seconds(self.image.header.stamp) >= before + SETTLE_S
                and seconds(self.truth.header.stamp) >= before + SETTLE_S
            )
        )
        frame = self.bridge.imgmsg_to_cv2(self.image, desired_encoding="bgr8")
        detections = self.detector.detect(
            frame, self.marker_length, self.camera_matrix, self.dist_coeffs
        )
        pose = self.truth.pose.pose
        return (pose.position.x, pose.position.y, yaw_of(pose.orientation)), detections


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--out", type=Path, default=RESULTS / "visibility", help="output folder")
    parser.add_argument("--step", type=float, default=GRID_STEP_M, help="grid step in metres")
    parser.add_argument("--headings", type=int, default=HEADINGS, help="headings per point")
    args, ros_args = parser.parse_known_args()

    positions = grid_positions(args.step)
    headings = [math.pi * (2 * k / args.headings - 1) for k in range(args.headings)]
    total = len(positions) * len(headings)
    args.out.mkdir(parents=True, exist_ok=True)

    rclpy.init(args=ros_args)
    node = Visibility()
    placer = RobotPlacer()
    node.wait_until_live()
    print(f"{len(positions)} positions x {len(headings)} headings = {total} poses")

    start = time.monotonic()
    with (
        open(args.out / "poses.csv", "w", newline="") as poses_file,
        open(args.out / "detections.csv", "w", newline="") as detections_file,
    ):
        poses = csv.writer(poses_file)
        detections = csv.writer(detections_file)
        poses.writerow(["pose", "x_cmd", "y_cmd", "yaw_cmd", "x", "y", "yaw", "markers"])
        detections.writerow(["pose", "marker", "distance", "viewing_angle_deg"])
        pose_id = 0
        for x, y in positions:
            for yaw in headings:
                (tx, ty, tyaw), found = node.measure(placer, x, y, yaw)
                poses.writerow(
                    [pose_id, x, y, f"{yaw:.4f}", f"{tx:.4f}", f"{ty:.4f}", f"{tyaw:.4f}",
                     len(found)]
                )  # fmt: skip
                for d in found:
                    detections.writerow(
                        [pose_id, d.marker_id, f"{d.distance:.4f}",
                         f"{math.degrees(d.viewing_angle):.2f}"]
                    )  # fmt: skip
                pose_id += 1
                if pose_id % 100 == 0:
                    rate = pose_id / (time.monotonic() - start)
                    print(
                        f"  {pose_id}/{total} poses, {(total - pose_id) / rate / 60:.1f} min left"
                    )
    print(f"done: {args.out}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
