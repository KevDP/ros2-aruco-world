#!/usr/bin/env python3
"""Where each marker the camera reads puts the robot, against where Gazebo has it.

The robot is placed on the visibility experiment's grid, with its headings turned by a random offset
at each point, as in the perception experiment but with offsets of its own. Each frame is read with
cell centres, like the live detector, once for each corner window compared. Every marker read gives
a fix through the localizer's own code: the marker's place in marker_map.yaml, the pose the detector
measured, and the camera's place on the robot from TF. Every fix is kept, however far away its
marker is, and experiments/localization_report.py applies the localizer's distance limit and
covariance afterwards. The markers in view of each frame, and which windows read them, go to their
own file.

Run from the repository root, with the workspace built and sourced (see experiments/README.md):
    experiments/sim.sh -- python3 experiments/localization.py
Then build the tables and figures with experiments/localization_report.py.
"""

from __future__ import annotations

import argparse
import csv
import math
import random
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from aruco_detector.detection import (
    CORNER_WINDOW_PX,
    OPENCV_CORNER_WINDOW_PX,
    MarkerDetector,
    quaternion_from_rvec,
)
from camera_model import CameraModel
from rclpy.time import Time
from sim_tools import RobotPlacer
from tf2_ros import Buffer, TransformException, TransformListener
from visibility import Visibility, grid_positions
from world import RESULTS, localization_core, marker_map, marker_settings

GRID_STEP_M = 0.5
HEADINGS = 12
SEED = 5  # heading offsets of its own, apart from the perception experiment's
# Corner refinement windows compared, counted like CORNER_WINDOW_PX, which is the live detector's,
# with OpenCV's own first.
WINDOWS = (OPENCV_CORNER_WINDOW_PX, 3, 2, 1)
BASE_FRAME = "base_footprint"  # the frame the localizer places, as in aruco_localizer.yaml
TF_TIMEOUT_S = 10.0  # wall time to wait for the camera's place on the robot

core = localization_core()


class LocalizationRun(Visibility):
    """The visibility experiment's node, with a TF lookup of where the camera sits on the robot."""

    def __init__(self) -> None:
        super().__init__(cell_centres=True)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def base_from_camera(self) -> np.ndarray:
        """Where the camera sits on the robot, from TF, as the localizer looks it up."""
        deadline = time.monotonic() + TF_TIMEOUT_S
        while True:
            try:
                t = self.tf_buffer.lookup_transform(BASE_FRAME, self.image.header.frame_id, Time())
                break
            except TransformException as e:
                if time.monotonic() > deadline:
                    raise RuntimeError(f"the camera's frame is not in TF: {e}") from e
                rclpy.spin_once(self, timeout_sec=0.05)
        p, q = t.transform.translation, t.transform.rotation
        return core.pose_matrix((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))


def true_sight(marker: np.ndarray, map_from_camera: np.ndarray) -> tuple[float, float, float]:
    """Distance, viewing angle in degrees and heading of the line of sight, from the true pose.

    The viewing angle is 0 when the camera sits on the marker's normal, and the heading runs from
    the camera to the marker.
    """
    to_marker = marker[:3, 3] - map_from_camera[:3, 3]
    distance = float(np.linalg.norm(to_marker))
    facing = float(-to_marker @ marker[:3, 2]) / distance
    angle = math.degrees(math.acos(max(-1.0, min(1.0, facing))))
    return distance, angle, math.atan2(to_marker[1], to_marker[0])


def rotation_error(estimate: np.ndarray, truth: np.ndarray) -> float:
    """Degrees between two rotations, the angle of the turn that takes one to the other."""
    cosine = (np.trace(estimate.T @ truth) - 1) / 2
    return math.degrees(math.acos(max(-1.0, min(1.0, float(cosine)))))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--out", type=Path, default=RESULTS / "localization", help="output folder")
    parser.add_argument("--step", type=float, default=GRID_STEP_M, help="grid step in metres")
    parser.add_argument("--headings", type=int, default=HEADINGS, help="headings per point")
    parser.add_argument("--seed", type=int, default=SEED, help="seed of the heading offsets")
    args, ros_args = parser.parse_known_args()

    positions = grid_positions(args.step)
    total = len(positions) * args.headings
    args.out.mkdir(parents=True, exist_ok=True)
    rng = random.Random(args.seed)
    markers = core.build_marker_map(marker_map())
    dictionary, marker_length = marker_settings()
    readers = {w: MarkerDetector(dictionary, cell_centres=True, corner_window=w) for w in WINDOWS}

    rclpy.init(args=ros_args)
    node = LocalizationRun()
    placer = RobotPlacer()
    node.wait_until_live()
    base_from_camera = node.base_from_camera()
    model = CameraModel(node.camera_matrix, (node.image.width, node.image.height))
    print(f"{len(positions)} positions x {args.headings} headings = {total} frames")

    unknown: dict[int, dict[int, int]] = {}  # window to the ids it read that the map lacks, counted
    start = time.monotonic()
    with (
        open(args.out / "frames.csv", "w", newline="") as frames_file,
        open(args.out / "views.csv", "w", newline="") as views_file,
        open(args.out / "fixes.csv", "w", newline="") as fixes_file,
    ):
        frames = csv.writer(frames_file, lineterminator="\n")
        views = csv.writer(views_file, lineterminator="\n")
        fixes = csv.writer(fixes_file, lineterminator="\n")
        frames.writerow(["frame", "x_cmd", "y_cmd", "yaw_cmd", "x", "y", "yaw"])
        views.writerow(
            ["frame", "marker", "distance", "viewing_angle_deg", "side_px"]
            + [f"read_w{w}" for w in WINDOWS]
        )
        fixes.writerow(
            ["frame", "window", "marker", "in_view", "side_px", "distance", "viewing_angle_deg",
             "sight", "measured_distance", "measured_angle_deg", "x", "y", "yaw", "fix_sight",
             "rotation_error_deg"]
        )  # fmt: skip
        frame_id = 0
        for x, y in positions:
            offset = rng.uniform(0, 2 * math.pi / args.headings)
            for k in range(args.headings):
                yaw = math.remainder(offset + 2 * math.pi * k / args.headings, 2 * math.pi)
                pose, _ = node.measure(placer, x, y, yaw)
                image = node.bridge.imgmsg_to_cv2(node.image, desired_encoding="bgr8")
                frames.writerow(
                    [frame_id, x, y, f"{yaw:.4f}", f"{pose[0]:.4f}", f"{pose[1]:.4f}",
                     f"{pose[2]:.4f}"]
                )  # fmt: skip
                map_from_base = core.pose_matrix(
                    (pose[0], pose[1], 0.0), core.yaw_quaternion(pose[2])
                )
                map_from_camera = map_from_base @ base_from_camera
                in_view = model.markers_in_view(pose)
                read = {}
                for window, reader in readers.items():
                    detections = reader.detect(
                        image, marker_length, node.camera_matrix, node.dist_coeffs
                    )
                    read[window] = {d.marker_id for d in detections}
                    for d in detections:
                        marker = markers.get(d.marker_id)
                        if marker is None:
                            seen = unknown.setdefault(window, {})
                            seen[d.marker_id] = seen.get(d.marker_id, 0) + 1
                            continue
                        camera_from_marker = core.pose_matrix(d.tvec, quaternion_from_rvec(d.rvec))
                        fix = core.locate(marker, camera_from_marker, base_from_camera)
                        distance, angle, sight = true_sight(marker, map_from_camera)
                        view = in_view.get(d.marker_id)
                        truth = core.invert(map_from_camera) @ marker  # camera_from_marker
                        fixes.writerow(
                            [frame_id, window, d.marker_id, int(view is not None),
                             f"{view.side_px:.2f}" if view else "", f"{distance:.4f}",
                             f"{angle:.2f}", f"{sight:.4f}", f"{d.distance:.4f}",
                             f"{math.degrees(d.viewing_angle):.2f}", f"{fix.x:.4f}",
                             f"{fix.y:.4f}", f"{fix.yaw:.4f}", f"{fix.sight:.4f}",
                             f"{rotation_error(camera_from_marker[:3, :3], truth[:3, :3]):.2f}"]
                        )  # fmt: skip
                for marker_id, view in sorted(in_view.items()):
                    views.writerow(
                        [frame_id, marker_id, f"{view.distance:.4f}",
                         f"{view.viewing_angle:.2f}", f"{view.side_px:.2f}"]
                        + [int(marker_id in read[w]) for w in WINDOWS]
                    )  # fmt: skip
                frame_id += 1
                if frame_id % 100 == 0:
                    rate = frame_id / (time.monotonic() - start)
                    print(
                        f"  {frame_id}/{total} frames, {(total - frame_id) / rate / 60:.1f} min left",
                        flush=True,
                    )

    settings = {
        "grid_step_m": args.step,
        "headings": args.headings,
        "seed": args.seed,
        "frames": frame_id,
        "windows": list(WINDOWS),
        "live_window": CORNER_WINDOW_PX,
        "base_from_camera": [[round(float(v), 6) for v in row] for row in base_from_camera],
        "unknown_ids_read": unknown,
    }
    (args.out / "settings.yaml").write_text(yaml.safe_dump(settings, sort_keys=False))
    print(f"done: {args.out}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
