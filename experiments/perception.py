#!/usr/bin/env python3
"""How often the detector reads the markers in view, and how far off the pose it measures is.

The robot is placed on the visibility experiment's grid. At each point its headings are turned by a
random offset, which spreads the markers over more angles and image positions than fixed headings
would. For every marker in view of each frame, with its white border inside the frame and nothing
hiding it, the script records its true distance and viewing angle, its size in the image, whether
the detector reads it with whole cells (OpenCV's default) and with cell centres (the live node's
reading), and the distance and viewing angle it measures. Detections with the wrong id, or where
no marker is, go to their own file.

Run from the repository root, with the workspace built and sourced (see experiments/README.md):
    experiments/sim.sh -- python3 experiments/perception.py
Then build the tables and figures with experiments/perception_report.py.
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
from aruco_detector.detection import Detection, MarkerDetector
from camera_model import CameraModel, MarkerView
from sim_tools import RobotPlacer
from visibility import Visibility, grid_positions
from world import RESULTS, marker_settings

GRID_STEP_M = 0.5
HEADINGS = 12
SEED = 11  # for the heading offset at each grid point
MATCH_PX = 15.0  # a detection belongs to a marker whose centre is this close, or within one side


def _match(detections: list[Detection], marker_id: int, view: MarkerView) -> Detection | None:
    """The detection of this marker at the place the camera model puts it, if there is one."""
    centre = view.corners.mean(axis=0)
    reach = max(MATCH_PX, view.side_px)
    for d in detections:
        if d.marker_id == marker_id and np.linalg.norm(d.corners.mean(axis=0) - centre) < reach:
            return d
    return None


def _anomalies(detections: list[Detection], drawn: dict[int, MarkerView]) -> list[tuple]:
    """Detections that match no marker drawn at their place, as (id, x, y, kind)."""
    found = []
    for d in detections:
        centre = d.corners.mean(axis=0)
        near = [
            i for i, v in drawn.items()
            if np.linalg.norm(v.corners.mean(axis=0) - centre) < max(MATCH_PX, v.side_px)
        ]  # fmt: skip
        if d.marker_id in near:
            continue
        kind = "wrong id" if near else "no marker"
        found.append((d.marker_id, round(float(centre[0]), 1), round(float(centre[1]), 1), kind))
    return found


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--out", type=Path, default=RESULTS / "perception", help="output folder")
    parser.add_argument("--step", type=float, default=GRID_STEP_M, help="grid step in metres")
    parser.add_argument("--headings", type=int, default=HEADINGS, help="headings per point")
    parser.add_argument("--seed", type=int, default=SEED, help="seed of the heading offsets")
    args, ros_args = parser.parse_known_args()

    positions = grid_positions(args.step)
    total = len(positions) * args.headings
    args.out.mkdir(parents=True, exist_ok=True)
    rng = random.Random(args.seed)

    rclpy.init(args=ros_args)
    node = Visibility()  # cell centres, like the live node
    whole_cells = MarkerDetector(marker_settings()[0], cell_centres=False)
    placer = RobotPlacer()
    node.wait_until_live()
    model = None
    print(f"{len(positions)} positions x {args.headings} headings = {total} frames")

    start = time.monotonic()
    with (
        open(args.out / "frames.csv", "w", newline="") as frames_file,
        open(args.out / "markers.csv", "w", newline="") as markers_file,
        open(args.out / "anomalies.csv", "w", newline="") as anomalies_file,
    ):
        frames = csv.writer(frames_file)
        markers = csv.writer(markers_file)
        anomalies = csv.writer(anomalies_file)
        frames.writerow(["frame", "x_cmd", "y_cmd", "yaw_cmd", "x", "y", "yaw"])
        markers.writerow(
            ["frame", "marker", "distance", "viewing_angle_deg", "side_px", "read_whole_cells",
             "read_cell_centres", "measured_distance", "measured_angle_deg"]
        )  # fmt: skip
        anomalies.writerow(["frame", "reading", "marker", "x_px", "y_px", "kind"])
        frame_id = 0
        for x, y in positions:
            offset = rng.uniform(0, 2 * math.pi / args.headings)
            for k in range(args.headings):
                yaw = math.remainder(offset + 2 * math.pi * k / args.headings, 2 * math.pi)
                pose, centres = node.measure(placer, x, y, yaw)
                image = node.bridge.imgmsg_to_cv2(node.image, desired_encoding="bgr8")
                if model is None:
                    model = CameraModel(node.camera_matrix, (image.shape[1], image.shape[0]))
                wholes = whole_cells.detect(
                    image, node.marker_length, node.camera_matrix, node.dist_coeffs
                )
                frames.writerow(
                    [frame_id, x, y, f"{yaw:.4f}", f"{pose[0]:.4f}", f"{pose[1]:.4f}",
                     f"{pose[2]:.4f}"]
                )  # fmt: skip
                for marker_id, view in sorted(model.markers_in_view(pose).items()):
                    by_centre = _match(centres, marker_id, view)
                    by_whole = _match(wholes, marker_id, view)
                    measured = by_centre or by_whole
                    markers.writerow(
                        [frame_id, marker_id, f"{view.distance:.4f}", f"{view.viewing_angle:.2f}",
                         f"{view.side_px:.2f}", int(by_whole is not None),
                         int(by_centre is not None),
                         f"{measured.distance:.4f}" if measured else "",
                         f"{math.degrees(measured.viewing_angle):.2f}" if measured else ""]
                    )  # fmt: skip
                drawn = model.markers_in_view(pose, whole_panel=False, unhidden=False)
                for reading, found in (("whole cells", wholes), ("cell centres", centres)):
                    for row in _anomalies(found, drawn):
                        anomalies.writerow([frame_id, reading, *row])
                frame_id += 1
                if frame_id % 100 == 0:
                    rate = frame_id / (time.monotonic() - start)
                    print(
                        f"  {frame_id}/{total} frames, {(total - frame_id) / rate / 60:.1f} min left",
                        flush=True,
                    )
    print(f"done: {args.out}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
