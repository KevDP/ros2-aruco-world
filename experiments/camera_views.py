#!/usr/bin/env python3
"""What the robot's camera sees from two poses: markers read, and markers in view but not read.

Gazebo places the robot at its start pose and then turned toward the box. For each frame, the
markers in view come from projecting every marker's corners with the camera's true pose and its
calibration, and the markers read come from the same detector the live node runs. Read markers are
drawn in green with the distance the detector measures, unread ones in amber. Frames are upscaled
before drawing so the labels stay legible when the figure is shown small.

Run from the repository root, with the workspace built and sourced (see experiments/README.md):
    experiments/sim.sh -- python3 experiments/camera_views.py
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import cv2
import numpy as np
import rclpy
from sim_tools import RobotPlacer
from visibility import Visibility
from world import RESULTS, load_world, robot_properties

# Robot poses to look from: name, x, y, heading in degrees.
VIEWS = (("start", -1.5, 0.0, 0.0), ("turned toward the box", -1.5, 0.0, 37.0))
UPSCALE = 2
KEEP_ROWS = 340  # rows of the original frame to keep; below them there is only floor
GAP_PX = 14

# BGR colours, matching the figures of the reports.
READ = (94, 211, 52)
UNREAD = (36, 191, 251)
BACKGROUND = (42, 23, 15)

# Camera optical frame in the robot's base frame: columns are the optical x, y and z axes.
BASE_FROM_OPTICAL = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=float)


def markers_in_view(
    camera_matrix: np.ndarray, size: tuple[int, int], pose: tuple[float, float, float]
) -> dict[int, np.ndarray]:
    """Pixel corners of every marker that faces the camera and lies wholly inside the frame.

    Args:
        camera_matrix: 3x3 intrinsics of the camera.
        size: Frame (width, height) in pixels.
        pose: The robot's true (x, y, heading) in the world.

    Returns:
        Marker id to its (4, 2) pixel corners. Markers hidden behind something are not removed.
    """
    world = load_world()
    props = robot_properties()
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    world_from_base = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    world_from_optical = world_from_base @ BASE_FROM_OPTICAL
    camera = np.array([x + props["camera_x"] * c, y + props["camera_x"] * s, props["camera_z"]])
    half = world.MARKER_SIDE_M / 2
    width, height = size
    found = {}
    for p in world.PLACEMENTS:
        rotation = world.rotation_for(p.facing)
        centre = np.asarray(p.position, dtype=float)
        if rotation[:, 2] @ (camera - centre) <= 0:
            continue  # seen from behind or edge-on
        corners = [
            centre + rotation @ np.array([sx * half, sy * half, 0.0])
            for sx, sy in ((-1, 1), (1, 1), (1, -1), (-1, -1))
        ]
        optical = np.array([world_from_optical.T @ (corner - camera) for corner in corners])
        if np.any(optical[:, 2] <= 0):
            continue
        pixels = (camera_matrix @ (optical / optical[:, 2:]).T).T[:, :2]
        inside = (pixels[:, 0] >= 0) & (pixels[:, 0] < width)
        inside &= (pixels[:, 1] >= 0) & (pixels[:, 1] < height)
        if np.all(inside):
            found[p.marker_id] = pixels
    return found


def _label(image: np.ndarray, text: str, x: int, y: int, colour: tuple[int, int, int]) -> None:
    font, scale, thickness = cv2.FONT_HERSHEY_SIMPLEX, 0.62 * UPSCALE, 2
    (w, h), base = cv2.getTextSize(text, font, scale, thickness)
    x = int(min(max(x, 4), image.shape[1] - w - 8))
    y = int(max(y, h + 8))
    cv2.rectangle(image, (x - 4, y - h - 6), (x + w + 4, y + base), BACKGROUND, -1)
    cv2.putText(image, text, (x, y - 2), font, scale, colour, thickness, cv2.LINE_AA)


def annotate(
    frame: np.ndarray, in_view: dict[int, np.ndarray], read: dict[int, float]
) -> np.ndarray:
    """The frame upscaled, with a box and a label on every marker in view."""
    image = cv2.resize(frame, None, fx=UPSCALE, fy=UPSCALE, interpolation=cv2.INTER_CUBIC)
    for marker_id, pixels in sorted(in_view.items()):
        corners = pixels * UPSCALE
        x0, y0 = (corners.min(axis=0) - 6).astype(int)
        x1, y1 = (corners.max(axis=0) + 6).astype(int)
        colour = READ if marker_id in read else UNREAD
        cv2.rectangle(image, (x0, y0), (x1, y1), colour, 3)
        text = f"{marker_id}: {read[marker_id]:.1f} m" if marker_id in read else str(marker_id)
        _label(image, text, x0, y0 - 6, colour)
    return image[: KEEP_ROWS * UPSCALE]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--out", type=Path, default=RESULTS / "camera", help="output folder")
    args, ros_args = parser.parse_known_args()
    args.out.mkdir(parents=True, exist_ok=True)

    rclpy.init(args=ros_args)
    node = Visibility()
    placer = RobotPlacer()
    node.wait_until_live()
    panels = []
    for name, x, y, heading in VIEWS:
        pose, detections = node.measure(placer, x, y, math.radians(heading))
        frame = node.bridge.imgmsg_to_cv2(node.image, desired_encoding="bgr8")
        size = (frame.shape[1], frame.shape[0])
        in_view = markers_in_view(node.camera_matrix, size, pose)
        read = {d.marker_id: d.distance for d in detections}
        panels.append(annotate(frame, in_view, read))
        print(f"{name}: in view {sorted(in_view)}, read {sorted(read)}")
    gap = np.full((GAP_PX * UPSCALE, panels[0].shape[1], 3), BACKGROUND, dtype=np.uint8)
    figure = np.vstack([panels[0], gap, panels[1]])
    cv2.imwrite(str(args.out / "camera-views.png"), figure)
    print(f"done: {args.out / 'camera-views.png'}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
