"""Which markers the robot's camera has in view, from its true pose and the files of the world.

Each marker's square is projected through the simulated pinhole camera, which has no lens
distortion. Nothing here needs ROS.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from world import load_world, robot_properties, solids

# Camera optical frame in the robot's base frame: columns are the optical x, y and z axes.
BASE_FROM_OPTICAL = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=float)
# A square's corners in ArUco order, in units of half its side.
SQUARE = ((-1, 1), (1, 1), (1, -1), (-1, -1))
EDGE_PX = 3  # OpenCV drops a candidate this close to the frame's edge


@dataclass(frozen=True)
class MarkerView:
    """One marker as the camera has it in view.

    Attributes:
        corners: (4, 2) pixel corners of the black square, in ArUco order.
        distance: How far the marker's centre is from the camera, in metres.
        viewing_angle: Degrees off head-on, from the marker's normal to the camera's direction.
    """

    corners: np.ndarray
    distance: float
    viewing_angle: float

    @property
    def side_px(self) -> float:
        """Mean length of the square's four sides in the image."""
        return float(
            np.mean([np.linalg.norm(self.corners[i] - self.corners[i - 1]) for i in range(4)])
        )


def _hits_box(start: np.ndarray, end: np.ndarray, low: np.ndarray, high: np.ndarray) -> bool:
    """Whether the segment between two points crosses an axis-aligned box (slab test)."""
    step = end - start
    enter, leave = 0.0, 1.0
    for axis in range(3):
        if abs(step[axis]) < 1e-12:
            if not low[axis] <= start[axis] <= high[axis]:
                return False
            continue
        t_low, t_high = (
            (low[axis] - start[axis]) / step[axis],
            (high[axis] - start[axis]) / step[axis],
        )
        enter, leave = max(enter, min(t_low, t_high)), min(leave, max(t_low, t_high))
        if enter > leave:
            return False
    return enter < 1.0 - 1e-6 and leave > 1e-6


class CameraModel:
    """The simulated camera on the robot, with every marker and everything that can hide one."""

    def __init__(self, camera_matrix: np.ndarray, size: tuple[int, int]) -> None:
        """Read the markers, the box, the posts and the camera's place on the robot.

        Args:
            camera_matrix: 3x3 intrinsics, as the camera publishes them.
            size: Frame (width, height) in pixels.
        """
        world = load_world()
        props = robot_properties()
        self.camera_matrix = camera_matrix
        self.width, self.height = size
        self._camera_x, self._camera_z = props["camera_x"], props["camera_z"]
        half, panel = world.MARKER_SIDE_M / 2, world.PANEL_SIDE_M / 2
        blocks = solids(world)
        self._markers = []
        self._posts = {}  # marker id to the (low, high) corners of the post under it
        for p in world.PLACEMENTS:
            rotation = world.rotation_for(p.facing)
            normal = rotation[:, 2]
            centre = np.asarray(p.position, dtype=float)
            square = np.array(
                [centre + rotation @ np.array([u * half, v * half, 0.0]) for u, v in SQUARE]
            )
            outline = np.array(
                [centre + rotation @ np.array([u * panel, v * panel, 0.0]) for u, v in SQUARE]
            )
            self._markers.append((p.marker_id, p.mount, centre, normal, square, outline))
            if p.mount == "post":
                post = blocks[f"post_{p.marker_id}"]
                self._posts[p.marker_id] = (np.array(post.low), np.array(post.high))
        self._box = (np.array(blocks["box"].low), np.array(blocks["box"].high))

    def markers_in_view(
        self, pose: tuple[float, float, float], whole_panel: bool = True, unhidden: bool = True
    ) -> dict[int, MarkerView]:
        """Every marker that faces the camera from inside the frame.

        Args:
            pose: The robot's true (x, y, heading) in the world.
            whole_panel: Require the white border inside the frame too, at least ``EDGE_PX`` from
                its edge, as detection does. Otherwise the black square inside the frame is enough.
            unhidden: Leave out markers the box or a post hides from the camera.

        Returns:
            Marker id to its view.
        """
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        world_from_optical = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]) @ BASE_FROM_OPTICAL
        camera = np.array([x + self._camera_x * c, y + self._camera_x * s, self._camera_z])
        found = {}
        for marker_id, mount, centre, normal, square, outline in self._markers:
            sight = camera - centre
            if normal @ sight <= 0:
                continue  # seen from behind or edge-on
            points = np.vstack([square, outline]) if whole_panel else square
            optical = (points - camera) @ world_from_optical
            if np.any(optical[:, 2] <= 0):
                continue
            pixels = (self.camera_matrix @ (optical / optical[:, 2:]).T).T[:, :2]
            margin = EDGE_PX if whole_panel else 0
            inside = (pixels[:, 0] >= margin) & (pixels[:, 0] < self.width - margin)
            inside &= (pixels[:, 1] >= margin) & (pixels[:, 1] < self.height - margin)
            if not np.all(inside):
                continue
            if unhidden and self._hidden(camera, centre, marker_id, mount):
                continue
            distance = float(np.linalg.norm(sight))
            angle = math.degrees(math.acos(max(-1.0, min(1.0, float(normal @ sight) / distance))))
            found[marker_id] = MarkerView(pixels[:4], distance, angle)
        return found

    def _hidden(self, camera: np.ndarray, centre: np.ndarray, marker_id: int, mount: str) -> bool:
        """Whether the box or another marker's post stands between the camera and a marker."""
        if mount != "box" and _hits_box(camera, centre, *self._box):
            return True
        return any(
            _hits_box(camera, centre, *post) for i, post in self._posts.items() if i != marker_id
        )
