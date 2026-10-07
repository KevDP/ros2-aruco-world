"""ArUco detection and pose estimation on one image, with no ROS dependency.

Kept apart from the node so the same code serves the live node, one-off tools and the tests, and so
it can be checked on synthetic images where the true pose is known.

Conventions follow OpenCV: poses are in the camera optical frame (x right, y down, z forward), in
metres, and a marker's own z axis points out of its printed side.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import cv2
import numpy as np

# ---------------------------------------------------------------------------
# Detections
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Detection:
    """One marker in one image.

    Attributes:
        marker_id: Id in the detector's dictionary.
        corners: (4, 2) pixel corners in ArUco order: top-left, top-right, bottom-right and
            bottom-left of the printed square.
        rvec: Rotation of the marker in the camera optical frame as a Rodrigues vector, or None
            when no calibration or marker size was given.
        tvec: Position of the marker centre in metres, or None like ``rvec``.
    """

    marker_id: int
    corners: np.ndarray
    rvec: np.ndarray | None = None
    tvec: np.ndarray | None = None

    @property
    def has_pose(self) -> bool:
        return self.rvec is not None and self.tvec is not None

    @property
    def distance(self) -> float:
        """Metres from the camera centre to the marker centre."""
        _, tvec = self._pose()
        return float(np.linalg.norm(tvec))

    @property
    def viewing_angle(self) -> float:
        """Radians between the marker's normal and the line of sight to the camera; 0 is head-on."""
        rvec, tvec = self._pose()
        normal = cv2.Rodrigues(rvec)[0][:, 2]
        to_camera = -tvec / np.linalg.norm(tvec)
        return math.acos(max(-1.0, min(1.0, float(normal @ to_camera))))

    def _pose(self) -> tuple[np.ndarray, np.ndarray]:
        if self.rvec is None or self.tvec is None:
            raise ValueError(
                f"marker {self.marker_id} has no pose: detect with a calibration and a marker size"
            )
        return self.rvec, self.tvec


class MarkerDetector:
    """ArUco detection for one dictionary, on either OpenCV API.

    OpenCV 4.7 moved ArUco to an ``ArucoDetector`` object; Ubuntu 24.04 ships 4.6, which only has
    the older free functions. The choice is made once, here.

    Corners are refined to subpixel precision: the pose comes from four corners only, so a corner
    left on the pixel grid shrinks the square and inflates the distance, more so the smaller the
    marker looks.
    """

    def __init__(self, dictionary_name: str) -> None:
        if not dictionary_name.startswith("DICT_") or not hasattr(cv2.aruco, dictionary_name):
            raise ValueError(
                f"unknown ArUco dictionary {dictionary_name!r}: expected a cv2.aruco.DICT_* name"
            )
        self._dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary_name))
        if hasattr(cv2.aruco, "ArucoDetector"):  # OpenCV 4.7 and later
            parameters = cv2.aruco.DetectorParameters()
            parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            self._detector = cv2.aruco.ArucoDetector(self._dictionary, parameters)
            self._parameters = None
        else:  # OpenCV 4.6
            self._detector = None
            self._parameters = cv2.aruco.DetectorParameters_create()
            self._parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

    def detect(
        self,
        image: np.ndarray,
        marker_length: float | None = None,
        camera_matrix: np.ndarray | None = None,
        dist_coeffs: np.ndarray | None = None,
    ) -> list[Detection]:
        """Markers in a BGR or grayscale image, sorted by id.

        Args:
            image: The frame, BGR or grayscale.
            marker_length: Side of the black square in metres; needed for poses.
            camera_matrix: 3x3 intrinsics; needed for poses.
            dist_coeffs: Distortion coefficients, or None for an undistorted image.

        Returns:
            One ``Detection`` per marker, with a pose when ``marker_length`` and ``camera_matrix``
            are both given.
        """
        gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        if self._detector is not None:
            corners, ids, _rejected = self._detector.detectMarkers(gray)
        else:
            corners, ids, _rejected = cv2.aruco.detectMarkers(
                gray, self._dictionary, parameters=self._parameters
            )
        if ids is None:
            return []

        posed = marker_length is not None and camera_matrix is not None
        detections = []
        for marker_corners, marker_id in zip(corners, ids.flatten(), strict=True):
            points = marker_corners.reshape(4, 2).astype(np.float64)
            rvec = tvec = None
            if posed:
                rvec, tvec = estimate_pose(points, marker_length, camera_matrix, dist_coeffs)
            detections.append(Detection(int(marker_id), points, rvec, tvec))
        return sorted(detections, key=lambda d: d.marker_id)


# ---------------------------------------------------------------------------
# Pose
# ---------------------------------------------------------------------------


def estimate_pose(
    corners: np.ndarray,
    marker_length: float,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    """Pose of a square marker from its four corners.

    Uses SQPnP rather than IPPE_SQUARE, the usual choice for markers: SQPnP is globally optimal, so
    the pose it returns is the one that best fits the corners, and a marker is never flipped to
    face away from the camera.

    Args:
        corners: (4, 2) pixel corners in ArUco order.
        marker_length: Side of the black square in metres.
        camera_matrix: 3x3 intrinsics.
        dist_coeffs: Distortion coefficients, or None for none.

    Returns:
        (rvec, tvec): the marker's rotation as a Rodrigues vector and its centre in metres, both in
        the camera optical frame.
    """
    if marker_length <= 0:
        raise ValueError(f"marker_length must be positive, got {marker_length}")
    half = marker_length / 2
    # The printed square in its own frame (x right, y up), in ArUco's corner order.
    object_points = np.array(
        [[-half, half, 0.0], [half, half, 0.0], [half, -half, 0.0], [-half, -half, 0.0]]
    )
    found, rvec, tvec = cv2.solvePnP(
        object_points, corners, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_SQPNP
    )
    if not found:
        raise RuntimeError("solvePnP found no pose for these marker corners")
    return rvec.reshape(3), tvec.reshape(3)


def quaternion_from_rvec(rvec: np.ndarray) -> tuple[float, float, float, float]:
    """(x, y, z, w) of the rotation in a Rodrigues vector, the order geometry_msgs uses."""
    angle = float(np.linalg.norm(rvec))
    if angle == 0.0:
        return 0.0, 0.0, 0.0, 1.0
    axis = np.asarray(rvec, dtype=float).reshape(3) / angle
    s = math.sin(angle / 2)
    return float(axis[0] * s), float(axis[1] * s), float(axis[2] * s), math.cos(angle / 2)


# ---------------------------------------------------------------------------
# Drawing
# ---------------------------------------------------------------------------

LABEL_FONT = cv2.FONT_HERSHEY_SIMPLEX
LABEL_SCALE = 0.5


def draw_detections(
    image: np.ndarray,
    detections: list[Detection],
    camera_matrix: np.ndarray | None = None,
    dist_coeffs: np.ndarray | None = None,
    marker_length: float | None = None,
) -> np.ndarray:
    """Copy of a BGR image with every marker outlined and labelled.

    Markers with a pose also get their axes and, in the label, distance and viewing angle.
    """
    out = image.copy()
    if not detections:
        return out
    corners = [d.corners.reshape(1, 4, 2).astype(np.float32) for d in detections]
    cv2.aruco.drawDetectedMarkers(out, corners)
    distortion = dist_coeffs if dist_coeffs is not None else np.zeros(5)
    for d in detections:
        label = f"id {d.marker_id}"
        if d.has_pose and camera_matrix is not None and marker_length is not None:
            cv2.drawFrameAxes(out, camera_matrix, distortion, d.rvec, d.tvec, marker_length / 2)
            label += f"  {d.distance:.2f} m  {math.degrees(d.viewing_angle):.0f} deg"
        x, y = d.corners.min(axis=0)
        _put_label(out, label, int(x), int(y) - 8)
    return out


def _put_label(image: np.ndarray, text: str, x: int, y: int) -> None:
    """White text with a black outline, kept inside the frame."""
    (width, height), _ = cv2.getTextSize(text, LABEL_FONT, LABEL_SCALE, 3)
    x = max(0, min(x, image.shape[1] - width - 1))
    y = max(y, height + 2)
    for color, thickness in (((0, 0, 0), 3), ((255, 255, 255), 1)):
        cv2.putText(image, text, (x, y), LABEL_FONT, LABEL_SCALE, color, thickness, cv2.LINE_AA)
