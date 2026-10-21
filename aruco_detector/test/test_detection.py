"""Detection and pose on synthetic images, where the true pose is known."""

from __future__ import annotations

import math

import cv2
import numpy as np
import pytest
from aruco_detector.detection import (
    Detection,
    MarkerDetector,
    draw_detections,
    quaternion_from_rvec,
)

DICTIONARY = "DICT_4X4_50"
MARKER_LENGTH = 0.15
IMAGE_SIZE = (640, 480)
# Pinhole camera with a 60 degree horizontal field of view at 640 px, like the simulated one.
FOCAL_PX = 320 / math.tan(math.radians(30))
CAMERA = np.array([[FOCAL_PX, 0.0, 320.0], [0.0, FOCAL_PX, 240.0], [0.0, 0.0, 1.0]])


def _marker_texture(marker_id: int, side_px: int = 200) -> tuple[np.ndarray, int]:
    """The marker with a white quiet zone, and the quiet zone's width in pixels."""
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, DICTIONARY))
    if hasattr(cv2.aruco, "generateImageMarker"):  # OpenCV 4.7 and later
        marker = cv2.aruco.generateImageMarker(dictionary, marker_id, side_px)
    else:  # OpenCV 4.6
        marker = cv2.aruco.drawMarker(dictionary, marker_id, side_px)
    quiet = side_px // 4
    texture = cv2.copyMakeBorder(marker, quiet, quiet, quiet, quiet, cv2.BORDER_CONSTANT, value=255)
    return texture, quiet


def _render(marker_id: int, rvec: np.ndarray, tvec: np.ndarray) -> np.ndarray:
    """Grayscale view of one marker, quiet zone included, from the pinhole camera."""
    texture, quiet = _marker_texture(marker_id)
    side = texture.shape[0]
    half = MARKER_LENGTH * side / (side - 2 * quiet) / 2
    corners_3d = np.array(
        [[-half, half, 0.0], [half, half, 0.0], [half, -half, 0.0], [-half, -half, 0.0]]
    )
    corners_px, _ = cv2.projectPoints(corners_3d, rvec, tvec, CAMERA, None)
    # Map the texture's outer pixel edges, not its pixel centres, onto the projected corners.
    edges = np.array(
        [[-0.5, -0.5], [side - 0.5, -0.5], [side - 0.5, side - 0.5], [-0.5, side - 0.5]],
        dtype=np.float32,
    )
    warp = cv2.getPerspectiveTransform(edges, corners_px.reshape(4, 2).astype(np.float32))
    return cv2.warpPerspective(texture, warp, IMAGE_SIZE, borderValue=255)


def _facing_camera(turn_deg: float) -> np.ndarray:
    """Rodrigues vector of a marker that faces the camera, turned about the camera's vertical axis."""
    facing = cv2.Rodrigues(np.array([math.pi, 0.0, 0.0]))[0]  # marker z toward the camera
    turn = cv2.Rodrigues(np.array([0.0, math.radians(turn_deg), 0.0]))[0]
    return cv2.Rodrigues(turn @ facing)[0].reshape(3)


def test_unknown_dictionary_is_rejected():
    with pytest.raises(ValueError, match="unknown ArUco dictionary"):
        MarkerDetector("DICT_NOT_A_DICTIONARY")


def _detect_one(turn_deg: float, distance: float) -> Detection:
    """The single marker detected in a rendered view, with its pose."""
    image = _render(7, _facing_camera(turn_deg), np.array([0.0, 0.0, distance]))
    detections = MarkerDetector(DICTIONARY).detect(image, MARKER_LENGTH, CAMERA)
    assert [d.marker_id for d in detections] == [7]
    return detections[0]


NEAR_AND_FAR = [(0, 1.0), (40, 1.0), (0, 2.5), (60, 2.5), (0, 5.0)]


@pytest.mark.parametrize(("turn_deg", "distance"), NEAR_AND_FAR)
def test_distance_matches_the_rendered_marker(turn_deg: float, distance: float):
    detection = _detect_one(turn_deg, distance)

    # The distance scales with the marker's side in pixels, and the tolerance is one pixel of that
    # side. A wrong marker size, camera matrix or corner order misses by far more.
    side_px = FOCAL_PX * MARKER_LENGTH / distance
    assert detection.distance == pytest.approx(distance, rel=1 / side_px)


@pytest.mark.parametrize(("turn_deg", "distance"), NEAR_AND_FAR)
def test_marker_always_faces_the_camera(turn_deg: float, distance: float):
    # A printed marker is only seen from its printed side, at any distance.
    assert math.degrees(_detect_one(turn_deg, distance).viewing_angle) < 90


@pytest.mark.parametrize(("turn_deg", "distance"), [(0, 1.0), (40, 1.0), (60, 2.5)])
def test_viewing_angle_where_it_is_observable(turn_deg: float, distance: float):
    # A tilt shows only as perspective, which shrinks with the square of the distance. A far marker
    # seen nearly head-on carries too little of it, and the angle is checked only near or oblique.
    # A wrong axis convention misses by tens of degrees.
    viewing_angle = math.degrees(_detect_one(turn_deg, distance).viewing_angle)
    assert viewing_angle == pytest.approx(turn_deg, abs=5)


def test_without_calibration_markers_have_no_pose():
    image = _render(3, _facing_camera(0), np.array([0.0, 0.0, 1.0]))

    detections = MarkerDetector(DICTIONARY).detect(image)

    assert [d.marker_id for d in detections] == [3]
    assert not detections[0].has_pose
    with pytest.raises(ValueError, match="no pose"):
        _ = detections[0].distance


def test_drawing_leaves_the_input_untouched():
    gray = _render(5, _facing_camera(0), np.array([0.0, 0.0, 1.0]))
    image = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    before = image.copy()
    detections = MarkerDetector(DICTIONARY).detect(image, MARKER_LENGTH, CAMERA)

    drawn = draw_detections(image, detections, CAMERA, None, MARKER_LENGTH)

    assert drawn.shape == image.shape
    assert not np.array_equal(drawn, image)
    assert np.array_equal(image, before)


def test_quaternion_from_rvec():
    assert quaternion_from_rvec(np.zeros(3)) == (0.0, 0.0, 0.0, 1.0)
    x, y, z, w = quaternion_from_rvec(np.array([0.0, 0.0, math.pi / 2]))
    assert (x, y) == (0.0, 0.0)
    assert z == pytest.approx(math.sin(math.pi / 4))
    assert w == pytest.approx(math.cos(math.pi / 4))
