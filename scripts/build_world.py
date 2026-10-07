#!/usr/bin/env python3
"""Generate the ArUco marker models and the room world from one description.

Each marker is described by where it is and which way it faces; the script computes the rotation,
writes one Gazebo model per marker (a thin panel with the marker texture) and the world file that
places them, together with the room, a few props and the robot.

Run it from the package root whenever a marker or the room changes:
    python3 scripts/build_world.py
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

# ---------------------------------------------------------------------------
# Markers
# ---------------------------------------------------------------------------

ROOT = Path(__file__).resolve().parent.parent
MODELS = ROOT / "models"
WORLD = ROOT / "worlds" / "aruco_room.sdf"

DICTIONARY = cv2.aruco.DICT_4X4_50
MARKER_SIDE_M = 0.15  # black square
BORDER_M = 0.025  # white quiet zone on each side, which detection needs
PANEL_THICKNESS_M = 0.002
TEXTURE_MARKER_PX = 600
TEXTURE_BORDER_PX = round(TEXTURE_MARKER_PX * BORDER_M / MARKER_SIDE_M)

# Room: 8 x 8 m, walls 2.5 m high; the inner wall faces sit at +-ROOM_HALF.
ROOM_HALF = 4.0
WALL_HEIGHT = 2.5
WALL_THICKNESS = 0.1

ROBOT_START = (-1.5, 0.0, 0.0, 0.0)  # x, y, z, yaw

# A plain box; marker 4 sits on its -X face, the one that looks back into the room.
BOX_CENTRE = (2.0, 2.4)
BOX_SIZE = (0.6, 0.6, 0.8)


@dataclass(frozen=True)
class Placement:
    """One marker in the world.

    Attributes:
        marker_id: ArUco id in DICT_4X4_50.
        position: Panel centre in metres.
        facing: Direction the printed side looks at; normalised by the script.
        mount: "post", "wall", "floor" or "box"; posts get a stand drawn under them.
        note: What this placement is meant to show.
    """

    marker_id: int
    position: tuple[float, float, float]
    facing: tuple[float, float, float]
    mount: str
    note: str


def _toward(angle_deg: float) -> tuple[float, float, float]:
    rad = math.radians(angle_deg)
    return (math.cos(rad), math.sin(rad), 0.0)


def _tilted(angle_deg: float, down_deg: float) -> tuple[float, float, float]:
    rad, down = math.radians(angle_deg), math.radians(down_deg)
    return (math.cos(rad) * math.cos(down), math.sin(rad) * math.cos(down), -math.sin(down))


def _turned_from_robot(x: float, y: float, turn_deg: float) -> tuple[float, float, float]:
    """Facing the robot's starting position, then turned by ``turn_deg`` around the vertical."""
    to_robot = math.degrees(math.atan2(ROBOT_START[1] - y, ROBOT_START[0] - x))
    return _toward(to_robot + turn_deg)


WALL = ROOM_HALF - PANEL_THICKNESS_M / 2
BOX_FACE = BOX_CENTRE[0] - BOX_SIZE[0] / 2 - PANEL_THICKNESS_M / 2

PLACEMENTS = [
    Placement(0, (0.5, 0.0, 0.6), _turned_from_robot(0.5, 0.0, 0), "post", "facing the robot"),
    Placement(1, (1.5, 1.3, 0.7), _turned_from_robot(1.5, 1.3, 30), "post", "turned 30 degrees"),
    Placement(2, (1.8, -1.2, 0.5), _turned_from_robot(1.8, -1.2, -60), "post", "turned 60 degrees"),
    Placement(3, (0.2, -0.5, 0.001), (0, 0, 1), "floor", "on the floor"),
    Placement(4, (BOX_FACE, BOX_CENTRE[1], 0.45), _toward(180), "box", "on the side of a box"),
    Placement(5, (WALL, 0.9, 1.0), _toward(180), "wall", "far wall"),
    Placement(6, (WALL, -1.4, 1.9), _tilted(180, 20), "wall", "far wall, high, tilted down"),
    Placement(7, (1.0, WALL, 1.2), _toward(-90), "wall", "left wall"),
    Placement(8, (-0.5, -WALL, 2.2), _tilted(90, 25), "wall", "right wall, high, tilted down"),
    Placement(9, (-WALL, 0.5, 1.0), _toward(0), "wall", "back wall, behind the robot"),
]


# ---------------------------------------------------------------------------
# Geometry
# ---------------------------------------------------------------------------


def _unit(v: np.ndarray) -> np.ndarray:
    return v / np.linalg.norm(v)


def rotation_for(facing: tuple[float, float, float]) -> np.ndarray:
    """Rotation that turns the panel's +Z (printed side) toward ``facing``.

    The marker's own +Y is kept as close to world up as possible, so wall markers stand upright;
    a marker that faces straight up or down uses world +X instead.
    """
    n = _unit(np.asarray(facing, dtype=float))
    up = np.array([0.0, 0.0, 1.0]) if abs(n[2]) < 0.99 else np.array([1.0, 0.0, 0.0])
    y_axis = _unit(up - up.dot(n) * n)
    x_axis = np.cross(y_axis, n)
    return np.column_stack([x_axis, y_axis, n])


def rpy_from(rotation: np.ndarray) -> tuple[float, float, float]:
    """Roll, pitch and yaw for SDF, which composes them as Rz(yaw) * Ry(pitch) * Rx(roll)."""
    pitch = math.asin(max(-1.0, min(1.0, -rotation[2, 0])))
    roll = math.atan2(rotation[2, 1], rotation[2, 2])
    yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    return roll, pitch, yaw


def _pose(x: float, y: float, z: float, roll: float = 0, pitch: float = 0, yaw: float = 0) -> str:
    return f"{x:.4f} {y:.4f} {z:.4f} {roll:.6f} {pitch:.6f} {yaw:.6f}"


# ---------------------------------------------------------------------------
# Marker models
# ---------------------------------------------------------------------------


def marker_texture(marker_id: int) -> np.ndarray:
    """The marker with its white quiet zone, as a grayscale image."""
    dictionary = cv2.aruco.getPredefinedDictionary(DICTIONARY)
    if hasattr(cv2.aruco, "generateImageMarker"):  # OpenCV 4.7 and later
        marker = cv2.aruco.generateImageMarker(dictionary, marker_id, TEXTURE_MARKER_PX)
    else:  # OpenCV 4.6, the version in Ubuntu 24.04
        marker = cv2.aruco.drawMarker(dictionary, marker_id, TEXTURE_MARKER_PX)
    pad = TEXTURE_BORDER_PX
    return cv2.copyMakeBorder(marker, pad, pad, pad, pad, cv2.BORDER_CONSTANT, value=255)


def write_marker_model(marker_id: int) -> None:
    name = f"aruco_{marker_id}"
    folder = MODELS / name
    (folder / "materials" / "textures").mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(folder / "materials" / "textures" / f"{name}.png"), marker_texture(marker_id))

    panel = MARKER_SIDE_M + 2 * BORDER_M
    texture_uri = f"model://{name}/materials/textures/{name}.png"
    (folder / "model.config").write_text(
        f"""<?xml version="1.0"?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>ArUco DICT_4X4_50 id {marker_id}, {MARKER_SIDE_M * 100:.0f} cm square on a white panel. Generated by scripts/build_world.py.</description>
</model>
"""
    )
    (folder / "model.sdf").write_text(
        f"""<?xml version="1.0"?>
<!-- Generated by scripts/build_world.py: edit the script, not this file. -->
<sdf version="1.9">
  <model name="{name}">
    <static>true</static>
    <link name="panel">
      <visual name="marker">
        <geometry><box><size>{panel} {panel} {PANEL_THICKNESS_M}</size></box></geometry>
        <material>
          <ambient>1 1 1 1</ambient>
          <diffuse>1 1 1 1</diffuse>
          <pbr><metal>
            <albedo_map>{texture_uri}</albedo_map>
            <roughness>1.0</roughness>
            <metalness>0.0</metalness>
          </metal></pbr>
        </material>
      </visual>
      <collision name="panel">
        <geometry><box><size>{panel} {panel} {PANEL_THICKNESS_M}</size></box></geometry>
      </collision>
    </link>
  </model>
</sdf>
"""
    )


# ---------------------------------------------------------------------------
# World
# ---------------------------------------------------------------------------


def _box_link(name: str, pose: str, size: tuple[float, float, float], rgb: str) -> str:
    sx, sy, sz = size
    return f"""      <link name="{name}">
        <pose>{pose}</pose>
        <visual name="v"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          <material><ambient>{rgb} 1</ambient><diffuse>{rgb} 1</diffuse></material></visual>
        <collision name="c"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry></collision>
      </link>
"""


def _room() -> str:
    half, h, t = ROOM_HALF, WALL_HEIGHT, WALL_THICKNESS
    wall = "0.82 0.82 0.80"
    links = [
        _box_link("wall_front", _pose(half + t / 2, 0, h / 2), (t, 2 * half + 2 * t, h), wall),
        _box_link("wall_back", _pose(-half - t / 2, 0, h / 2), (t, 2 * half + 2 * t, h), wall),
        _box_link("wall_left", _pose(0, half + t / 2, h / 2), (2 * half, t, h), wall),
        _box_link("wall_right", _pose(0, -half - t / 2, h / 2), (2 * half, t, h), wall),
    ]
    return '    <model name="room">\n      <static>true</static>\n' + "".join(links) + "    </model>\n"


def _props() -> str:
    box = _box_link("box", _pose(*BOX_CENTRE, BOX_SIZE[2] / 2), BOX_SIZE, "0.4 0.42 0.45")
    links = [box]
    for p in PLACEMENTS:
        if p.mount != "post":
            continue
        n = _unit(np.asarray(p.facing, dtype=float))
        x, y = p.position[0] - 0.03 * n[0], p.position[1] - 0.03 * n[1]
        height = p.position[2] - MARKER_SIDE_M / 2
        links.append(
            _box_link(f"post_{p.marker_id}", _pose(x, y, height / 2), (0.04, 0.04, height), "0.25 0.25 0.25")
        )
    return '    <model name="props">\n      <static>true</static>\n' + "".join(links) + "    </model>\n"


def _marker_includes() -> str:
    parts = []
    for p in PLACEMENTS:
        roll, pitch, yaw = rpy_from(rotation_for(p.facing))
        parts.append(
            f"""    <!-- {p.note} -->
    <include>
      <uri>model://aruco_{p.marker_id}</uri>
      <name>aruco_{p.marker_id}</name>
      <pose>{_pose(*p.position, roll, pitch, yaw)}</pose>
    </include>
"""
        )
    return "".join(parts)


def write_world() -> None:
    x, y, z, yaw = ROBOT_START
    world = f"""<?xml version="1.0"?>
<!-- Generated by scripts/build_world.py: edit the script, not this file. -->
<sdf version="1.9">
  <world name="aruco_room">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <scene>
      <ambient>0.55 0.55 0.55 1</ambient>
      <background>0.75 0.8 0.85 1</background>
      <shadows>true</shadows>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.4 0.3 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="c"><geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry></collision>
        <visual name="v"><geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry>
          <material><ambient>0.55 0.55 0.52 1</ambient><diffuse>0.55 0.55 0.52 1</diffuse></material></visual>
      </link>
    </model>

{_room()}
{_props()}
    <include>
      <uri>model://camera_bot</uri>
      <name>camera_bot</name>
      <pose>{_pose(x, y, z, 0, 0, yaw)}</pose>
    </include>

{_marker_includes()}  </world>
</sdf>
"""
    WORLD.parent.mkdir(parents=True, exist_ok=True)
    WORLD.write_text(world)


CAMERA_HEIGHT_M = 0.25  # must match models/camera_bot/model.sdf


def view_from_start(p: Placement) -> tuple[float, float, float]:
    """Distance, bearing and obliqueness of a marker seen from the robot's starting camera.

    Returns:
        (distance in metres, bearing in degrees with + to the left, angle in degrees between the
        line of sight and the marker's facing direction; 0 is head-on, 90 is edge-on).
    """
    camera = np.array([ROBOT_START[0], ROBOT_START[1], CAMERA_HEIGHT_M])
    sight = camera - np.asarray(p.position, dtype=float)
    distance = float(np.linalg.norm(sight))
    bearing = math.degrees(math.atan2(-sight[1], -sight[0])) - math.degrees(ROBOT_START[3])
    facing = _unit(np.asarray(p.facing, dtype=float))
    oblique = math.degrees(math.acos(max(-1.0, min(1.0, float(facing.dot(sight / distance))))))
    return distance, bearing, oblique


def main() -> None:
    for placement in PLACEMENTS:
        write_marker_model(placement.marker_id)
    write_world()
    print(f"{len(PLACEMENTS)} marker models in {MODELS}")
    print(f"world: {WORLD}")
    print("seen from the starting pose:")
    for p in PLACEMENTS:
        distance, bearing, oblique = view_from_start(p)
        print(
            f"  id {p.marker_id}: {distance:4.1f} m, bearing {bearing:+5.0f} deg, "
            f"oblique {oblique:3.0f} deg  ({p.note})"
        )


if __name__ == "__main__":
    main()
