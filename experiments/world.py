"""The world and the robot as the experiments see them, read from the files that define them.

Nothing here needs ROS. The reports that use it run with NumPy, Matplotlib and PyYAML alone.
"""

from __future__ import annotations

import importlib.util
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from types import ModuleType

import yaml

REPO = Path(__file__).resolve().parent.parent
WORLD_PACKAGE = REPO / "aruco_world"
ROBOT_XACRO = WORLD_PACKAGE / "urdf" / "camera_bot.urdf.xacro"
LOCALIZATION_CORE = REPO / "aruco_localization" / "aruco_localization" / "localization.py"
RESULTS = REPO / "experiments" / "results"


def _load(path: Path, name: str) -> ModuleType:
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module  # dataclasses look their module up while being built
    spec.loader.exec_module(module)
    return module


def load_world() -> ModuleType:
    """build_world.py as a module, for its marker placements and the room's layout."""
    return _load(WORLD_PACKAGE / "scripts" / "build_world.py", "build_world")


def localization_core() -> ModuleType:
    """The localizer's own code, loaded from the repository: it needs NumPy and no ROS."""
    return _load(LOCALIZATION_CORE, "localization_core")


def marker_settings() -> tuple[str, float]:
    """Dictionary name and marker size, from the detector parameters build_world.py writes."""
    path = WORLD_PACKAGE / "config" / "aruco_markers.yaml"
    params = yaml.safe_load(path.read_text())["aruco_detector"]["ros__parameters"]
    return params["dictionary"], float(params["marker_length"])


def marker_map() -> dict[int, list[float]]:
    """Each marker's x, y, z, qx, qy, qz, qw in the map, as build_world.py writes them."""
    path = WORLD_PACKAGE / "config" / "marker_map.yaml"
    params = yaml.safe_load(path.read_text())["aruco_localizer"]["ros__parameters"]
    return {i: [float(v) for v in params[f"marker_{i}"]] for i in params["marker_ids"]}


def localizer_parameters() -> dict[str, float]:
    """The localizer's distance limit and spread, from its configuration."""
    path = WORLD_PACKAGE / "config" / "aruco_localizer.yaml"
    params = yaml.safe_load(path.read_text())["aruco_localizer"]["ros__parameters"]
    return {
        name: float(params[name])
        for name in ("max_distance", "range_sigma_per_m", "angle_sigma_per_m")
    }


def localizer_reach() -> float:
    """Metres past which the localizer gives no fix, from its configuration."""
    return localizer_parameters()["max_distance"]


def robot_start() -> tuple[float, float, float]:
    """Where the launch spawns the robot: x and y in metres, heading in radians."""
    start = yaml.safe_load((WORLD_PACKAGE / "config" / "robot_start.yaml").read_text())
    return float(start["x"]), float(start["y"]), float(start["yaw"])


def robot_properties() -> dict[str, float]:
    """The numeric properties the URDF builds the robot with (sizes in metres)."""
    tag = "{http://www.ros.org/wiki/xacro}property"
    props = {}
    for element in ET.parse(ROBOT_XACRO).getroot().iter(tag):
        try:
            props[element.get("name")] = float(element.get("value"))
        except (TypeError, ValueError):
            continue  # skip expressions and blocks, keep plain numbers
    return props


@dataclass(frozen=True)
class Solid:
    """A block standing on the floor with its sides along x and y.

    Attributes:
        low: Corner with the smallest x, y and z, in metres.
        high: Corner with the largest x, y and z.
    """

    low: tuple[float, float, float]
    high: tuple[float, float, float]


def _solid(centre: tuple[float, float], size: tuple[float, float, float]) -> Solid:
    (x, y), (sx, sy, sz) = centre, size
    return Solid((x - sx / 2, y - sy / 2, 0.0), (x + sx / 2, y + sy / 2, sz))


def solids(world: ModuleType | None = None) -> dict[str, Solid]:
    """What the robot can run into: the four walls, the box and the post under each post marker.

    Keyed by the names build_world.py gives them: wall_front, box, post_0 and so on. The marker
    panels are left out. Those on posts hang above the robot, and the rest lie flat on a wall, the
    box or the floor.

    Args:
        world: build_world.py as load_world() returns it, to save loading it again.
    """
    world = world or load_world()
    found = {name: _solid(centre[:2], size) for name, centre, size in world.walls()}
    found["box"] = _solid(world.BOX_CENTRE, world.BOX_SIZE)
    for p in world.PLACEMENTS:
        if p.mount == "post":
            x, y, height = world.post_under(p)
            side = world.POST_SIDE_M
            found[f"post_{p.marker_id}"] = _solid((x, y), (side, side, height))
    return found
