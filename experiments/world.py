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
RESULTS = REPO / "experiments" / "results"


def load_world() -> ModuleType:
    """build_world.py as a module, for its marker placements and the room's layout."""
    path = WORLD_PACKAGE / "scripts" / "build_world.py"
    spec = importlib.util.spec_from_file_location("build_world", path)
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module  # its dataclasses look their module up while being built
    spec.loader.exec_module(module)
    return module


def marker_settings() -> tuple[str, float]:
    """Dictionary name and marker size, from the detector parameters build_world.py writes."""
    path = WORLD_PACKAGE / "config" / "aruco_markers.yaml"
    params = yaml.safe_load(path.read_text())["aruco_detector"]["ros__parameters"]
    return params["dictionary"], float(params["marker_length"])


def localizer_reach() -> float:
    """Metres past which the localizer gives no fix, from its configuration."""
    path = WORLD_PACKAGE / "config" / "aruco_localizer.yaml"
    params = yaml.safe_load(path.read_text())["aruco_localizer"]["ros__parameters"]
    return float(params["max_distance"])


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
