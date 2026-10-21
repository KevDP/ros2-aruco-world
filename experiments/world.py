"""The world and the robot as the experiments see them, read from the files that define them.

Nothing here needs ROS, so the reports run on any machine with NumPy and Matplotlib.
"""

from __future__ import annotations

import importlib.util
import sys
import xml.etree.ElementTree as ET
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


def robot_properties() -> dict[str, float]:
    """The numeric properties the URDF builds the robot with (sizes in metres)."""
    tag = "{http://www.ros.org/wiki/xacro}property"
    props = {}
    for element in ET.parse(ROBOT_XACRO).getroot().iter(tag):
        try:
            props[element.get("name")] = float(element.get("value"))
        except (TypeError, ValueError):
            continue  # expressions and blocks, not plain numbers
    return props
