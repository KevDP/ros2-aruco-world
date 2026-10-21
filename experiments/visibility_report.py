#!/usr/bin/env python3
"""Tables and figures from the visibility experiment's results.

Reads poses.csv and detections.csv from experiments/visibility.py and writes, next to them:
    summary.md    how much of the room sees markers, and each marker's reach
    map.png       distinct markers the robot reads from each spot, turning in place
    envelope.png  every marker read, by its true distance and viewing angle

True distances and viewing angles come from the robot's true pose, the camera's place on the
robot and the marker map, not from the detector. Needs NumPy and Matplotlib only, no ROS:
    python3 experiments/visibility_report.py
"""

from __future__ import annotations

import argparse
import csv
import math
from collections import defaultdict
from pathlib import Path
from types import ModuleType

import numpy as np
import plot_style
from matplotlib import patches
from matplotlib.colors import BoundaryNorm, ListedColormap
from plot_style import plt
from world import RESULTS, load_world, robot_properties

MAX_POSE_ERROR_M = 0.02  # a pose that landed further than this from the command is reported
MAX_YAW_ERROR_DEG = 1.0
LABEL_BOX = {"boxstyle": "round,pad=0.15", "fc": plot_style.BACKGROUND, "ec": "none", "alpha": 0.8}
# Map colours from "nothing read" up; nothing read stays lighter than the background, where the
# robot cannot stand.
MAP_COLOURS = ["#334155", "#164e63", "#0e7490", "#06b6d4", "#67e8f9", "#cffafe"]
CASE_COLOURS = {
    "posts": plot_style.CYAN,
    "floor": plot_style.AMBER,
    "box": plot_style.VIOLET,
    "walls": plot_style.EMERALD,
    "walls, high and tilted down": "#f472b6",
}


def _read(path: Path) -> list[dict[str, str]]:
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def _true_view(
    robot: tuple[float, float, float], marker: np.ndarray, facing: np.ndarray, camera: dict
) -> tuple[float, float]:
    """True distance and viewing angle (degrees) of a marker from the robot's camera."""
    x, y, yaw = robot
    cam = np.array([x + camera["x"] * math.cos(yaw), y + camera["x"] * math.sin(yaw), camera["z"]])
    sight = cam - marker
    distance = float(np.linalg.norm(sight))
    angle = math.degrees(math.acos(max(-1.0, min(1.0, float(facing @ sight) / distance))))
    return distance, angle


def _draw_room(ax: plt.Axes, world: ModuleType) -> None:
    """Walls, the box, and each marker as a short bar with an arrow the way it faces."""
    half = world.ROOM_HALF
    ax.add_patch(
        patches.Rectangle((-half, -half), 2 * half, 2 * half, fill=False, lw=2,
                          ec=plot_style.MUTED)
    )  # fmt: skip
    bx, by = world.BOX_CENTRE
    sx, sy = world.BOX_SIZE[:2]
    ax.add_patch(
        patches.Rectangle((bx - sx / 2, by - sy / 2), sx, sy, fc=plot_style.PANEL,
                          ec=plot_style.MUTED)
    )  # fmt: skip
    for p in world.PLACEMENTS:
        x, y = p.position[:2]
        n = np.asarray(p.facing, dtype=float)
        if p.mount == "floor":
            ax.add_patch(patches.Rectangle((x - 0.1, y - 0.1), 0.2, 0.2, fc=plot_style.AMBER))
            ax.annotate(str(p.marker_id), (x, y), xytext=(8, -14), textcoords="offset points",
                        color=plot_style.AMBER, fontsize=9, bbox=LABEL_BOX)  # fmt: skip
            continue
        if p.mount == "post":
            ax.plot(x, y, "s", color=plot_style.MUTED, ms=4)
        flat = n[:2] / np.linalg.norm(n[:2])
        side = np.array([-flat[1], flat[0]]) * 0.15
        ax.plot([x - side[0], x + side[0]], [y - side[1], y + side[1]], lw=4,
                color=plot_style.AMBER, solid_capstyle="butt")  # fmt: skip
        ax.annotate("", xy=(x + 0.35 * flat[0], y + 0.35 * flat[1]), xytext=(x, y),
                    arrowprops={"arrowstyle": "->", "color": plot_style.AMBER, "lw": 1.2})  # fmt: skip
        label = (x + 0.6 * flat[0], y + 0.6 * flat[1])
        ax.text(*label, str(p.marker_id), color=plot_style.AMBER, fontsize=9, ha="center",
                va="center", bbox=LABEL_BOX)  # fmt: skip


def _case(placement: object) -> str:
    """The kind of case a marker stands for, to group the scatter plot."""
    if placement.mount == "wall" and abs(placement.facing[2]) > 0:
        return "walls, high and tilted down"
    return {"post": "posts", "floor": "floor", "box": "box", "wall": "walls"}[placement.mount]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--results", type=Path, default=RESULTS / "visibility")
    args = parser.parse_args()

    world = load_world()
    props = robot_properties()
    camera = {"x": props["camera_x"], "z": props["camera_z"]}
    placements = {p.marker_id: p for p in world.PLACEMENTS}
    poses = _read(args.results / "poses.csv")
    detections = _read(args.results / "detections.csv")

    # Did every pose land where it was asked to?
    position_error = max(
        math.hypot(float(p["x"]) - float(p["x_cmd"]), float(p["y"]) - float(p["y_cmd"]))
        for p in poses
    )
    yaw_error = max(
        abs(math.degrees(math.remainder(float(p["yaw"]) - float(p["yaw_cmd"]), 2 * math.pi)))
        for p in poses
    )
    misplaced = position_error > MAX_POSE_ERROR_M or yaw_error > MAX_YAW_ERROR_DEG

    # Per grid point: which markers it reads over all headings, and how many headings read any.
    by_pose = {p["pose"]: p for p in poses}
    spot_of = {p["pose"]: (float(p["x_cmd"]), float(p["y_cmd"])) for p in poses}
    headings_at: dict[tuple, int] = defaultdict(int)
    seeing_at: dict[tuple, int] = defaultdict(int)
    for p in poses:
        headings_at[spot_of[p["pose"]]] += 1
        seeing_at[spot_of[p["pose"]]] += int(p["markers"]) > 0
    ids_at: dict[tuple, set[int]] = defaultdict(set)
    reads: dict[int, list[tuple[float, float, float]]] = defaultdict(list)
    for d in detections:
        marker_id = int(d["marker"])
        if marker_id not in placements:
            continue  # a misread id; reported below
        pose = by_pose[d["pose"]]
        ids_at[spot_of[d["pose"]]].add(marker_id)
        placement = placements[marker_id]
        facing = np.asarray(placement.facing, dtype=float)
        distance, angle = _true_view(
            (float(pose["x"]), float(pose["y"]), float(pose["yaw"])),
            np.asarray(placement.position, dtype=float),
            facing / np.linalg.norm(facing),
            camera,
        )
        reads[marker_id].append((distance, angle, float(d["distance"])))
    unknown = sum(int(d["marker"]) not in placements for d in detections)

    spots = sorted(headings_at)
    distinct = np.array([len(ids_at[s]) for s in spots])
    any_heading = np.array([seeing_at[s] / headings_at[s] for s in spots])

    # ---- summary.md
    lines = [
        "# Visibility: which markers the robot reads from each spot",
        "",
        f"{len(spots)} spots on a grid over the free floor, {len(poses) // len(spots)} headings "
        f"each: {len(poses)} poses. Largest gap between the pose asked for and the pose Gazebo "
        f"reported: {100 * position_error:.1f} cm and {yaw_error:.2f} deg"
        + (" (above the limit: check the run)." if misplaced else "."),
        "",
        "## How much of the room sees markers",
        "",
        "| Distinct markers read from the spot, turning in place | Spots | Share |",
        "|---|---:|---:|",
    ]
    top = max(4, int(distinct.max()))
    for n in range(top + 1):
        count = int((distinct == n).sum()) if n < top else int((distinct >= n).sum())
        label = str(n) if n < top else f"{n} or more"
        lines.append(f"| {label} | {count} | {100 * count / len(spots):.1f}% |")
    seen = sum(int(p["markers"]) > 0 for p in poses)
    lines += [
        "",
        f"Poses (spot and heading) with at least one marker read: {seen} of {len(poses)} "
        f"({100 * seen / len(poses):.1f}%). Median share of headings that read a marker, per "
        f"spot: {100 * float(np.median(any_heading)):.0f}%.",
        "",
        "## Each marker's reach",
        "",
        "True distance and viewing angle (0 is head-on) at which it was read; the last column "
        "compares the detector's distance with the true one.",
        "",
        "| Id | Placement | Read from (share of spots) | True distance, min / median / max (m) "
        "| Viewing angle, median / max (deg) | Detector distance error, median |",
        "|---|---|---:|---|---|---:|",
    ]
    for marker_id in sorted(placements):
        spots_reading = sum(marker_id in ids_at[s] for s in spots)
        r = np.array(reads.get(marker_id, []))
        if len(r) == 0:
            lines.append(f"| {marker_id} | {placements[marker_id].note} | 0% | - | - | - |")
            continue
        error = 100 * float(np.median((r[:, 2] - r[:, 0]) / r[:, 0]))
        lines.append(
            f"| {marker_id} | {placements[marker_id].note} "
            f"| {100 * spots_reading / len(spots):.0f}% "
            f"| {r[:, 0].min():.1f} / {np.median(r[:, 0]):.1f} / {r[:, 0].max():.1f} "
            f"| {np.median(r[:, 1]):.0f} / {r[:, 1].max():.0f} | {error:+.1f}% |"
        )
    if unknown:
        lines += ["", f"Detections with an id that is not in the world: {unknown}."]
    (args.results / "summary.md").write_text("\n".join(lines) + "\n")

    # ---- map.png
    step = min(abs(a[1] - b[1]) for a, b in zip(spots, spots[1:], strict=False) if a[0] == b[0])
    levels = min(len(MAP_COLOURS), top + 1)
    cmap = ListedColormap(MAP_COLOURS[:levels])
    norm = BoundaryNorm(np.arange(-0.5, levels), levels)
    fig, ax = plt.subplots(figsize=(7.2, 6.4))
    for (x, y), n in zip(spots, distinct, strict=True):
        ax.add_patch(
            patches.Rectangle((x - step / 2, y - step / 2), step, step,
                              fc=cmap(norm(min(n, levels - 1))), ec=plot_style.BACKGROUND, lw=0.5)
        )  # fmt: skip
    _draw_room(ax, world)
    sx, sy, _, syaw = world.ROBOT_START
    ax.annotate("", xy=(sx + 0.4 * math.cos(syaw), sy + 0.4 * math.sin(syaw)), xytext=(sx, sy),
                arrowprops={"arrowstyle": "-|>", "color": plot_style.EMERALD, "lw": 2})  # fmt: skip
    half = world.ROOM_HALF
    ax.set_xlim(-half - 0.3, half + 0.3)
    ax.set_ylim(-half - 0.3, half + 0.3)
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("Distinct markers read from each spot, turning in place")
    bar = fig.colorbar(plt.cm.ScalarMappable(norm=norm, cmap=cmap), ax=ax, ticks=range(levels),
                       fraction=0.046, pad=0.04)  # fmt: skip
    bar.ax.set_yticklabels([str(n) for n in range(levels - 1)] + [f"{levels - 1}+"])
    plot_style.save(fig, args.results / "map.png")

    # ---- envelope.png
    fig, ax = plt.subplots(figsize=(7.2, 4.4))
    by_case: dict[str, list[int]] = defaultdict(list)
    for marker_id in sorted(placements):
        by_case[_case(placements[marker_id])].append(marker_id)
    for case, ids in by_case.items():
        r = np.array([read for marker_id in ids for read in reads.get(marker_id, [])])
        if len(r) == 0:
            continue
        ax.scatter(r[:, 0], r[:, 1], s=8, alpha=0.7, color=CASE_COLOURS[case],
                   label=f"{case} ({', '.join(map(str, ids))})")  # fmt: skip
    ax.set_xlabel("true distance (m)")
    ax.set_ylabel("viewing angle (deg, 0 = head-on)")
    ax.set_ylim(0, 90)
    ax.grid(True)
    ax.set_title("Every marker read, by distance and viewing angle")
    ax.legend(fontsize=8, loc="upper right", markerscale=2)
    plot_style.save(fig, args.results / "envelope.png")

    print((args.results / "summary.md").read_text())


if __name__ == "__main__":
    main()
