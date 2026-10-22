#!/usr/bin/env python3
"""Tables and figures from the perception experiment's results.

Reads markers.csv and anomalies.csv from experiments/perception.py and writes, next to them:
    summary.md    how often each reading reads a marker, by cell size, distance and angle, and how
                  far off the measured distance and viewing angle are
    reading.png   share of markers read against the width of a cell in pixels, for each reading
    accuracy.png  error of the measured distance and viewing angle against the true distance

A cell is one square of the marker's grid: its 4 x 4 bits plus the black border make 6 across.
Runs without ROS, with NumPy, Matplotlib and PyYAML:
    python3 experiments/perception_report.py
"""

from __future__ import annotations

import argparse
import csv
import re
from collections import Counter
from pathlib import Path

import numpy as np
import plot_style
from plot_style import plt
from world import RESULTS, localizer_reach, marker_settings

# Views further than this from head-on squash the cells along one axis and are reported apart.
FRONTAL_DEG = 45.0
CELL_BINS = [1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 5.0, 7.0, np.inf]  # pixels per cell
DISTANCE_BINS = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 8.0]  # metres
ANGLE_BINS = [0.0, 45.0, 60.0, 75.0]  # degrees, for markers within the localizer's reach
READINGS = {"read_whole_cells": "whole cells", "read_cell_centres": "cell centres"}
COLOURS = {"read_whole_cells": plot_style.AMBER, "read_cell_centres": plot_style.CYAN}


def _read(path: Path) -> list[dict[str, str]]:
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def _cells_across(dictionary: str) -> int:
    """Cells along a marker's side: its data bits plus one border cell on each side."""
    bits = re.match(r"DICT_(\d+)X\1_", dictionary)
    if bits is None:
        raise ValueError(f"cannot tell the grid size of {dictionary}")
    return int(bits.group(1)) + 2


def _rate(rows: list[dict], key: str) -> str:
    return f"{100 * np.mean([r[key] for r in rows]):.0f}%" if rows else "-"


def _bins(edges: list[float]) -> list[tuple[float, float]]:
    return list(zip(edges[:-1], edges[1:], strict=True))


def _span(lo: float, hi: float, unit: str) -> str:
    return f"{lo:g}+ {unit}" if np.isinf(hi) else f"{lo:g}-{hi:g} {unit}"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--results", type=Path, default=RESULTS / "perception")
    args = parser.parse_args()

    cells = _cells_across(marker_settings()[0])
    reach_m = localizer_reach()
    rows = []
    for r in _read(args.results / "markers.csv"):
        row = {
            "distance": float(r["distance"]),
            "angle": float(r["viewing_angle_deg"]),
            "cell_px": float(r["side_px"]) / cells,
            "read_whole_cells": r["read_whole_cells"] == "1",
            "read_cell_centres": r["read_cell_centres"] == "1",
        }
        if r["measured_distance"]:
            row["distance_error"] = (
                100 * (float(r["measured_distance"]) - row["distance"]) / row["distance"]
            )
            row["angle_error"] = float(r["measured_angle_deg"]) - row["angle"]
        rows.append(row)
    frontal = [r for r in rows if r["angle"] < FRONTAL_DEG]
    frames = len(_read(args.results / "frames.csv"))
    anomalies = Counter((a["reading"], a["kind"]) for a in _read(args.results / "anomalies.csv"))

    # ---- summary.md
    head = "| " + " | ".join(READINGS.values()) + " |"
    lines = [
        "# Perception: how often the markers are read, and how well",
        "",
        f"{len(rows)} markers in view over {frames} frames, unhidden and with their white border",
        f"inside the frame. Rates below count markers within {FRONTAL_DEG:.0f} degrees of head-on",
        "unless a column says otherwise.",
        "",
        "## Read rate by cell width",
        "",
        "| Pixels per cell | Markers | Median distance (m) " + head,
        "|---|---:|---:|" + "---:|" * len(READINGS),
    ]
    for lo, hi in _bins(CELL_BINS):
        sel = [r for r in frontal if lo <= r["cell_px"] < hi]
        if sel:
            median = np.median([r["distance"] for r in sel])
            rates = " | ".join(_rate(sel, k) for k in READINGS)
            lines.append(f"| {_span(lo, hi, 'px')} | {len(sel)} | {median:.1f} | {rates} |")
    lines += [
        "",
        "## Read rate by distance",
        "",
        "| Distance | Markers " + head,
        "|---|---:|" + "---:|" * len(READINGS),
    ]
    for lo, hi in _bins(DISTANCE_BINS):
        sel = [r for r in frontal if lo <= r["distance"] < hi]
        if sel:
            lines.append(
                f"| {_span(lo, hi, 'm')} | {len(sel)} | "
                + " | ".join(_rate(sel, k) for k in READINGS)
                + " |"
            )
    lines += [
        "",
        f"## Read rate within {reach_m:g} m, by viewing angle",
        "",
        "| Viewing angle | Markers " + head,
        "|---|---:|" + "---:|" * len(READINGS),
    ]
    for lo, hi in _bins(ANGLE_BINS):
        sel = [r for r in rows if r["distance"] < reach_m and lo <= r["angle"] < hi]
        if sel:
            lines.append(
                f"| {_span(lo, hi, 'deg')} | {len(sel)} | "
                + " | ".join(_rate(sel, k) for k in READINGS)
                + " |"
            )
    lines += [
        "",
        "## Error of the measured pose",
        "",
        "Measured minus true, for every marker read: the median and the 10th to 90th percentile.",
        "",
        "| Distance | Read | Distance error | Viewing angle error |",
        "|---|---:|---:|---:|",
    ]
    measured = [r for r in frontal if "distance_error" in r]
    for lo, hi in _bins(DISTANCE_BINS):
        sel = [r for r in measured if lo <= r["distance"] < hi]
        if len(sel) < 5:
            continue
        d = np.percentile([r["distance_error"] for r in sel], [50, 10, 90])
        a = np.percentile([r["angle_error"] for r in sel], [50, 10, 90])
        lines.append(
            f"| {_span(lo, hi, 'm')} | {len(sel)} | {d[0]:+.1f}% ({d[1]:+.1f} to {d[2]:+.1f}) "
            f"| {a[0]:+.1f} deg ({a[1]:+.1f} to {a[2]:+.1f}) |"
        )
    lines += ["", "## Detections with the wrong id, or where no marker is", ""]
    for label in READINGS.values():
        wrong, none = anomalies[(label, "wrong id")], anomalies[(label, "no marker")]
        lines.append(f"- {label}: {wrong} with the wrong id, {none} where no marker is")
    (args.results / "summary.md").write_text("\n".join(lines) + "\n")

    # ---- reading.png
    fig, ax = plt.subplots(figsize=(7.2, 4.2))
    points = []
    for lo, hi in _bins(CELL_BINS[:-1]):
        sel = [r for r in frontal if lo <= r["cell_px"] < hi]
        if len(sel) >= 10:
            points.append(((lo + hi) / 2, sel))
    for key, label in READINGS.items():
        ax.plot([c for c, _ in points], [100 * np.mean([r[key] for r in sel]) for _, sel in points],
                "o-", color=COLOURS[key], lw=2, label=label)  # fmt: skip
    ax.set_xlabel("width of a cell in the image (pixels)")
    ax.set_ylabel("markers read (%)")
    ax.set_ylim(0, 102)
    ax.grid(True)
    ax.legend(loc="lower right")
    ax.set_title(f"Markers read, within {FRONTAL_DEG:.0f} degrees of head-on")
    plot_style.save(fig, args.results / "reading.png")

    # ---- accuracy.png
    fig, (top, bottom) = plt.subplots(2, 1, figsize=(7.2, 6.4), sharex=True)
    spans = [
        (lo, hi, [r for r in measured if lo <= r["distance"] < hi])
        for lo, hi in _bins(DISTANCE_BINS)
    ]
    spans = [(lo, hi, sel) for lo, hi, sel in spans if len(sel) >= 5]
    mids = [(lo + hi) / 2 for lo, hi, _ in spans]
    for axis, key, unit in (
        (top, "distance_error", "distance error (%)"),
        (bottom, "angle_error", "viewing angle error (deg)"),
    ):
        p = np.array([np.percentile([r[key] for r in sel], [10, 50, 90]) for _, _, sel in spans])
        axis.fill_between(
            mids,
            p[:, 0],
            p[:, 2],
            color=plot_style.CYAN,
            alpha=0.25,
            lw=0,
            label="10th to 90th percentile",
        )
        axis.plot(mids, p[:, 1], "o-", color=plot_style.CYAN, lw=2, label="median")
        axis.axhline(0, color=plot_style.MUTED, lw=0.6)
        reach = f"localizer's reach, {reach_m:g} m" if axis is top else None
        axis.axvline(reach_m, color=plot_style.MUTED, lw=0.8, ls="--", label=reach)
        axis.set_ylabel(unit)
        axis.grid(True)
    top.legend(loc="upper left")
    top.set_title("Measured minus true, by true distance")
    bottom.set_xlabel("true distance (m)")
    plot_style.save(fig, args.results / "accuracy.png")
    print(f"done: {args.results}")


if __name__ == "__main__":
    main()
