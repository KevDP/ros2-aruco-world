#!/usr/bin/env python3
"""Tables and figures from the localization experiment's results.

Reads frames.csv, views.csv, fixes.csv and settings.yaml from experiments/localization.py and
writes, next to them:
    summary.md          how far the fixes land from the truth, by corner window, distance and
                        viewing angle, how much of that the localizer's covariance covers, and
                        the frame fixes-top.png shows
    corner-windows.png  distance error and fixes far off against the distance, per corner window
    fix-error.png       the live window's fixes, their error along and across the line of sight
                        against the distance, with the spread their covariance gives them
    fixes-top.png       one frame's fixes from above, up close, with their covariance ellipses

A fix is within the localizer's reach when its measured distance is, as in the live node. Runs
without ROS, with NumPy, Matplotlib and PyYAML:
    python3 experiments/localization_report.py
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import numpy as np
import plot_style
import yaml
from matplotlib.lines import Line2D
from matplotlib.patches import Ellipse, Patch
from plot_style import plt
from world import RESULTS, localization_core, localizer_parameters, marker_map

DISTANCE_BINS = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 8.0]  # metres
ANGLE_BINS = [0.0, 15.0, 30.0, 45.0, 60.0, 90.0]  # degrees off head-on
FAR_M = (0.5, 1.0)  # position errors counted as far off, in metres
CHI2_2D_95 = 5.991  # squared Mahalanobis distance that holds 95% of a 2-D Gaussian
ELLIPSE_SIGMAS = 2.0  # as the localizer draws its ellipses in RViz
# fix-error.png's vertical range. A corner of each panel counts the fixes past it.
ERROR_LIMIT_CM = 100.0
FAR_MARKER_M = 3.0  # fixes-top.png shows a frame whose farthest marker is at least this far
TURNED_DEG = 30.0  # a marker orientation this far off is the other pose the square allows
COLOURS = (plot_style.AMBER, plot_style.VIOLET, plot_style.CYAN, plot_style.EMERALD)
FRAME_COLOURS = (plot_style.VIOLET, plot_style.AMBER, plot_style.EMERALD)  # cyan is the truth
# Behind each note in fix-error.png, to keep it readable over the dots.
TEXT_BOX = {"boxstyle": "square,pad=0.15", "fc": plot_style.BACKGROUND, "lw": 0, "alpha": 0.8}

core = localization_core()


def _read(path: Path) -> list[dict[str, str]]:
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def _bins(edges: list[float]) -> list[tuple[float, float]]:
    return list(zip(edges[:-1], edges[1:], strict=True))


def _span(lo: float, hi: float, unit: str) -> str:
    return f"{lo:g}-{hi:g} {unit}"


def _pct(values) -> str:
    return f"{100 * np.mean(values):.1f}%" if len(values) else "-"


# ---------------------------------------------------------------------------
# Loading
# ---------------------------------------------------------------------------


def load_fixes(results: Path, uncertainty, reach: float) -> list[dict]:
    """Every fix with its error against the truth, split along and across the line of sight."""
    truth = {r["frame"]: r for r in _read(results / "frames.csv")}
    fixes = []
    for r in _read(results / "fixes.csv"):
        t = truth[r["frame"]]
        ex, ey = float(r["x"]) - float(t["x"]), float(r["y"]) - float(t["y"])
        sight = float(r["sight"])
        heading = math.remainder(float(r["yaw"]) - float(t["yaw"]), 2 * math.pi)
        measured = float(r["measured_distance"])
        fix = core.Fix(
            float(r["x"]), float(r["y"]), float(r["yaw"]), measured, float(r["fix_sight"])
        )
        cov = core.covariance(fix, uncertainty)
        error = np.array([ex, ey])
        distance = float(r["distance"])
        fixes.append(
            {
                "frame": int(r["frame"]),
                "window": int(r["window"]),
                "marker": int(r["marker"]),
                "distance": distance,
                "angle": float(r["viewing_angle_deg"]),
                "measured": measured,
                "in_reach": measured <= reach,
                "error": math.hypot(ex, ey),
                "along": ex * math.cos(sight) + ey * math.sin(sight),  # + toward the marker
                "across": -ex * math.sin(sight) + ey * math.cos(sight),
                "heading": heading,
                "distance_error": (measured - distance) / distance,
                "rotation_error": float(r["rotation_error_deg"]),
                "mahalanobis2": float(error @ np.linalg.solve(cov[:2, :2], error)),
                "fix": fix,
                "cov": cov,
            }
        )
    return fixes


# ---------------------------------------------------------------------------
# Tables
# ---------------------------------------------------------------------------


def window_table(fixes: list[dict], views: list[dict], windows: list[int], reach: float) -> list:
    """One row per corner window: what it reads and how far its fixes land, within reach."""
    lines = [
        f"## Corner windows, within {reach:g} m",
        "",
        "Read: markers in view within the reach that the window reads. Distance error: median of",
        "measured minus true distance, from 3 to 4 m. Turned: fixes whose marker orientation came",
        f"out more than {TURNED_DEG:g} deg off. Far off: fixes more than {FAR_M[0]:g} and "
        f"{FAR_M[1]:g} m from the truth. Covered: fixes inside the covariance's 95% ellipse.",
        "",
        "| Window (px each side) | Read | Fixes | Distance error, 3-4 m | Rotation error, median "
        f"| Turned | Position error, median | 90th percentile | > {FAR_M[0]:g} m | > {FAR_M[1]:g} m "
        "| Covered |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    near = [v for v in views if v["distance"] < reach]
    for w in windows:
        sel = [f for f in fixes if f["window"] == w and f["in_reach"]]
        mid = [f["distance_error"] for f in sel if 3 <= f["distance"] < 4]
        errors = [f["error"] for f in sel]
        lines.append(
            f"| {w} | {_pct([v[f'read_w{w}'] for v in near])} | {len(sel)} "
            f"| {100 * np.median(mid):+.1f}% | {np.median([f['rotation_error'] for f in sel]):.1f} deg "
            f"| {sum(f['rotation_error'] > TURNED_DEG for f in sel)} "
            f"| {100 * np.median(errors):.1f} cm | {100 * np.percentile(errors, 90):.0f} cm "
            f"| {_pct([e > FAR_M[0] for e in errors])} | {_pct([e > FAR_M[1] for e in errors])} "
            f"| {_pct([f['mahalanobis2'] <= CHI2_2D_95 for f in sel])} |"
        )
    return lines


def reach_lines(fixes: list[dict], reach: float) -> list:
    """The live window's fixes within reach, all together, and the turned ones among them."""
    sel = [f for f in fixes if f["in_reach"]]
    turned = sorted(f["error"] for f in sel if f["rotation_error"] > TURNED_DEG)
    errors = [f["error"] for f in sel]
    shares = [f["error"] / f["distance"] for f in sel]
    lines = [
        f"## The live window's fixes within {reach:g} m",
        "",
        f"{len(sel)} fixes, {_pct([f['mahalanobis2'] <= CHI2_2D_95 for f in sel])} inside the "
        f"covariance's 95% ellipse, {_pct([e > FAR_M[1] for e in errors])} more than "
        f"{FAR_M[1]:g} m from the truth.",
        f"Position error, median {100 * np.median(errors):.1f} cm and 90th percentile "
        f"{100 * np.percentile(errors, 90):.0f} cm. As a share of the true distance, median "
        f"{100 * np.median(shares):.1f}% and 90th percentile {100 * np.percentile(shares, 90):.1f}%.",
    ]
    if turned:
        lines.append(
            f"Orientation more than {TURNED_DEG:g} deg off: {len(turned)} of them, landing "
            f"{turned[0]:.1f} to {turned[-1]:.1f} m from the truth."
        )
    return lines


def distance_table(fixes: list[dict], uncertainty) -> list:
    """The live window's error by distance, against the spread its covariance allows."""
    lines = [
        "## By distance, every fix of the live window",
        "",
        "Along: median error along the line of sight, as a share of the distance (negative is",
        "farther from the marker than the truth). Across and heading: 90th percentile of the",
        "absolute error. Covered: fixes inside the covariance's 95% ellipse, whose sigmas are",
        f"{100 * uncertainty.range_per_m:g}% of the distance along the line of sight and the",
        f"distance times {uncertainty.angle_per_m:g} rad per metre across it.",
        "",
        "| Distance | Fixes | Position error, median | 90th percentile | Along | Across, p90 "
        f"| Heading, p90 | Covered | > {FAR_M[1]:g} m |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for lo, hi in _bins(DISTANCE_BINS):
        sel = [f for f in fixes if lo <= f["distance"] < hi]
        if len(sel) < 5:
            continue
        errors = [f["error"] for f in sel]
        along = np.median([f["along"] / f["distance"] for f in sel])
        lines.append(
            f"| {_span(lo, hi, 'm')} | {len(sel)} | {100 * np.median(errors):.1f} cm "
            f"| {100 * np.percentile(errors, 90):.0f} cm | {100 * along:+.1f}% "
            f"| {100 * np.percentile([abs(f['across']) for f in sel], 90):.0f} cm "
            f"| {math.degrees(np.percentile([abs(f['heading']) for f in sel], 90)):.1f} deg "
            f"| {_pct([f['mahalanobis2'] <= CHI2_2D_95 for f in sel])} "
            f"| {_pct([e > FAR_M[1] for e in errors])} |"
        )
    return lines


def angle_table(fixes: list[dict], reach: float) -> list:
    """The live window's fixes within reach, by how far off head-on the marker was seen."""
    lines = [
        f"## By viewing angle, the live window's fixes within {reach:g} m",
        "",
        f"| Viewing angle | Fixes | Position error, median | 90th percentile | > {FAR_M[0]:g} m "
        f"| > {FAR_M[1]:g} m | Covered |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    for lo, hi in _bins(ANGLE_BINS):
        sel = [f for f in fixes if f["in_reach"] and lo <= f["angle"] < hi]
        if len(sel) < 5:
            continue
        errors = [f["error"] for f in sel]
        lines.append(
            f"| {_span(lo, hi, 'deg')} | {len(sel)} | {100 * np.median(errors):.1f} cm "
            f"| {100 * np.percentile(errors, 90):.0f} cm | {_pct([e > FAR_M[0] for e in errors])} "
            f"| {_pct([e > FAR_M[1] for e in errors])} "
            f"| {_pct([f['mahalanobis2'] <= CHI2_2D_95 for f in sel])} |"
        )
    return lines


def spread_lines(fixes: list[dict], uncertainty, reach: float) -> list:
    """Per-metre spreads that would put 95% of the fixes within reach inside two sigmas."""
    sel = [f for f in fixes if f["in_reach"]]
    range_fit = np.percentile([abs(f["along"]) / f["distance"] for f in sel], 95) / 2
    across_fit = np.percentile([abs(f["across"]) / f["distance"] ** 2 for f in sel], 95) / 2
    heading_fit = np.percentile([abs(f["heading"]) / f["distance"] for f in sel], 95) / 2
    return [
        f"## Spread per metre that covers 95% of the live window's fixes within {reach:g} m",
        "",
        "Two sigmas at the 95th percentile, each error divided by how it grows in the covariance.",
        "",
        "| | Configured | Would cover 95% |",
        "|---|---:|---:|",
        f"| range_sigma_per_m (along) | {uncertainty.range_per_m:g} | {range_fit:.3f} |",
        f"| angle_sigma_per_m, from the error across | {uncertainty.angle_per_m:g} "
        f"| {across_fit:.3f} |",
        f"| angle_sigma_per_m, from the heading | {uncertainty.angle_per_m:g} | {heading_fit:.3f} |",
    ]


def frame_lines(fixes: list[dict]) -> list:
    """The fixes of the frame fixes-top.png shows, nearest marker first."""
    frame = pick_frame(fixes)
    chosen = sorted(
        (f for f in fixes if f["frame"] == frame and f["in_reach"]), key=lambda f: f["distance"]
    )
    lines = [
        f"## The frame in fixes-top.png: {frame}",
        "",
        "Along is positive toward the marker, as in the table by distance.",
        "",
        "| Marker | True distance | Position error | Along | Across | Inside the 95% ellipse |",
        "|---:|---:|---:|---:|---:|---|",
    ]
    for f in chosen:
        inside = "yes" if f["mahalanobis2"] <= CHI2_2D_95 else "no"
        lines.append(
            f"| {f['marker']} | {f['distance']:.2f} m | {100 * f['error']:.1f} cm "
            f"| {100 * f['along']:+.1f} cm | {100 * f['across']:+.1f} cm | {inside} |"
        )
    return lines


# ---------------------------------------------------------------------------
# Figures
# ---------------------------------------------------------------------------


def plot_windows(fixes: list[dict], windows: list[int], live: int, path: Path) -> None:
    figure, (top, bottom) = plt.subplots(2, 1, figsize=(7.2, 6.4), sharex=True)
    spans = [(lo, hi) for lo, hi in _bins(DISTANCE_BINS) if lo >= 1.0]
    mids = [(lo + hi) / 2 for lo, hi in spans]
    for colour, w in zip(COLOURS, windows, strict=False):
        rows = [
            [f for f in fixes if f["window"] == w and lo <= f["distance"] < hi] for lo, hi in spans
        ]
        label = f"{w} px" + (" (live)" if w == live else "")
        top.plot(mids, [100 * np.median([f["distance_error"] for f in r]) for r in rows], "o-",
                 color=colour, lw=2, label=label)  # fmt: skip
        bottom.plot(mids, [100 * np.mean([f["error"] > FAR_M[0] for f in r]) for r in rows], "o-",
                    color=colour, lw=2, label=label)  # fmt: skip
    top.set_ylabel("distance error, median (%)")
    bottom.set_ylabel(f"fixes > {FAR_M[0]:g} m off (%)")
    bottom.set_xlabel("true distance (m)")
    for axis in (top, bottom):
        axis.axhline(0, color=plot_style.MUTED, lw=0.6)
        axis.grid(True)
    top.legend(title="corner window", loc="upper left")
    top.set_title("Each corner window, by distance")
    plot_style.save(figure, path)


def plot_errors(fixes: list[dict], uncertainty, reach: float, path: Path) -> None:
    figure, (top, bottom) = plt.subplots(2, 1, figsize=(7.2, 6.4), sharex=True)
    d = np.linspace(0.3, max(f["distance"] for f in fixes), 100)
    bands = (
        (
            top,
            "along",
            ELLIPSE_SIGMAS * uncertainty.range_per_m * d,
            "along the line of sight (cm)",
        ),
        (
            bottom,
            "across",
            ELLIPSE_SIGMAS * uncertainty.angle_per_m * d**2,
            "across the line of sight (cm)",
        ),
    )
    for axis, key, sigma, label in bands:
        values = np.array([100 * f[key] for f in fixes])
        distances = np.array([f["distance"] for f in fixes])
        inside = np.abs(values) <= ERROR_LIMIT_CM
        axis.scatter(distances[inside], values[inside], s=6, color=plot_style.CYAN, alpha=0.5, lw=0)
        axis.fill_between(d, -100 * sigma, 100 * sigma, color=plot_style.VIOLET, alpha=0.18, lw=0,
                          label=f"covariance, {ELLIPSE_SIGMAS:g}σ")  # fmt: skip
        reach_label = f"{reach:g} m limit" if axis is top else None
        axis.axvline(reach, color=plot_style.MUTED, lw=0.8, ls="--", label=reach_label)
        axis.axhline(0, color=plot_style.MUTED, lw=0.6)
        axis.set_ylim(-ERROR_LIMIT_CM, ERROR_LIMIT_CM)
        axis.set_ylabel(label)
        axis.grid(True)
        outside = int((~inside).sum())
        axis.text(0.99, 0.05, f"{outside} beyond ±{ERROR_LIMIT_CM:g} cm",
                  transform=axis.transAxes, ha="right", color=plot_style.MUTED,
                  bbox=TEXT_BOX)  # fmt: skip
    top.text(0.01, 0.05, "below 0: farther from the marker", transform=top.transAxes,
             color=plot_style.MUTED, bbox=TEXT_BOX)  # fmt: skip
    top.legend(loc="upper left")
    top.set_title("Error of each fix against Gazebo's truth")
    bottom.set_xlabel("distance to the marker (m)")
    plot_style.save(figure, path)


def pick_frame(fixes: list[dict]) -> int:
    """The frame shown from above: a near marker and a far one, each fix inside its ellipse.

    Among frames with at least two fixes within reach, the farthest marker at least FAR_MARKER_M
    away and every fix inside its own 95% ellipse, the one whose distances span the most.
    """
    by_frame: dict[int, list[dict]] = {}
    for f in fixes:
        if f["in_reach"]:
            by_frame.setdefault(f["frame"], []).append(f)
    best = None
    for frame, chosen in by_frame.items():
        distances = [f["distance"] for f in chosen]
        if len(chosen) < 2 or max(distances) < FAR_MARKER_M:
            continue
        if any(f["mahalanobis2"] > CHI2_2D_95 for f in chosen):
            continue
        if best is None or max(distances) / min(distances) > best[0]:
            best = (max(distances) / min(distances), frame)
    return best[1]


def draw_frame(axis, chosen: list[dict], truth: dict, view: float) -> list:
    """One frame's fixes around the true pose, each in its own colour, farthest marker first.

    Args:
        axis: Where to draw. Its limits are left to the caller.
        chosen: The frame's fixes.
        truth: The frame's row of frames.csv.
        view: A length in metres that sizes the arrows.

    Returns:
        Legend handles: one per fix, then the true pose and the line of sight.
    """
    tx, ty, tyaw = (float(truth[k]) for k in ("x", "y", "yaw"))
    arrow = 0.25 * view
    markers = marker_map()
    handles = []
    by_distance = sorted(chosen, key=lambda f: -f["distance"])
    for colour, f in zip(FRAME_COLOURS, by_distance, strict=False):
        fix = f["fix"]
        mx, my = markers[f["marker"]][:2]
        axis.plot([fix.x, mx], [fix.y, my], ls="--", lw=0.9, color=colour, alpha=0.7)
        axis.plot(mx, my, "s", ms=9, color=colour)
        variances, vectors = np.linalg.eigh(f["cov"][:2, :2])
        width, height = 2 * ELLIPSE_SIGMAS * np.sqrt(variances[::-1])
        angle = math.degrees(math.atan2(vectors[1, 1], vectors[0, 1]))
        axis.add_patch(Ellipse((fix.x, fix.y), width, height, angle=angle, color=colour,
                               alpha=0.22, lw=0))  # fmt: skip
        axis.arrow(fix.x, fix.y, arrow * math.cos(fix.yaw), arrow * math.sin(fix.yaw),
                   color=colour, width=0.012 * view, head_width=0.06 * view,
                   length_includes_head=True)  # fmt: skip
        label = f"marker {f['marker']}, {f['distance']:.1f} m"
        handles.append(Patch(color=colour, label=label))
    axis.arrow(tx, ty, arrow * math.cos(tyaw), arrow * math.sin(tyaw), color=plot_style.CYAN,
               width=0.016 * view, head_width=0.07 * view, length_includes_head=True)  # fmt: skip
    handles.append(Patch(color=plot_style.CYAN, label="true pose"))
    handles.append(Line2D([], [], ls="--", color=plot_style.MUTED, label="line of sight"))
    return handles


def plot_frame(fixes: list[dict], frames: dict, path: Path) -> None:
    """The frame pick_frame chooses, seen from above and up close."""
    frame = pick_frame(fixes)
    chosen = [f for f in fixes if f["frame"] == frame and f["in_reach"]]
    truth = frames[frame]
    tx, ty = float(truth["x"]), float(truth["y"])
    # Half the side of the square shown, in metres: room for every ellipse around its fix.
    view = 1.15 * max(
        math.hypot(f["fix"].x - tx, f["fix"].y - ty)
        + ELLIPSE_SIGMAS * math.sqrt(np.linalg.eigvalsh(f["cov"][:2, :2]).max())
        for f in chosen
    )
    view = max(view, 0.5)
    figure, axis = plt.subplots(figsize=(6.4, 6.4))
    handles = draw_frame(axis, chosen, truth, view)
    axis.set_xlim(tx - view, tx + view)
    axis.set_ylim(ty - view, ty + view)
    axis.set_aspect("equal")
    axis.grid(True)
    axis.set_xlabel("x (m)")
    axis.set_ylabel("y (m)")
    axis.set_title(f"Fixes from one image, with {ELLIPSE_SIGMAS:g}σ ellipses")
    axis.legend(handles=handles, loc="upper center", bbox_to_anchor=(0.5, -0.12), ncol=2)
    plot_style.save(figure, path)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--results", type=Path, default=RESULTS / "localization")
    args = parser.parse_args()

    settings = yaml.safe_load((args.results / "settings.yaml").read_text())
    params = localizer_parameters()
    reach = params["max_distance"]
    uncertainty = core.Uncertainty(params["range_sigma_per_m"], params["angle_sigma_per_m"])
    windows, live = settings["windows"], settings["live_window"]
    fixes = load_fixes(args.results, uncertainty, reach)
    views = [
        {
            "distance": float(v["distance"]),
            **{f"read_w{w}": v[f"read_w{w}"] == "1" for w in windows},
        }
        for v in _read(args.results / "views.csv")
    ]
    frames = {int(r["frame"]): r for r in _read(args.results / "frames.csv")}
    live_fixes = [f for f in fixes if f["window"] == live]

    lines = [
        "# Localization: where each marker's fix lands",
        "",
        f"{settings['frames']} frames, {len(views)} markers in view. The live detector refines",
        f"corners {live} px to each side, and the localizer's reach is {reach:g} m.",
        "",
        *window_table(fixes, views, windows, reach),
        "",
        *reach_lines(live_fixes, reach),
        "",
        *distance_table(live_fixes, uncertainty),
        "",
        *angle_table(live_fixes, reach),
        "",
        *spread_lines(live_fixes, uncertainty, reach),
        "",
        *frame_lines(live_fixes),
    ]
    unknown = settings.get("unknown_ids_read") or {}
    lines += ["", f"Ids read that the map does not have: {unknown or 'none'}."]
    (args.results / "summary.md").write_text("\n".join(lines) + "\n")

    plot_windows(fixes, windows, live, args.results / "corner-windows.png")
    plot_errors(live_fixes, uncertainty, reach, args.results / "fix-error.png")
    plot_frame(live_fixes, frames, args.results / "fixes-top.png")
    print(f"done: {args.results}")


if __name__ == "__main__":
    main()
