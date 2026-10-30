#!/usr/bin/env python3
"""Tables and figures from the fusion experiment's runs.

Reads runs.csv and runs/run_*.csv from experiments/fusion.py and writes, next to them:
    summary.md         how far each estimate is from the truth along the route and at its end, the
                       gyro's bias in each run, and the fixes: how many, how late and how far off
    error-in-time.png  the position and heading error of the wheels and the filters the robot
                       uses (SHOWN) against time, median and band over the runs, with when
                       markers were in view
    gyro-bias.png      the local filter's heading error against time, with the gyro as it comes
                       and with its bias taken out, next to the wheels alone
    route-top.png      one run from above: the true path, the wheel odometry's and the global
                       filter's, and the markers

Runs without ROS, with NumPy, Matplotlib and PyYAML:
    python3 experiments/fusion_report.py
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import numpy as np
import plot_style
from plot_style import plt
from world import RESULTS, marker_map

SOURCES = (
    "wheels", "local", "global", "local_raw", "global_raw",
    "global_gate5", "global_x4", "global_x16", "global_2hz",
)  # fmt: skip
LABELS = {
    "wheels": "wheels",
    "local": "wheels + gyro",
    "global": "wheels + gyro + markers",
    "local_raw": "wheels + uncalibrated gyro",
    "global_raw": "wheels + uncalibrated gyro + markers",
    "global_gate5": "wheels + gyro + markers, 5σ gate",
    "global_x4": "the same, fix covariance x4",
    "global_x16": "the same, fix covariance x16",
    "global_2hz": "the same, 2 fixes a second",
}
COLOURS = {
    "wheels": plot_style.AMBER,
    "local": plot_style.VIOLET,
    "global": plot_style.EMERALD,
    "local_raw": plot_style.CYAN,
    "global_raw": plot_style.MUTED,
    "global_gate5": plot_style.MUTED,
    "global_x4": plot_style.MUTED,
    "global_x16": plot_style.MUTED,
    "global_2hz": plot_style.MUTED,
}
SHOWN = ("wheels", "local", "global")  # what error-in-time.png and route-top.png show
STEP_S = 0.5  # the time grid every run is put on
BAND = (10, 90)  # percentiles of the runs that bound the band
IN_VIEW_S = 1.0  # a fix stamped this recently means a marker is in view
FAR_M = 1.0  # a fix farther than this from the truth is far off
OFF_M = 1.0  # an estimate farther than this from the truth has gone off
FIRST_CORNER = (-2.5, -2.5)  # where fusion.py's laps start and end (its CORNERS[0])
AT_CORNER_M = 0.2  # the robot passes a corner this close


def _read(path: Path) -> list[dict[str, str]]:
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def _wrap(angle: np.ndarray) -> np.ndarray:
    return np.arctan2(np.sin(angle), np.cos(angle))


def load_run(path: Path) -> dict[str, dict[str, np.ndarray]]:
    """One run's rows by source, as arrays, with each row's error against the truth."""
    rows: dict[str, list[dict[str, str]]] = {}
    for r in _read(path):
        rows.setdefault(r["source"], []).append(r)
    run = {}
    for source, found in rows.items():
        column = {k: np.array([float(r[k]) for r in found]) for k in
                  ("t", "x", "y", "yaw", "truth_x", "truth_y", "truth_yaw")}  # fmt: skip
        column["error"] = np.hypot(column["x"] - column["truth_x"], column["y"] - column["truth_y"])
        column["heading"] = _wrap(column["yaw"] - column["truth_yaw"])
        if source == "fix":
            column["arrival"] = np.array([float(r["arrival"]) for r in found])
            column["marker"] = np.array([int(r["marker"]) if r["marker"] else -1 for r in found])
            for key in ("measured_distance", "viewing_angle_deg"):
                column[key] = np.array([float(r[key]) if r[key] else np.nan for r in found])
        run[source] = column
    return run


def on_grid(runs: list[dict], grid: np.ndarray, key: str) -> dict[str, np.ndarray]:
    """Each source's error on the time grid, one row per run."""
    return {
        s: np.array([np.interp(grid, run[s]["t"], np.abs(run[s][key])) for run in runs])
        for s in SOURCES
    }


def lap_ends(run: dict) -> list[float]:
    """When the robot reaches the first corner after each lap, from the true path."""
    truth = run["global"]
    distance = np.hypot(truth["truth_x"] - FIRST_CORNER[0], truth["truth_y"] - FIRST_CORNER[1])
    near = distance < AT_CORNER_M
    # Each pass by the corner is a run of samples near it; the first pass starts the laps.
    starts = np.flatnonzero(near & ~np.concatenate(([False], near[:-1])))
    return [float(truth["t"][i]) for i in starts[1:]]


def in_view(runs: list[dict], grid: np.ndarray) -> np.ndarray:
    """Share of the runs with a fix stamped in the last IN_VIEW_S at each time on the grid."""
    seen = np.zeros((len(runs), grid.size), dtype=bool)
    for i, run in enumerate(runs):
        stamps = np.sort(run["fix"]["t"]) if "fix" in run else np.empty(0)
        last = np.searchsorted(stamps, grid, side="right") - 1
        recent = last >= 0
        seen[i, recent] = grid[recent] - stamps[last[recent]] <= IN_VIEW_S
    return seen.mean(axis=0)


# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------


def _cm(values) -> str:
    v = 100 * np.asarray(values)
    return f"{np.median(v):.1f} cm ({np.percentile(v, 90):.0f}, {np.max(v):.0f})"


def _deg(values) -> str:
    v = np.degrees(values)
    return f"{np.median(v):.1f} deg ({np.percentile(v, 90):.1f}, {np.max(v):.1f})"


def summary_lines(meta: list[dict], runs: list[dict], grid, position, heading, seen) -> list:
    drive = np.array([float(m["drive_s"]) for m in meta])
    path = np.array([float(m["path_m"]) for m in meta])
    bias = np.array([float(m["gyro_bias"]) for m in meta])
    lines = [
        "# Fusion: how far each estimate is from the truth",
        "",
        f"{len(runs)} runs of the same route, steered by the true pose: {np.median(path):.1f} m "
        f"in {np.median(drive):.0f} s (median). Each run starts a fresh simulation with its own "
        "drift seed, and Gazebo draws a new gyro bias.",
        "",
        "Sources: wheels is /wheel/odometry, local is /odometry/local (wheels and gyro), global "
        "is /odometry/global (the same plus the marker fixes). Values are the median over the "
        "runs, with the 90th percentile and the maximum in brackets.",
        "",
        "## Along the route and at its end",
        "",
        "| Source | Position error, mean over the route | Largest position error | "
        "Position error at the end | Heading error at the end |",
        "|---|---:|---:|---:|---:|",
    ]
    for s in SOURCES:
        end_pos = [run[s]["error"][-1] for run in runs]
        end_head = [abs(run[s]["heading"][-1]) for run in runs]
        lines.append(
            f"| {s} | {_cm(position[s].mean(axis=1))} | {_cm(position[s].max(axis=1))} "
            f"| {_cm(end_pos)} | {_deg(end_head)} |"
        )
    laps = [lap_ends(run) for run in runs]
    count = min(len(found) for found in laps)
    if count:
        lines += [
            "",
            "Position error as each lap ends, back at the first corner (median over the runs):",
            "",
            "| Source | " + " | ".join(f"Lap {i + 1}" for i in range(count)) + " |",
            "|---|" + "---:|" * count,
        ]
        for s in SOURCES:
            cells = []
            for i in range(count):
                errors = [np.interp(found[i], run[s]["t"], run[s]["error"])
                          for run, found in zip(runs, laps, strict=True)]  # fmt: skip
                cells.append(f"{100 * np.median(errors):.1f} cm")
            lines.append(f"| {s} | " + " | ".join(cells) + " |")
    lines += [
        "",
        "Before driving, the robot stands still at its start pose while the first fixes arrive. "
        "Error as the drive starts, median (largest):",
        "",
        "| Source | Position error | Heading error |",
        "|---|---:|---:|",
    ]
    for s in SOURCES:
        pos = [run[s]["error"][0] for run in runs]
        head = np.degrees([abs(run[s]["heading"][0]) for run in runs])
        lines.append(
            f"| {s} | {100 * np.median(pos):.1f} cm ({100 * np.max(pos):.1f}) "
            f"| {np.median(head):.2f} deg ({np.max(head):.2f}) |"
        )
    lines += [
        "",
        f"Markers in view (a fix stamped in the last {IN_VIEW_S:g} s): "
        f"{100 * seen.mean():.0f}% of the route time, averaged over the runs.",
        "",
        "## Gyro bias",
        "",
        "The gyro's mean turn rate in the second and a half at rest before each drive.",
        "",
        f"Bias: median {np.degrees(np.median(np.abs(bias))) * 60:.1f} deg/min, largest "
        f"{np.degrees(np.max(np.abs(bias))) * 60:.1f} deg/min. Over a median drive, the largest "
        f"would turn the heading by {np.degrees(np.max(np.abs(bias)) * np.median(drive)):.1f} deg.",
    ]
    fixes = [run["fix"] for run in runs if "fix" in run]
    if fixes:
        late = np.concatenate([f["arrival"] - f["t"] for f in fixes])
        errors = np.concatenate([f["error"] for f in fixes])
        far = errors > FAR_M
        lines += [
            "",
            "## Fixes",
            "",
            f"{errors.size} fixes in all, {np.median([f['t'].size for f in fixes]):.0f} per run "
            f"(median). Late by (arrival minus image stamp): median {1000 * np.median(late):.0f} "
            f"ms, 95th percentile {1000 * np.percentile(late, 95):.0f} ms, largest "
            f"{1000 * np.max(late):.0f} ms.",
            f"Position error of the fixes: median {100 * np.median(errors):.1f} cm, 90th "
            f"percentile {100 * np.percentile(errors, 90):.0f} cm; {far.sum()} "
            f"({100 * far.mean():.0f}%) more than {FAR_M:g} m off, in "
            f"{sum(int((f['error'] > FAR_M).any()) for f in fixes)} runs.",
        ]
        lines += ["", *marker_lines(fixes)]
        for source in [s for s in SOURCES if s.startswith("global")]:
            jumps = far_fix_effect(runs, source)
            if jumps:
                lines.append(
                    f"After each far-off fix, {source} changed its error by at most "
                    f"{100 * max(jumps):.1f} cm (error after minus error before)."
                )
    lines += ["", *off_lines(meta, runs)]
    return lines


def marker_lines(fixes: list[dict]) -> list[str]:
    """The fixes by the marker they came from, every run together."""
    marker = np.concatenate([f["marker"] for f in fixes])
    error = np.concatenate([f["error"] for f in fixes])
    distance = np.concatenate([f["measured_distance"] for f in fixes])
    angle = np.concatenate([f["viewing_angle_deg"] for f in fixes])
    far = error > FAR_M
    lines = [
        "By marker (measured distance and viewing angle are medians; the last column is the "
        "marker's share of every far-off fix):",
        "",
        f"| Marker | Fixes | Distance | Viewing angle | Position error, median | > {FAR_M:g} m "
        "| Of the far-off |",
        "|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for m in sorted(set(marker.tolist())):
        sel = marker == m
        name = str(m) if m >= 0 else "unmatched"
        lines.append(
            f"| {name} | {sel.sum()} | {np.nanmedian(distance[sel]):.1f} m "
            f"| {np.nanmedian(angle[sel]):.0f} deg | {100 * np.median(error[sel]):.1f} cm "
            f"| {100 * np.mean(far[sel]):.1f}% | {100 * far[sel].sum() / max(far.sum(), 1):.0f}% |"
        )
    return lines


def off_stretches(t: np.ndarray, error: np.ndarray) -> list[tuple[float, float]]:
    """Start and end of each stretch an estimate spends more than OFF_M from the truth."""
    off = np.concatenate(([False], error > OFF_M, [False]))
    edges = np.flatnonzero(np.diff(off.astype(int)))
    return [(float(t[a]), float(t[b - 1])) for a, b in zip(edges[::2], edges[1::2], strict=True)]


def off_lines(meta: list[dict], runs: list[dict]) -> list[str]:
    """Which runs each estimate went off in, for how long, and how far."""
    lines = [
        "## Runs that went off",
        "",
        f"Runs where the estimate passed {OFF_M:g} m from the truth, the longest stretch it "
        "stayed past that, and its largest error in those runs.",
        "",
        "| Source | Runs | Which | Longest stretch | Largest error |",
        "|---|---:|---|---|---:|",
    ]
    for s in SOURCES:
        found = []
        for m, run in zip(meta, runs, strict=True):
            for start, end in off_stretches(run[s]["t"], run[s]["error"]):
                found.append((int(m["run"]), start, end, float(run[s]["error"].max())))
        if not found:
            lines.append(f"| {s} | 0 | | | |")
            continue
        which = sorted({f[0] for f in found})
        run, start, end, _ = max(found, key=lambda f: f[2] - f[1])
        lines.append(
            f"| {s} | {len(which)} | {', '.join(map(str, which))} | {end - start:.1f} s "
            f"(run {run}, {start:.1f} to {end:.1f} s) | {max(f[3] for f in found):.2f} m |"
        )
    return lines


def far_fix_effect(runs: list[dict], source: str) -> list[float]:
    """Change in a global filter's error across each far-off fix's arrival, one per fix."""
    changes = []
    for run in runs:
        if "fix" not in run:
            continue
        g = run[source]
        for t, arrival, error in zip(run["fix"]["t"], run["fix"]["arrival"], run["fix"]["error"],
                                     strict=True):  # fmt: skip
            if error <= FAR_M:
                continue
            before = np.interp(t, g["t"], g["error"])
            after = np.interp(arrival + 2 * STEP_S, g["t"], g["error"])
            changes.append(float(after - before))
    return changes


# ---------------------------------------------------------------------------
# Figures
# ---------------------------------------------------------------------------


def plot_in_time(grid, position, heading, seen, path: Path) -> None:
    figure, (top, bottom) = plt.subplots(2, 1, figsize=(7.2, 6.0), sharex=True)
    for axis, values, scale, label in (
        (top, position, 100.0, "position error (cm)"),
        (bottom, heading, math.degrees(1.0), "heading error (deg)"),
    ):
        highest = 0.0
        for s in SHOWN:
            v = scale * values[s]
            median = np.median(v, axis=0)
            axis.fill_between(grid, *np.percentile(v, BAND, axis=0), color=COLOURS[s], alpha=0.15,
                              lw=0)  # fmt: skip
            axis.plot(grid, median, color=COLOURS[s], lw=2, label=LABELS[s])
            highest = max(highest, float(np.percentile(v, BAND[1], axis=0).max()))
        axis.fill_between(grid, 0, 1, where=seen >= 0.5, color=plot_style.MUTED, alpha=0.12,
                          lw=0, transform=axis.get_xaxis_transform(),
                          label="markers in view")  # fmt: skip
        axis.set_ylim(0, 1.05 * highest)
        axis.set_ylabel(label)
        axis.grid(True)
    top.legend(loc="upper left")
    top.set_title("Error along the route")
    bottom.set_xlabel("time (s)")
    plot_style.save(figure, path)


def plot_gyro(grid, heading, path: Path) -> None:
    figure, axis = plt.subplots(figsize=(7.2, 3.6))
    for s in ("wheels", "local_raw", "local"):
        v = np.degrees(heading[s])
        axis.fill_between(grid, *np.percentile(v, BAND, axis=0), color=COLOURS[s], alpha=0.15, lw=0)
        label = "wheels + calibrated gyro" if s == "local" else LABELS[s]
        axis.plot(grid, np.median(v, axis=0), color=COLOURS[s], lw=2, label=label)
    axis.set_ylim(0, None)
    axis.set_xlabel("time (s)")
    axis.set_ylabel("heading error (deg)")
    axis.set_title("Heading without markers")
    axis.grid(True)
    axis.legend(loc="upper left")
    plot_style.save(figure, path)


def plot_route(run: dict, path: Path) -> None:
    figure, axis = plt.subplots(figsize=(6.4, 6.4))
    truth = run[SHOWN[-1]]
    axis.plot(truth["truth_x"], truth["truth_y"], color=plot_style.CYAN, lw=2.5, label="true path")
    for s in (SHOWN[0], SHOWN[-1]):
        axis.plot(run[s]["x"], run[s]["y"], color=COLOURS[s], lw=1.5, label=LABELS[s])
    markers = marker_map()
    xs = [m[0] for m in markers.values()]
    ys = [m[1] for m in markers.values()]
    axis.plot(xs, ys, "s", color=plot_style.MUTED, ms=6, label="marker")
    axis.set_xlim(-4, 4)
    axis.set_ylim(-4, 4)
    axis.set_aspect("equal")
    axis.grid(True)
    axis.set_xlabel("x (m)")
    axis.set_ylabel("y (m)")
    axis.set_title("One run from above")
    axis.legend(loc="upper center", bbox_to_anchor=(0.5, -0.12), ncol=2)
    plot_style.save(figure, path)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--results", type=Path, default=RESULTS / "fusion")
    args = parser.parse_args()

    meta = sorted(_read(args.results / "runs.csv"), key=lambda m: int(m["run"]))
    runs = [load_run(args.results / "runs" / f"run_{int(m['run']):02d}.csv") for m in meta]
    end = min(min(run[s]["t"][-1] for s in SOURCES) for run in runs)
    grid = np.arange(0.0, end, STEP_S)
    position = on_grid(runs, grid, "error")
    heading = on_grid(runs, grid, "heading")
    seen = in_view(runs, grid)

    lines = summary_lines(meta, runs, grid, position, heading, seen)
    (args.results / "summary.md").write_text("\n".join(lines) + "\n")
    plot_in_time(grid, position, heading, seen, args.results / "error-in-time.png")
    plot_gyro(grid, heading, args.results / "gyro-bias.png")
    # The run shown from above: the one whose wheel odometry ends at the median error.
    ends = [run["wheels"]["error"][-1] for run in runs]
    shown = int(np.argsort(ends)[len(ends) // 2])
    plot_route(runs[shown], args.results / "route-top.png")
    print(f"done: {args.results} (route-top shows run {meta[shown]['run']})")


if __name__ == "__main__":
    main()
