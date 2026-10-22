#!/usr/bin/env python3
"""Tables and figures from the turning experiment's results.

Reads trials_<label>.csv and trace_<label>.csv from experiments/turning.py (one pair per wheel
shape) and writes, next to them:
    summary.md   odometry against the truth, per shape, turn rate and speed
    heading.png  the odometry's heading error as the robot turns, per shape
    turning.gif  one turn trial per shape, played back: where the robot is and where it thinks it is

It also prints what the odometry would report if each wheel pushed on the floor from the inner
edge of its rim, from the URDF's wheel separation and width. Runs without
ROS, with NumPy, Matplotlib and PyYAML:
    python3 experiments/turning_report.py
"""

from __future__ import annotations

import argparse
import csv
import math
from collections import defaultdict
from pathlib import Path

import numpy as np
import plot_style
from matplotlib import patches
from matplotlib.animation import FuncAnimation, PillowWriter
from plot_style import plt
from world import RESULTS, robot_properties

COLOURS = {"sphere": plot_style.CYAN, "cylinder": plot_style.AMBER}
SHAPE_TITLES = {"sphere": "point contact (sphere)", "cylinder": "flat contact (cylinder)"}

# The trial the animation plays back, and how fast.
ANIMATED_RATE = 0.6  # rad/s, turning left, first repeat
PLAYBACK_SPEED = 2.5  # times real time
GIF_FPS = 12


def _read(path: Path) -> list[dict[str, str]]:
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def _ratio_cell(values: list[float]) -> str:
    v = np.asarray(values)
    return f"{v.mean():.3f} ± {v.std(ddof=1):.3f}" if len(v) > 1 else f"{v.mean():.3f}"


def _robot_outline(props: dict[str, float]) -> list[np.ndarray]:
    """The robot seen from above, x forward: body, both wheels and a nose that marks the front."""
    half_l, half_w = props["body_length"] / 2, props["body_width"] / 2
    r, w, y = props["wheel_radius"], props["wheel_width"], props["wheel_separation"] / 2

    def box(x0: float, x1: float, y0: float, y1: float) -> np.ndarray:
        return np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]])

    nose = np.array([[half_l, -0.05], [half_l + 0.07, 0.0], [half_l, 0.05]])
    return [
        box(-half_l, half_l, -half_w, half_w),
        box(-r, r, y - w / 2, y + w / 2),
        box(-r, r, -y - w / 2, -y + w / 2),
        nose,
    ]


def _turned(shape: np.ndarray, heading: float) -> np.ndarray:
    c, s = math.cos(heading), math.sin(heading)
    return shape @ np.array([[c, s], [-s, c]])


def _trial_index(rows: list[dict[str, str]], rate: float) -> str:
    """Trace number of the first left turn at ``rate``, counting trials in file order."""
    for number, r in enumerate(rows):
        if r["kind"] == "turn" and float(r["command"]) == rate and int(r["direction"]) == 1:
            return str(number)
    raise ValueError(f"no left turn at {rate} rad/s in the trials")


def _animate(labels: list[str], trials: dict, traces: dict, props: dict, out: Path) -> None:
    """Play back one turn per wheel shape: the robot where Gazebo has it, solid, and where its
    odometry puts it, dashed. Both turn about the same point and differ only in heading."""
    series = {}
    for label in labels:
        trial = _trial_index(trials[label], ANIMATED_RATE)
        by_source = defaultdict(list)
        for r in traces[label]:
            if r["trial"] == trial:
                by_source[r["source"]].append((float(r["t"]), float(r["turned"])))
        series[label] = {source: np.array(points) for source, points in by_source.items()}
    end = min(s["truth"][-1, 0] for s in series.values())
    times = np.arange(0.0, end, PLAYBACK_SPEED / GIF_FPS)

    outline = _robot_outline(props)
    reach = props["body_length"] / 2 + 0.15
    fig, axes = plt.subplots(1, len(labels), figsize=(3.6 * len(labels), 4.0))
    axes = np.atleast_1d(axes)
    drawn = {}
    for ax, label in zip(axes, labels, strict=True):
        ax.set_xlim(-reach, reach)
        ax.set_ylim(-reach, reach)
        ax.set_aspect("equal")
        ax.axis("off")
        ax.set_title(SHAPE_TITLES.get(label, label), color=plot_style.TEXT)
        colour = COLOURS.get(label, plot_style.VIOLET)
        truth = [
            ax.add_patch(patches.Polygon(p, fc=colour, ec=colour, alpha=0.55)) for p in outline
        ]
        belief = [
            ax.add_patch(patches.Polygon(p, fill=False, ec=plot_style.TEXT, ls="--", lw=1.4))
            for p in outline
        ]
        text = ax.text(0, -reach * 0.98, "", ha="center", va="bottom", fontsize=9,
                       color=plot_style.TEXT)  # fmt: skip
        drawn[label] = (truth, belief, text)
    fig.text(0.5, 0.02, "solid: where the robot is (Gazebo)    dashed: where it thinks it is",
             ha="center", fontsize=9, color=plot_style.MUTED)  # fmt: skip

    def frame(t: float) -> list:
        changed = []
        for label in labels:
            s = series[label]
            true_turn = float(np.interp(t, s["truth"][:, 0], s["truth"][:, 1]))
            believed = float(np.interp(t, s["odom"][:, 0], s["odom"][:, 1]))
            truth, belief, text = drawn[label]
            for patch, shape in zip(truth, outline, strict=True):
                patch.set_xy(_turned(shape, true_turn))
            for patch, shape in zip(belief, outline, strict=True):
                patch.set_xy(_turned(shape, believed))
            text.set_text(
                f"turned {math.degrees(true_turn):4.0f}°, thinks {math.degrees(believed):4.0f}°"
            )
            changed += [*truth, *belief, text]
        return changed

    animation = FuncAnimation(fig, frame, frames=times, blit=False)
    animation.save(out, writer=PillowWriter(fps=GIF_FPS), dpi=90,
                   savefig_kwargs={"facecolor": plot_style.BACKGROUND})  # fmt: skip
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--results", type=Path, default=RESULTS / "turning")
    args = parser.parse_args()

    labels = sorted(p.stem.removeprefix("trials_") for p in args.results.glob("trials_*.csv"))
    trials = {label: _read(args.results / f"trials_{label}.csv") for label in labels}
    traces = {label: _read(args.results / f"trace_{label}.csv") for label in labels}

    props = robot_properties()
    separation, width = props["wheel_separation"], props["wheel_width"]
    inner_edges = (separation - width) / separation

    # ---- summary.md
    turn_ratios: dict[tuple[str, float], list[float]] = defaultdict(list)
    distance_ratios: dict[tuple[str, float], list[float]] = defaultdict(list)
    for label, rows in trials.items():
        for r in rows:
            if r["kind"] == "turn":
                ratio = float(r["odom_turn"]) / float(r["true_turn"])
                turn_ratios[(label, float(r["command"]))].append(ratio)
            else:
                ratio = float(r["odom_distance"]) / float(r["true_distance"])
                distance_ratios[(label, float(r["command"]))].append(ratio)
    rates = sorted({rate for _, rate in turn_ratios})
    speeds = sorted({speed for _, speed in distance_ratios})
    repeats = {label: max(int(r["repeat"]) for r in rows) + 1 for label, rows in trials.items()}

    lines = [
        "# Turning: wheel odometry against the truth",
        "",
        "Odometry over truth for each trial, mean ± standard deviation over both directions and "
        f"every repeat ({', '.join(f'{label}: {n}' for label, n in repeats.items())} repeats). "
        "1.000 means the odometry agrees with the truth.",
        "",
        "## Turning in place, two turns",
        "",
        "| Wheel collision | " + " | ".join(f"{r:.1f} rad/s" for r in rates) + " |",
        "|---|" + "---:|" * len(rates),
    ]
    for label in labels:
        cells = [_ratio_cell(turn_ratios[(label, r)]) for r in rates]
        lines.append(f"| {label} | " + " | ".join(cells) + " |")
    lines += [
        "",
        f"If each wheel pushed from the inner edge of its rim, the odometry would report "
        f"({separation:.3f} - {width:.3f}) / {separation:.3f} = {inner_edges:.3f} of the true "
        "turn.",
        "",
        "## Driving straight, 2 m",
        "",
        "| Wheel collision | " + " | ".join(f"{s:.1f} m/s" for s in speeds) + " |",
        "|---|" + "---:|" * len(speeds),
    ]
    for label in labels:
        cells = [_ratio_cell(distance_ratios[(label, s)]) for s in speeds]
        lines.append(f"| {label} | " + " | ".join(cells) + " |")
    (args.results / "summary.md").write_text("\n".join(lines) + "\n")

    # ---- heading.png: the heading error as the turn builds up, every turn trial. The two sources
    # have their own stamps, and the odometry is interpolated to the truth's before subtracting.
    fig, ax = plt.subplots(figsize=(7.2, 4.4))
    for label in labels:
        series: dict[tuple[str, str], list[tuple[float, float]]] = defaultdict(list)
        for r in traces[label]:
            series[(r["trial"], r["source"])].append((float(r["t"]), float(r["turned"])))
        first = True
        for trial in sorted({trial for trial, _ in series}, key=int):
            odom, truth = np.array(series[(trial, "odom")]), np.array(series[(trial, "truth")])
            if len(odom) < 2 or len(truth) < 2 or abs(truth[-1, 1]) < math.pi:
                continue  # straight trials barely turn
            odom_at_truth = np.interp(truth[:, 0], odom[:, 0], odom[:, 1])
            sign = math.copysign(1.0, truth[-1, 1])
            ax.plot(np.degrees(sign * truth[:, 1]), np.degrees(sign * (odom_at_truth - truth[:, 1])),
                    color=COLOURS.get(label, plot_style.VIOLET), lw=1, alpha=0.6,
                    label=f"{label} wheels" if first else None)  # fmt: skip
            first = False
    # The prediction is drawn last and dashed, visible even over a measured line.
    longest = max(abs(float(r["true_turn"])) for rows in trials.values() for r in rows)
    turned = np.array([0.0, math.degrees(longest)])
    ax.plot(turned, (inner_edges - 1) * turned, "--", color=plot_style.TEXT, lw=1.2, zorder=3,
            label="predicted, inner rim edges")  # fmt: skip
    ax.axhline(0, color=plot_style.MUTED, lw=0.5)
    ax.set_xlabel("true turn (deg)")
    ax.set_ylabel("odometry minus truth (deg)")
    ax.set_xlim(0, 1.03 * turned[-1])
    ax.grid(True)
    ax.legend(loc="lower left")
    ax.set_title("Heading error of the wheel odometry while turning in place")
    plot_style.save(fig, args.results / "heading.png")

    _animate(labels, trials, traces, props, args.results / "turning.gif")

    print((args.results / "summary.md").read_text())


if __name__ == "__main__":
    main()
