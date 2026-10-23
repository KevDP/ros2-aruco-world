#!/usr/bin/env python3
"""Tables and figures from the stopping experiment's results.

Reads settings.yaml, trials.csv, trace.csv and guard.csv from experiments/stopping.py and writes,
next to them:
    summary.md    how much room the guard left in every approach, and how far the robot went
                  once a box appeared in its way, by speed
    approach.png  the robot's true speed against the room ahead, driving head-on at a wall
    stopping-distance.png
                  how far the robot went between a box appearing in the stop zone and coming to
                  rest, against its speed, with what the base's braking limit allows

Room is measured on the true geometry: the shortest distance between the robot's outline, as the
guard has it, and anything solid in the world. Runs without ROS, with NumPy, Matplotlib and PyYAML:
    python3 experiments/stopping_report.py
"""

from __future__ import annotations

import argparse
import csv
import math
from collections import defaultdict
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import plot_style
import yaml
from plot_style import plt
from world import RESULTS, robot_properties, solids

TOUCH_M = 0.005  # room this small or less counts as touching
SPEED_WINDOW_S = 0.1  # the speed at an instant is measured over this much time before it
REST_SPEED = 0.003  # m/s, below which the robot is at rest
SHOWN_APPROACH = ("wall", "0")  # head-on, where the room left is the room ahead
NAN = math.nan


@dataclass
class Trial:
    """One trial as recorded."""

    number: int
    kind: str
    target: str
    setting: str
    speed: float
    ended: str
    obstacle: tuple[float, float] | None  # centre of the box put in the way, in appear trials
    placed_at: float  # simulated time the box was in place, nan in approach trials
    removed_at: float  # simulated time the box was taken away again, nan otherwise
    t: np.ndarray = field(default_factory=lambda: np.empty(0))
    pose: np.ndarray = field(default_factory=lambda: np.empty((0, 3)))  # x, y, yaw per row of t
    guard: np.ndarray = field(default_factory=lambda: np.empty((0, 2)))  # t, linear speed sent
    room: np.ndarray = field(default_factory=lambda: np.empty(0))  # true room per row of t

    @property
    def end_at(self) -> float:
        """When the trial's outcome is read: as the box is taken away, or at the end."""
        return float(self.t[-1]) if math.isnan(self.removed_at) else self.removed_at


@dataclass
class Outcome:
    """What the report works out from one trial. Times are simulated seconds, lengths metres."""

    trial: Trial
    room_end: float  # room once the trial was over, the command still held
    room_min: float  # least room at any moment
    cut_at: float  # when the guard first sent less than the command
    stop_at: float  # when the guard first sent zero
    room_at_cut: float
    room_at_stop: float
    speed_at_stop: float
    after_stop: float  # distance covered from the stop to the end of the trial
    moved_again: float  # distance covered after first coming to rest, while the command was held
    room_placed: float  # room when the box appeared, in appear trials
    after_placed: float  # distance covered from the box appearing to the end of the trial
    reaction: float  # from the box appearing to the guard's first cut
    released_after: float  # from the box being taken away to the guard letting the robot go

    @property
    def touched(self) -> bool:
        return self.room_min <= TOUCH_M


# ---------------------------------------------------------------------------
# Loading
# ---------------------------------------------------------------------------


def _read(path: Path) -> list[dict[str, str]]:
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def load(folder: Path) -> tuple[dict, list[Trial]]:
    settings = yaml.safe_load((folder / "settings.yaml").read_text())
    trials = {}
    for row in _read(folder / "trials.csv"):
        centre = (float(row["obstacle_x"]), float(row["obstacle_y"]))
        trials[int(row["trial"])] = Trial(
            number=int(row["trial"]),
            kind=row["kind"],
            target=row["target"],
            setting=row["setting"],
            speed=float(row["speed"]),
            ended=row["ended"],
            obstacle=None if math.isnan(centre[0]) else centre,
            placed_at=float(row["obstacle_at"]),
            removed_at=float(row.get("removed_at", "nan")),
        )
    rows = defaultdict(list)
    for row in _read(folder / "trace.csv"):
        rows[int(row["trial"])].append([float(row[k]) for k in ("t", "x", "y", "yaw")])
    for number, values in rows.items():
        values = np.array(values)
        trials[number].t, trials[number].pose = values[:, 0], values[:, 1:]
    rows = defaultdict(list)
    for row in _read(folder / "guard.csv"):
        rows[int(row["trial"])].append([float(row["t"]), float(row["linear"])])
    for number, values in rows.items():
        trials[number].guard = np.array(values)
    return settings, [trials[n] for n in sorted(trials) if trials[n].t.size]


# ---------------------------------------------------------------------------
# Geometry
# ---------------------------------------------------------------------------


def _to_rectangle(points: np.ndarray, low: np.ndarray, high: np.ndarray) -> np.ndarray:
    """Distance from each point to an axis-aligned rectangle, zero inside it."""
    gap = np.maximum(np.maximum(low - points, points - high), 0.0)
    return np.hypot(gap[..., 0], gap[..., 1])


def room_left(pose: np.ndarray, outline: dict, obstacles: list[tuple]) -> np.ndarray:
    """Shortest distance from the robot's outline to any obstacle, for each pose.

    Both are rectangles, and two convex shapes that do not overlap are closest at a corner of one
    of them. The distance is the smallest from a corner of either to the other rectangle.

    Args:
        pose: (n, 3) x, y and yaw in the world.
        outline: front, rear and half_width of the robot, as the guard has them.
        obstacles: (low, high) corners of each obstacle's footprint, in the world.

    Returns:
        (n,) metres, zero where they touch.
    """
    front, rear, half = outline["front"], outline["rear"], outline["half_width"]
    corners = np.array([[front, half], [front, -half], [-rear, -half], [-rear, half]])
    cos, sin = np.cos(pose[:, 2])[:, None], np.sin(pose[:, 2])[:, None]
    world_x = pose[:, :1] + cos * corners[:, 0] - sin * corners[:, 1]
    world_y = pose[:, 1:2] + sin * corners[:, 0] + cos * corners[:, 1]
    robot = np.stack((world_x, world_y), axis=-1)  # (n, 4, 2)
    body_low, body_high = np.array([-rear, -half]), np.array([front, half])
    best = np.full(len(pose), np.inf)
    for low, high in obstacles:
        best = np.minimum(best, _to_rectangle(robot, low, high).min(axis=1))
        square = np.array([low, [high[0], low[1]], high, [low[0], high[1]]])  # (4, 2)
        dx = square[None, :, 0] - pose[:, :1]
        dy = square[None, :, 1] - pose[:, 1:2]
        local = np.stack((cos * dx + sin * dy, -sin * dx + cos * dy), axis=-1)  # (n, 4, 2)
        best = np.minimum(best, _to_rectangle(local, body_low, body_high).min(axis=1))
    return best


# ---------------------------------------------------------------------------
# What happened in a trial
# ---------------------------------------------------------------------------


def speed_before(trial: Trial, when: float) -> float:
    """True speed over the SPEED_WINDOW_S before ``when``, before anything sent then acts."""
    if math.isnan(when) or when - SPEED_WINDOW_S < trial.t[0]:
        return NAN
    step = _pose_at(trial, when) - _pose_at(trial, when - SPEED_WINDOW_S)
    return float(np.hypot(*step) / SPEED_WINDOW_S)


def _pose_at(trial: Trial, when: float) -> np.ndarray:
    return np.array([np.interp(when, trial.t, trial.pose[:, i]) for i in (0, 1)])


def travel_after(trial: Trial, when: float) -> float:
    """Straight-line distance from the robot's place at ``when`` to its place at the trial's end."""
    if math.isnan(when):
        return NAN
    return float(np.hypot(*(_pose_at(trial, trial.end_at) - _pose_at(trial, when))))


def room_at(trial: Trial, when: float) -> float:
    return NAN if math.isnan(when) else float(np.interp(when, trial.t, trial.room))


def guard_events(trial: Trial, after: float) -> tuple[float, float]:
    """When the guard first sent less than the command, and when it first sent zero.

    Only messages after the guard first passed a speed on, and after ``after``, count. Before
    that it is still holding the robot from the previous rest.

    Returns:
        The two times, nan where the guard never did it.
    """
    sent = trial.guard
    moving = np.nonzero(sent[:, 1] > 0)[0]
    if moving.size == 0:
        return NAN, NAN
    sent = sent[moving[0] :]
    sent = sent[sent[:, 0] >= after]
    cut = sent[sent[:, 1] < trial.speed - 1e-6]
    zero = sent[sent[:, 1] == 0.0]
    return (float(cut[0, 0]) if cut.size else NAN, float(zero[0, 0]) if zero.size else NAN)


def first_rest(trial: Trial, after: float) -> float:
    """First time after ``after`` the robot's true speed fell below REST_SPEED."""
    if math.isnan(after):
        return NAN
    step = np.hypot(*np.diff(trial.pose[:, :2], axis=0).T) / np.diff(trial.t)
    still = np.nonzero((trial.t[1:] >= after) & (step < REST_SPEED))[0]
    return float(trial.t[still[0] + 1]) if still.size else NAN


def released(trial: Trial) -> float:
    """Seconds from the box being taken away to the guard's first speed after it, or nan."""
    if math.isnan(trial.removed_at):
        return NAN
    sent = trial.guard[(trial.guard[:, 0] >= trial.removed_at) & (trial.guard[:, 1] > 0)]
    return float(sent[0, 0] - trial.removed_at) if sent.size else NAN


def outcome(trial: Trial) -> Outcome:
    appear = trial.kind == "appear"
    cut, stop = guard_events(trial, trial.placed_at if appear else -math.inf)
    if trial.ended == "passed":
        cut = stop = NAN  # the zero at the end is the experiment's, sent once the robot was past
    rest = first_rest(trial, stop)
    before_end = trial.t <= trial.end_at  # after it, the robot drives on where the box was
    return Outcome(
        trial=trial,
        room_end=room_at(trial, trial.end_at),
        room_min=float(trial.room[before_end].min()),
        cut_at=cut,
        stop_at=stop,
        room_at_cut=room_at(trial, cut),
        room_at_stop=room_at(trial, stop),
        speed_at_stop=speed_before(trial, stop),
        after_stop=travel_after(trial, stop),
        moved_again=travel_after(trial, rest),
        room_placed=room_at(trial, trial.placed_at) if appear else NAN,
        after_placed=travel_after(trial, trial.placed_at) if appear else NAN,
        reaction=cut - trial.placed_at if appear else NAN,
        released_after=released(trial),
    )


# ---------------------------------------------------------------------------
# Tables
# ---------------------------------------------------------------------------


def _stat(values: list[float], how: str = "median", scale: float = 1.0, digits: int = 2) -> str:
    values = [v for v in values if not math.isnan(v)]
    return f"{scale * getattr(np, how)(values):.{digits}f}" if values else "-"


def _cm(values: list[float], how: str = "median") -> str:
    return _stat(values, how, 100.0, 1)


def stop_estimate(speed: float, settings: dict, base: dict) -> float:
    """Metres covered from something appearing in the stop zone to rest, from the rates alone.

    A full scan period for the lidar to see it and a full cycle for the guard to send zero, at
    full speed, then braking at the base's limit. The messages' trip and the base's own response
    add a little to it.
    """
    wait = 1 / base["lidar_rate"] + 1 / settings["guard"]["rate"]
    return speed * wait + speed**2 / (2 * base["max_acceleration"])


def summarize(settings: dict, outcomes: list[Outcome]) -> str:
    guard, base = settings["guard"], robot_properties()
    lines = [
        "# Stopping: how much room the guard leaves",
        "",
        f"Guard: stop zone {guard['stop_distance']} m, slow zone {guard['slow_distance']} m at "
        f"{guard['slow_speed']} m/s, side margin {guard['side_margin']} m, {guard['rate']:g} Hz. "
        f"Base: top speed {base['max_speed']} m/s, braking at most {base['max_acceleration']} "
        f"m/s^2. Lidar: {base['lidar_rate']:g} Hz.",
        "",
        "Room is the shortest true distance between the robot's outline and anything solid. "
        f"{TOUCH_M * 1000:g} mm or less counts as touching.",
        "",
    ]
    approach = [o for o in outcomes if o.trial.kind == "approach"]
    if approach:
        lines += [
            "## Approaches, by speed",
            "",
            "Room when the guard cut the speed (slow zone) and when it sent zero (stop zone), the "
            "speed just before the stop, and the distance covered after it.",
            "",
            "| Speed | Trials | Stopped | Passed | Touched | Room at the cut | Room at the stop | "
            "Speed at the stop | After the stop, median (max) | Room at rest, median (min) |",
            "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
        ]
        for speed in sorted({o.trial.speed for o in approach}):
            group = [o for o in approach if o.trial.speed == speed]
            ended = [o.trial.ended for o in group]
            stopped = [o for o in group if o.trial.ended == "stopped"]
            lines.append(
                f"| {speed:.2f} m/s | {len(group)} | {ended.count('stopped')} | "
                f"{ended.count('passed')} | {sum(o.touched for o in group)} | "
                f"{_cm([o.room_at_cut for o in stopped])} cm | "
                f"{_cm([o.room_at_stop for o in stopped])} cm | "
                f"{_stat([o.speed_at_stop for o in stopped])} m/s | "
                f"{_cm([o.after_stop for o in stopped])} cm "
                f"({_cm([o.after_stop for o in stopped], 'max')}) | "
                f"{_cm([o.room_end for o in stopped])} cm "
                f"({_cm([o.room_end for o in stopped], 'min')}) |"
            )
        lines += [
            "",
            "## Approaches, by target",
            "",
            "Walls by the angle between the heading and the wall's normal, the post by the offset "
            "of its centre from the robot's centre line, all speeds together.",
            "",
            "| Target | Setting | Trials | Stopped | Passed | Room at rest, median (min) | "
            "Least room |",
            "|---|---|---:|---:|---:|---:|---:|",
        ]
        keys = list(dict.fromkeys((o.trial.target, o.trial.setting) for o in approach))
        for target, setting in keys:
            group = [o for o in approach if (o.trial.target, o.trial.setting) == (target, setting)]
            ended = [o.trial.ended for o in group]
            stopped = [o for o in group if o.trial.ended == "stopped"]
            at_rest = (
                f"{_cm([o.room_end for o in stopped])} cm "
                f"({_cm([o.room_end for o in stopped], 'min')})"
                if stopped
                else "-"
            )
            lines.append(
                f"| {target} | {setting} | {len(group)} | {ended.count('stopped')} | "
                f"{ended.count('passed')} | {at_rest} | "
                f"{_cm([o.room_min for o in group], 'min')} cm |"
            )
        again = [o for o in approach if o.moved_again > 0.001]
        moved = (
            f"{len(again)} of {len(approach)} approaches, at most "
            f"{_cm([o.moved_again for o in again], 'max')} cm"
            if again
            else f"none of the {len(approach)} approaches"
        )
        lines += ["", f"Moved again after coming to rest, with the command held: {moved}.", ""]

    appear = [o for o in outcomes if o.trial.kind == "appear"]
    if appear:
        stop_zone = guard["stop_distance"]
        lines += [
            "## A box appears in the way",
            "",
            f"Boxes that appeared inside the stop zone ({stop_zone} m) and were not touched: the "
            "distance from the moment the box was in place to rest, and the time until the guard "
            "first cut the speed. The estimate comes from the rates alone: a full lidar period and "
            "a full guard cycle at full speed, then braking at the base's limit.",
            "",
            "| Speed | Trials | Distance to rest, median (max) | Reaction, median (max) | "
            "Estimate |",
            "|---|---:|---:|---:|---:|",
        ]
        for speed in sorted({o.trial.speed for o in appear}):
            group = [
                o
                for o in appear
                if o.trial.speed == speed and o.room_placed <= stop_zone and not o.touched
            ]
            lines.append(
                f"| {speed:.2f} m/s | {len(group)} | {_cm([o.after_placed for o in group])} cm "
                f"({_cm([o.after_placed for o in group], 'max')}) | "
                f"{_stat([o.reaction for o in group], digits=3)} s "
                f"({_stat([o.reaction for o in group], 'max', digits=3)}) | "
                f"{100 * stop_estimate(speed, settings, base):.1f} cm |"
            )
        lines += [
            "",
            "Every appearance, by the room there was when the box appeared:",
            "",
            "| Speed | Room when it appeared | Trials | Touched | Room at rest, median |",
            "|---|---|---:|---:|---:|",
        ]
        edges = [0.0, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.5, 0.7]
        for speed in sorted({o.trial.speed for o in appear}):
            for lo, hi in zip(edges[:-1], edges[1:], strict=True):
                group = [o for o in appear if o.trial.speed == speed and lo <= o.room_placed < hi]
                if group:
                    lines.append(
                        f"| {speed:.2f} m/s | {lo:g}-{hi:g} m | {len(group)} | "
                        f"{sum(o.touched for o in group)} | {_cm([o.room_end for o in group])} cm |"
                    )
        removed = [o for o in appear if not math.isnan(o.trial.removed_at)]
        went_on = [o.released_after for o in removed if not math.isnan(o.released_after)]
        lines += [
            "",
            "## The box taken away",
            "",
            f"With the command still held, the box was taken away in {len(removed)} appearances. "
            f"The guard let the robot go on in {len(went_on)} of them, "
            f"{_stat(went_on, digits=2)} s after (median) and {_stat(went_on, 'max', digits=2)} s "
            f"at most. It waits for the way to stay clear {guard['resume_delay']:g} s, "
            f"{guard['resume_margin']:g} m past the stop zone.",
            "",
        ]
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Figures
# ---------------------------------------------------------------------------


def plot_approach(settings: dict, trials: list[Trial], path: Path) -> None:
    guard = settings["guard"]
    stop, slow = guard["stop_distance"], guard["slow_distance"]
    figure, axis = plt.subplots(figsize=(6.4, 3.6))
    axis.axvspan(0, stop, color=plot_style.AMBER, alpha=0.15)
    axis.axvspan(stop, slow, color=plot_style.CYAN, alpha=0.10)
    speeds = sorted({t.speed for t in trials if t.kind == "approach"})
    colours = (plot_style.EMERALD, plot_style.CYAN, plot_style.VIOLET, plot_style.AMBER)
    shown = set()
    for trial in trials:
        if (trial.target, trial.setting) != SHOWN_APPROACH or trial.speed in shown:
            continue
        shown.add(trial.speed)  # the first repeat at each speed
        step = np.hypot(*np.diff(trial.pose[:, :2], axis=0).T) / np.diff(trial.t)
        colour = colours[speeds.index(trial.speed) % len(colours)]
        axis.plot(trial.room[1:], step, color=colour, label=f"{trial.speed:.2f} m/s")
    axis.set_xlim(1.5, 0)
    axis.set_ylim(0, None)
    axis.set_xlabel("room ahead (m)")
    axis.set_ylabel("true speed (m/s)")
    axis.set_title("Driving head-on at a wall")
    axis.text(stop / 2, 0.02, "stop", ha="center", color=plot_style.AMBER)
    axis.text((stop + slow) / 2, 0.02, "slow", ha="center", color=plot_style.CYAN)
    axis.grid(True)
    axis.legend(loc="upper left")
    plot_style.save(figure, path)


def plot_appear(settings: dict, outcomes: list[Outcome], path: Path) -> None:
    guard, base = settings["guard"], robot_properties()
    stop_zone = guard["stop_distance"]
    appear = [
        o
        for o in outcomes
        if o.trial.kind == "appear" and o.room_placed <= stop_zone and not o.touched
    ]
    figure, axis = plt.subplots(figsize=(6.4, 3.8))
    speeds = np.linspace(0, base["max_speed"], 50)
    axis.plot(
        speeds,
        [100 * stop_estimate(v, settings, base) for v in speeds],
        color=plot_style.MUTED,
        linestyle="--",
        label="estimate from rates and braking",
    )
    axis.axhline(100 * stop_zone, color=plot_style.AMBER, linewidth=1.5)
    axis.text(0.01, 100 * stop_zone + 0.8, "stop zone", color=plot_style.AMBER)
    rng = np.random.default_rng(0)  # spreads the points of one speed sideways, to tell them apart
    axis.scatter(
        [o.trial.speed + rng.uniform(-0.012, 0.012) for o in appear],
        [100 * o.after_placed for o in appear],
        color=plot_style.CYAN,
        s=16,
        label="measured, one trial each",
        zorder=3,
    )
    axis.set_xlim(0, base["max_speed"] + 0.05)
    axis.set_ylim(0, 100 * stop_zone + 5)
    axis.set_xlabel("speed when the box appeared (m/s)")
    axis.set_ylabel("distance to rest (cm)")
    axis.set_title("From a box appearing to the robot at rest")
    axis.grid(True)
    axis.legend(loc="upper left", bbox_to_anchor=(0.0, 0.84))
    plot_style.save(figure, path)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--folder", type=Path, default=RESULTS / "stopping", help="results folder")
    args = parser.parse_args()
    settings, trials = load(args.folder)
    size = np.array(settings["obstacle_size"][:2])
    fixed = [(np.array(s.low[:2]), np.array(s.high[:2])) for s in solids().values()]
    for trial in trials:
        box = []
        if trial.obstacle is not None:
            centre = np.array(trial.obstacle)
            box = [(centre - size / 2, centre + size / 2)]
        trial.room = room_left(trial.pose, settings["guard"], fixed + box)
    outcomes = [outcome(t) for t in trials]
    text = summarize(settings, outcomes)
    (args.folder / "summary.md").write_text(text)
    plot_approach(settings, trials, args.folder / "approach.png")
    plot_appear(settings, outcomes, args.folder / "stopping-distance.png")
    print(text)


if __name__ == "__main__":
    main()
