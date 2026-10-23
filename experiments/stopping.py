#!/usr/bin/env python3
"""How much room the lidar guard leaves between the robot and what stands in its way.

The experiment drives the robot through scan_guard the way a person with a teleop does: it sends a
constant speed on /cmd_vel_teleop twenty times a second, and the guard decides what reaches the
wheels. Two kinds of trial:

    approach  Gazebo places the robot at rest, aimed at a wall, a post or the box, and the command
              holds until the robot has stood still for a while.
    appear    The robot drives across open floor and Gazebo puts a box in its path, a set distance
              ahead of its front edge, as when something is carried or steps into the way. Once
              the robot has stopped, the box is taken away with the command still held, and the
              robot should move on by itself.

Each trial records the robot's true pose from Gazebo and every speed the guard sent, on the
simulation's clock, and settings.yaml keeps the guard's parameters as the running node reports
them. experiments/stopping_report.py works out the rest from those and from the world's geometry.

Run from the repository root (see experiments/README.md):
    experiments/sim.sh guard:=true -- python3 experiments/stopping.py
"""

from __future__ import annotations

import argparse
import csv
import math
import time
from collections import deque
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from sim_tools import Obstacle, RobotPlacer, guard_settings, seconds, yaw_of
from world import RESULTS, Solid, solids

SPEEDS = (0.15, 0.3, 0.45, 0.6)  # m/s, up to the base's top speed
WALL_ANGLES = (0, 15, 30, 45, 60, 75)  # degrees between the heading and the wall's normal
POST_OFFSETS = (0.0, 0.08, 0.16, 0.24)  # m from the robot's centre line to the post's centre
# Metres from the front edge to the new box. The robot covers a few centimetres more while Gazebo
# creates it, which puts 0.33 inside the stop zone at the top speeds.
APPEAR_GAPS = (0.1, 0.15, 0.2, 0.25, 0.3, 0.33, 0.45, 0.6)
REPEATS = 3  # per approach
APPEAR_REPEATS = 5  # per appearance: how soon the guard sees the box varies more
RUN_UP_M = 2.2  # along the heading, from the front edge at the start to what it drives at
WALL_AIM_Y = 1.2  # where the back wall is driven at, with clear floor all around
POST = "post_0"  # the post under marker 0, approached from the robot's side of the room
APPEAR_START = (-2.6, -3.2, math.pi / 2)  # x, y, yaw: open floor along the back wall
APPEAR_AT_Y = -1.0  # where the front edge is when the box goes in its way
OBSTACLE_SIZE = (0.3, 0.2, 0.6)  # m: about a person's legs, or a carried box
PASSED_M = 0.05  # how far past a post the rear edge goes before the robot counts as clear of it
COMMAND_HZ = 20.0
SETTLE_S = 1.0  # simulated seconds at rest after each move or change to the world
HOLD_S = 1.5  # simulated seconds the command stays up after the robot has stopped
REST_SPEED = 0.003  # m/s, below which the robot counts as stopped
MOVING_SPEED = 0.02  # m/s, above which a trial's robot has started to move
SPEED_WINDOW_S = 0.2  # simulated seconds the speed is measured over
TRIAL_TIMEOUT_S = 40.0  # simulated seconds
RESUME_TIMEOUT_S = 5.0  # simulated seconds for the robot to move on once the box is gone
WAIT_TIMEOUT_S = 10.0  # wall seconds without news from the simulation


@dataclass(frozen=True)
class Target:
    """Where an approach starts, aimed so that the front edge meets the target after RUN_UP_M.

    Attributes:
        kind: "wall", "post" or "box".
        setting: The wall's angle in degrees, the post's offset in metres, or the box's face or
            corner.
        start: x, y and yaw of the robot at rest.
        passed_x: Once the robot's centre is past this x, it has cleared a post it drove beside.
    """

    kind: str
    setting: str
    start: tuple[float, float, float]
    passed_x: float = math.inf


def approach_targets(blocks: dict[str, Solid], front: float, rear: float) -> list[Target]:
    """Every approach, from the world's geometry and the robot's outline as the guard has it."""
    run = RUN_UP_M + front  # from the robot's centre to the point its front edge drives at
    targets = []
    wall_x = blocks["wall_back"].high[0]
    for angle in WALL_ANGLES:
        yaw = math.pi - math.radians(angle)
        start = (wall_x - run * math.cos(yaw), WALL_AIM_Y - run * math.sin(yaw), yaw)
        targets.append(Target("wall", str(angle), start))
    post = blocks[POST]
    post_y = (post.low[1] + post.high[1]) / 2
    for offset in POST_OFFSETS:
        start = (post.low[0] - run, post_y + offset, 0.0)
        targets.append(Target("post", str(offset), start, post.high[0] + rear + PASSED_M))
    box = blocks["box"]
    targets.append(Target("box", "face", (box.low[0] - run, (box.low[1] + box.high[1]) / 2, 0.0)))
    diagonal = run / math.sqrt(2)  # toward the corner nearest the room's centre, at 45 degrees
    targets.append(
        Target("box", "corner", (box.low[0] - diagonal, box.low[1] - diagonal, math.pi / 4))
    )
    return targets


class Stopping(Node):
    def __init__(self) -> None:
        super().__init__(
            "stopping_experiment", parameter_overrides=[Parameter("use_sim_time", value=True)]
        )
        self.outline: dict[str, float] = {}  # the guard's parameters, front and rear among them
        self.pose: tuple[float, float, float, float] | None = None  # t, x, y, yaw from Gazebo
        self.guard_out = math.nan  # the last linear speed the guard sent, m/s
        self.trace: list[tuple[float, float, float, float]] | None = None
        self.guard_log: list[tuple[float, float]] | None = None
        self.moved = False  # whether the robot has started moving in the current trial
        self._recent: deque[tuple[float, float, float]] = deque()  # (t, x, y) for the speed
        self.create_subscription(Odometry, "/ground_truth", self.on_truth, 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_guard, 10)
        self.cmd = self.create_publisher(Twist, "/cmd_vel_teleop", 10)

    # -- what comes in ----------------------------------------------------------------------------

    def on_truth(self, msg: Odometry) -> None:
        p = msg.pose.pose
        self.pose = (seconds(msg.header.stamp), p.position.x, p.position.y, yaw_of(p.orientation))
        t, x, y, _ = self.pose
        self._recent.append((t, x, y))
        while len(self._recent) > 2 and self._recent[1][0] <= t - SPEED_WINDOW_S:
            self._recent.popleft()
        if self.speed() > MOVING_SPEED:
            self.moved = True
        if self.trace is not None:
            self.trace.append(self.pose)

    def on_guard(self, msg: Twist) -> None:
        self.guard_out = msg.linear.x
        if self.guard_log is not None:
            self.guard_log.append((self.now(), msg.linear.x))

    def now(self) -> float:
        """The simulation's time, in seconds."""
        return self.get_clock().now().nanoseconds * 1e-9

    def speed(self) -> float:
        """The robot's true speed over the last SPEED_WINDOW_S, in m/s, or inf until known."""
        if len(self._recent) < 2:
            return math.inf
        (t0, x0, y0), (t1, x1, y1) = self._recent[0], self._recent[-1]
        if t1 - t0 < SPEED_WINDOW_S / 2:
            return math.inf
        return math.hypot(x1 - x0, y1 - y0) / (t1 - t0)

    def stopped(self) -> bool:
        """Whether the robot moved in this trial and the guard has since brought it to rest."""
        return self.moved and self.guard_out == 0.0 and self.speed() < REST_SPEED

    # -- driving ------------------------------------------------------------------------------------

    def run(self, speed: float, done: Callable[[], bool], limit_s: float) -> bool:
        """Send ``speed`` at COMMAND_HZ until ``done()``, for at most ``limit_s`` simulated seconds.

        Returns:
            Whether ``done()`` came true.

        Raises:
            RuntimeError: if the simulation's clock stops.
        """
        command = Twist()
        command.linear.x = speed
        start = self.pose[0]
        last_tick = (time.monotonic(), start)  # wall time of the last tick of the clock
        next_send = 0.0
        while not done():
            now = self.pose[0]
            if now - start > limit_s:
                return False
            if now > last_tick[1]:
                last_tick = (time.monotonic(), now)
            elif time.monotonic() - last_tick[0] > WAIT_TIMEOUT_S:
                raise RuntimeError("the simulation's clock stopped")
            if time.monotonic() >= next_send:
                self.cmd.publish(command)
                next_send = time.monotonic() + 1 / COMMAND_HZ
            rclpy.spin_once(self, timeout_sec=0.01)
        return True

    def rest(self, sim_seconds: float = SETTLE_S) -> None:
        self.run(0.0, lambda: False, sim_seconds)

    def place(self, placer: RobotPlacer, pose: tuple[float, float, float]) -> None:
        """Stop, have Gazebo move the robot, and let it settle there."""
        self.rest(0.1)
        placer.place(*pose)
        self._recent.clear()
        self.rest()

    def wait_until_live(self, placer: RobotPlacer) -> None:
        """Wait for Gazebo's truth and for a guard that lets the robot move.

        Raises:
            RuntimeError: if either never comes.
        """
        deadline = time.monotonic() + WAIT_TIMEOUT_S
        while self.pose is None or math.isnan(self.guard_out):
            if time.monotonic() > deadline:
                raise RuntimeError("no true pose or no guard: run with guard:=true")
            rclpy.spin_once(self, timeout_sec=0.01)
        self.place(placer, APPEAR_START)
        # The guard holds the robot until it has a scan and the lidar's place on the robot.
        if not self.run(0.1, lambda: self.guard_out > 0.0, 3 * WAIT_TIMEOUT_S):
            raise RuntimeError("the guard never let the robot move")
        self.rest()

    # -- trials ---------------------------------------------------------------------------------

    def record(self) -> None:
        self.trace, self.guard_log, self.moved = [], [], False

    def recorded(self) -> tuple[list, list]:
        rows = (self.trace, self.guard_log)
        self.trace = self.guard_log = None
        return rows

    def approach(self, placer: RobotPlacer, target: Target, speed: float) -> str:
        """Drive at ``target`` until the guard stops the robot or it clears the target.

        Returns:
            How the trial ended: "stopped", "passed" or "timeout".
        """
        self.place(placer, target.start)
        self.record()

        def cleared() -> bool:
            return self.pose[1] > target.passed_x

        if not self.run(speed, lambda: cleared() or self.stopped(), TRIAL_TIMEOUT_S):
            ended = "timeout"
        elif cleared():
            ended = "passed"
        else:
            self.run(speed, lambda: False, HOLD_S)  # a held key keeps asking to move
            ended = "stopped"
        self.rest(0.1)
        return ended

    def appear(
        self, placer: RobotPlacer, obstacle: Obstacle, speed: float, gap: float
    ) -> tuple[str, float, tuple[float, float], float]:
        """Drive across open floor, have a box appear ``gap`` metres ahead of the front edge, and
        take it away again once the robot has stopped, with the command still held.

        Returns:
            How the trial ended ("stopped" or "timeout"), the time the box was in place, the box's
            centre, and the time it was taken away.
        """
        front = self.outline["front"]
        self.place(placer, APPEAR_START)
        self.record()
        if not self.run(speed, lambda: self.pose[2] + front >= APPEAR_AT_Y, TRIAL_TIMEOUT_S):
            return "timeout", math.nan, (math.nan, math.nan), math.nan
        centre = (self.pose[1], self.pose[2] + front + gap + OBSTACLE_SIZE[1] / 2)
        obstacle.put(*centre)
        placed_at = self.clock_after_request()
        ended = "stopped" if self.run(speed, self.stopped, TRIAL_TIMEOUT_S) else "timeout"
        removed_at = math.nan
        if ended == "stopped":
            self.run(speed, lambda: False, HOLD_S)
            obstacle.take_away()
            removed_at = self.clock_after_request()
            # The guard lets the robot go on by itself once the way has stayed clear.
            self.run(speed, lambda: self.guard_out > 0.0, RESUME_TIMEOUT_S)
        else:
            obstacle.take_away()
        self.rest()
        return ended, placed_at, centre, removed_at

    def clock_after_request(self) -> float:
        """The simulation's time once the clock messages that came during a request are in."""
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.0)
        return self.now()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--out", type=Path, default=RESULTS / "stopping", help="output folder")
    parser.add_argument("--repeats", type=int, help="trials per condition, for both kinds")
    parser.add_argument("--speeds", type=float, nargs="+", default=SPEEDS, help="m/s to try")
    parser.add_argument("--only", choices=("approach", "appear"), help="one kind of trial")
    args, ros_args = parser.parse_known_args()
    args.out.mkdir(parents=True, exist_ok=True)

    rclpy.init(args=ros_args)
    node = Stopping()
    placer = RobotPlacer()
    obstacle = Obstacle(OBSTACLE_SIZE)
    settings = guard_settings(node)
    node.outline.update(settings)
    record = {"guard": settings, "obstacle_size": list(OBSTACLE_SIZE)}
    (args.out / "settings.yaml").write_text(yaml.safe_dump(record, sort_keys=False))
    node.wait_until_live(placer)

    trials = []
    targets = approach_targets(solids(), settings["front"], settings["rear"])
    if args.only != "appear":
        for repeat in range(args.repeats or REPEATS):
            trials += [("approach", t, s, repeat) for t in targets for s in args.speeds]
    if args.only != "approach":
        for repeat in range(args.repeats or APPEAR_REPEATS):
            trials += [("appear", g, s, repeat) for g in APPEAR_GAPS for s in args.speeds]

    with (
        open(args.out / "trials.csv", "w", newline="") as trials_file,
        open(args.out / "trace.csv", "w", newline="") as trace_file,
        open(args.out / "guard.csv", "w", newline="") as guard_file,
    ):
        summary, trace, guard = (
            csv.writer(f, lineterminator="\n") for f in (trials_file, trace_file, guard_file)
        )
        summary.writerow(
            ["trial", "kind", "target", "setting", "speed", "repeat", "start_x", "start_y",
             "start_yaw", "obstacle_x", "obstacle_y", "obstacle_at", "removed_at", "ended"]
        )  # fmt: skip
        trace.writerow(["trial", "t", "x", "y", "yaw"])
        guard.writerow(["trial", "t", "linear"])
        for number, (kind, what, speed, repeat) in enumerate(trials):
            if kind == "approach":
                ended = node.approach(placer, what, speed)
                target, setting, start = what.kind, what.setting, what.start
                obstacle_at, removed_at, centre = math.nan, math.nan, (math.nan, math.nan)
            else:
                ended, obstacle_at, centre, removed_at = node.appear(placer, obstacle, speed, what)
                target, setting, start = "box in the way", str(what), APPEAR_START
            poses, guard_rows = node.recorded()
            summary.writerow(
                [number, kind, target, setting, speed, repeat, *(f"{v:.4f}" for v in start),
                 *(f"{v:.4f}" for v in centre), f"{obstacle_at:.4f}", f"{removed_at:.4f}", ended]
            )  # fmt: skip
            trace.writerows([number, *(f"{v:.4f}" for v in row)] for row in poses)
            guard.writerows([number, f"{t:.4f}", f"{v:.4f}"] for t, v in guard_rows)
            for f in (trials_file, trace_file, guard_file):
                f.flush()
            print(f"  {number:3d} {kind:8s} {target:14s} {setting:6s} {speed:.2f} m/s: {ended}")
    print(f"done: {args.out}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
