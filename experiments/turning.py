#!/usr/bin/env python3
"""How far the wheel odometry strays from the truth when the robot turns in place and drives.

Each trial starts with Gazebo placing the robot at rest. A turn trial turns in place at a set rate
for long enough to make a set number of turns, then stops. A straight trial drives forward at a
set speed for a set distance. Both are open loop, timed on the simulation's clock.
Each trial records how far Gazebo's wheel odometry says the robot turned or travelled and how far
it really did, plus the two over time.

Run once per wheel collision shape, from the repository root (see experiments/README.md):
    experiments/sim.sh wheel_collision:=sphere -- python3 experiments/turning.py --label sphere
    experiments/sim.sh wheel_collision:=cylinder -- python3 experiments/turning.py --label cylinder
Then build the tables and figures with experiments/turning_report.py.
"""

from __future__ import annotations

import argparse
import csv
import math
import time
from collections.abc import Callable
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sim_tools import RobotPlacer, seconds, wrap, yaw_of
from world import RESULTS

TURN_RATES = (0.3, 0.6, 1.2)  # rad/s, under the base's cap of 1.5
TURNS = 2
SPEEDS = (0.2, 0.4)  # m/s
STRAIGHT_M = 2.0
REPEATS = 3
TURN_START = (-1.5, 0.0, 0.0)  # x, y, yaw: the spawn pose, clear of everything when turning
STRAIGHT_START = (-3.2, 0.0, 0.0)  # leaves room for the drive before the first post
SETTLE_S = 1.0  # simulated seconds to come to rest. Stopping from 1.2 rad/s takes 0.6 s.
COMMAND_HZ = 20.0
WAIT_TIMEOUT_S = 10.0


class Track:
    """Heading turned and distance travelled by one odometry source since the last reset.

    While ``history`` is a list, every message adds (stamp, turned, travelled) to it. Each source
    keeps its own stamps: the two arrive at different instants, and pairing each message with the
    other source's latest one would mix in up to a message period of motion.
    """

    def __init__(self) -> None:
        self.stamp = 0.0
        self.turned = 0.0
        self.travelled = 0.0
        self.history: list[tuple[float, float, float]] | None = None
        self._last: tuple[float, float, float] | None = None

    def update(self, msg: Odometry) -> None:
        p = msg.pose.pose
        now = (p.position.x, p.position.y, yaw_of(p.orientation))
        if self._last is not None:
            self.turned += wrap(now[2] - self._last[2])
            self.travelled += math.hypot(now[0] - self._last[0], now[1] - self._last[1])
        self._last = now
        self.stamp = seconds(msg.header.stamp)
        if self.history is not None:
            self.history.append((self.stamp, self.turned, self.travelled))

    def reset(self) -> None:
        self.turned = self.travelled = 0.0


class Turning(Node):
    def __init__(self) -> None:
        super().__init__("turning_experiment")
        self.odom, self.truth = Track(), Track()
        self.trace: dict[str, list[tuple[float, float, float]]] = {}
        self.create_subscription(Odometry, "/sim/wheel_odometry", self.odom.update, 10)
        self.create_subscription(Odometry, "/ground_truth", self.truth.update, 10)
        self.cmd = self.create_publisher(Twist, "/cmd_vel", 10)

    def _spin_until(self, done: Callable[[], bool]) -> None:
        deadline = time.monotonic() + WAIT_TIMEOUT_S
        while not done():
            if time.monotonic() > deadline:
                raise RuntimeError("no odometry or true pose: is the simulation running?")
            rclpy.spin_once(self, timeout_sec=0.01)

    def _rest_for(self, sim_seconds: float) -> None:
        until = self.truth.stamp + sim_seconds
        self._spin_until(lambda: self.truth.stamp >= until and self.odom.stamp >= until)

    def _drive_for(self, linear: float, angular: float, sim_seconds: float) -> None:
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = linear, angular
        until = self.truth.stamp + sim_seconds
        deadline = time.monotonic() + 3 * sim_seconds + WAIT_TIMEOUT_S
        next_send = 0.0
        while self.truth.stamp < until:
            if time.monotonic() > deadline:
                raise RuntimeError("the simulation's clock stopped while driving")
            if time.monotonic() >= next_send:
                self.cmd.publish(cmd)
                next_send = time.monotonic() + 1 / COMMAND_HZ
            rclpy.spin_once(self, timeout_sec=0.01)
        self.cmd.publish(Twist())

    def wait_until_live(self) -> None:
        self._spin_until(lambda: self.odom.stamp > 0 and self.truth.stamp > 0)

    def trial(
        self, placer: RobotPlacer, start: tuple, linear: float, angular: float, duration: float
    ) -> tuple[float, float, float, float]:
        """One move from rest to rest.

        Returns:
            (odometry turn, true turn, odometry distance, true distance), radians and metres.
        """
        self.cmd.publish(Twist())
        placer.place(*start)
        self._rest_for(SETTLE_S)
        self.odom.reset()
        self.truth.reset()
        start = self.truth.stamp
        self.odom.history, self.truth.history = [], []
        self._drive_for(linear, angular, duration)
        self._rest_for(SETTLE_S)
        self.trace = {
            "odom": [(t - start, a, d) for t, a, d in self.odom.history],
            "truth": [(t - start, a, d) for t, a, d in self.truth.history],
        }
        self.odom.history = self.truth.history = None
        return self.odom.turned, self.truth.turned, self.odom.travelled, self.truth.travelled


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--label", required=True, help="what this run is, e.g. the wheel shape")
    parser.add_argument("--out", type=Path, default=RESULTS / "turning", help="output folder")
    parser.add_argument("--repeats", type=int, default=REPEATS, help="trials per condition")
    args, ros_args = parser.parse_known_args()
    args.out.mkdir(parents=True, exist_ok=True)

    trials = []
    for repeat in range(args.repeats):
        for rate in TURN_RATES:
            for direction in (1, -1):
                trials.append(("turn", rate, direction, repeat))
        for speed in SPEEDS:
            trials.append(("straight", speed, 1, repeat))

    rclpy.init(args=ros_args)
    node = Turning()
    placer = RobotPlacer()
    node.wait_until_live()

    with (
        open(args.out / f"trials_{args.label}.csv", "w", newline="") as trials_file,
        open(args.out / f"trace_{args.label}.csv", "w", newline="") as trace_file,
    ):
        summary = csv.writer(trials_file)
        trace = csv.writer(trace_file)
        summary.writerow(
            ["label", "kind", "command", "direction", "repeat",
             "odom_turn", "true_turn", "odom_distance", "true_distance"]
        )  # fmt: skip
        trace.writerow(["label", "trial", "source", "t", "turned", "travelled"])
        for number, (kind, command, direction, repeat) in enumerate(trials):
            if kind == "turn":
                result = node.trial(
                    placer, TURN_START, 0.0, direction * command, TURNS * 2 * math.pi / command
                )
            else:
                result = node.trial(placer, STRAIGHT_START, command, 0.0, STRAIGHT_M / command)
            summary.writerow(
                [args.label, kind, command, direction, repeat, *(f"{v:.5f}" for v in result)]
            )
            for source, rows in node.trace.items():
                for row in rows:
                    trace.writerow([args.label, number, source, *(f"{v:.4f}" for v in row)])
            odom_turn, true_turn, odom_distance, true_distance = result
            print(
                f"  {kind:8s} {command:.1f} x{direction:+d}: odometry {math.degrees(odom_turn):8.1f}"
                f" deg {odom_distance:6.3f} m, truth {math.degrees(true_turn):8.1f} deg"
                f" {true_distance:6.3f} m"
            )
    print(f"done: {args.out}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
