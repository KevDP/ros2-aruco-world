#!/usr/bin/env python3
"""How far each pose estimate strays from the truth while the robot drives a fixed route.

One run per simulation. The robot starts where the launch spawns it and drives three laps of a
rectangle, steered by Gazebo's true pose so that every run follows the same path. On the way the
experiment records, next to the truth:
    wheels        the drifting wheel odometry the robot's nodes see (/wheel/odometry)
    local         the local filter, wheels and calibrated gyro (/odometry/local)
    global        the global filter, the same plus the marker fixes (/odometry/global)
    local_raw     a second local filter, fed the gyro as it comes, bias included (/imu)
    global_raw    a second global filter, likewise
    global_gate5  a global filter like global, with its gate at 5 sigma
    global_x4     like global_gate5, fed every fix with its covariance times 4
    global_x16    likewise, times 16
    global_2hz    like global_gate5, fed at most two fixes a second
    fix           every fix the localizer publishes (/aruco/robot_pose), when it arrived, and the
                  marker it came from as the detector saw it: measured distance, viewing angle,
                  reprojection error, ambiguity, and position and orientation in the camera frame

The extra filters copy the settings of ekf.yaml with the changes in EXTRA_FILTERS and publish no
TF. The experiment republishes the fixes for the last three: scaled, or thinned out. Every filter
sees the same drift, the same gyro and the same fixes. The localizer publishes one fix per
detection within reach and in its map, in the detections' order, and the experiment matches each
fix to its detection by stamp and order. Before driving, with the robot at rest, the
experiment also averages the raw gyro's rates, which at rest are its bias, to record it.

The wheel odometry and the local filters estimate the pose in odom, which starts at the spawn
pose, and the experiment puts them in the map through that pose.

Run from the repository root, once per run (see experiments/README.md):
    experiments/sim.sh detector:=true drift_seed:=1 -- python3 experiments/fusion.py --run 1
Then build the tables and figures with experiments/fusion_report.py.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from sim_tools import node_parameters, seconds, wrap, yaw_of
from world import RESULTS, WORLD_PACKAGE, localizer_reach, marker_map, robot_start

from aruco_interfaces.msg import MarkerDetectionArray

# The route: from the spawn pose to the first corner, then LAPS laps of a rectangle that keeps
# clear of the box, the posts and the walls. Corners in metres, in the map.
CORNERS = ((-2.5, -2.5), (2.5, -2.5), (2.5, 0.7), (-2.5, 0.7))
LAPS = 3
SPEED = 0.3  # m/s on the straights
TURN_RATE = 0.6  # rad/s at most, turning in place at the corners
TURN_GAIN, HEADING_GAIN = 2.0, 2.0  # rad/s per radian of heading error, turning and driving
FACING_RAD = math.radians(1.5)  # a turn ends when the heading is this close
ARRIVED_M = 0.05  # a straight ends this close to its corner, or once past it
REST_S = 3.0  # simulated seconds at rest before driving: settling, then the gyro's bias
BIAS_S = 1.5  # the last simulated seconds of that rest, averaged to record the gyro's bias
FILTER_START_S = 2.0  # simulated seconds at rest for the extra filters to settle
SAMPLE_S = 0.5  # one estimate row per source every this many simulated seconds
COMMAND_HZ = 20.0
TIMEOUT_S = 3600.0  # wall seconds for the whole run
WAIT_TIMEOUT_S = 30.0
SOURCES = {
    "wheels": "/wheel/odometry",
    "local": "/odometry/local",
    "global": "/odometry/global",
    "local_raw": "/odometry/local_raw",
    "global_raw": "/odometry/global_raw",
    "global_gate5": "/odometry/global_gate5",
    "global_x4": "/odometry/global_x4",
    "global_x16": "/odometry/global_x16",
    "global_2hz": "/odometry/global_2hz",
}
# Fixes republished for the extra filters: how much each one's covariance is scaled by.
SCALED_FIXES = {"/aruco/robot_pose_x4": 4.0, "/aruco/robot_pose_x16": 16.0}
THINNED_FIXES = "/aruco/robot_pose_2hz"  # at most one fix every THIN_S
# What a fix row records of the detection it came from, after when it arrived.
DETECTION_COLUMNS = (
    "marker", "measured_distance", "viewing_angle_deg", "reprojection_error", "ambiguity",
    "cam_x", "cam_y", "cam_z", "cam_qx", "cam_qy", "cam_qz", "cam_qw",
)  # fmt: skip
THIN_S = 0.5
# The extra filters: the ekf.yaml filter each copies, and what it changes besides publishing no TF.
EXTRA_FILTERS = {
    "local_raw": ("ekf_local", {"imu0": "/imu"}),
    "global_raw": ("ekf_global", {"imu0": "/imu"}),
    "global_gate5": ("ekf_global", {"pose0_rejection_threshold": 5.0}),
    "global_x4": (
        "ekf_global",
        {"pose0_rejection_threshold": 5.0, "pose0": "/aruco/robot_pose_x4"},
    ),
    "global_x16": (
        "ekf_global",
        {"pose0_rejection_threshold": 5.0, "pose0": "/aruco/robot_pose_x16"},
    ),
    "global_2hz": ("ekf_global", {"pose0_rejection_threshold": 5.0, "pose0": THINNED_FIXES}),
}


def _pose(msg: Odometry) -> tuple:
    """Stamp, x, y, heading and the covariance's xx, xy, yy and heading terms of an odometry."""
    p, c = msg.pose.pose, msg.pose.covariance
    return (seconds(msg.header.stamp), p.position.x, p.position.y, yaw_of(p.orientation),
            c[0], c[1], c[7], c[35])  # fmt: skip


class FusionRun(Node):
    def __init__(self) -> None:
        super().__init__("fusion_experiment")
        self.truth: list[tuple[float, float, float, float]] = []
        self.estimates: dict[str, list[tuple]] = {s: [] for s in SOURCES}
        self.fixes: list[tuple] = []
        # Image stamp to each detection the localizer turns into a fix, in order: id, distance,
        # viewing angle in degrees, reprojection error, ambiguity, then the marker's position and
        # orientation (quaternion) in the camera frame.
        self.detections: dict[float, list[tuple]] = {}
        self.reach = localizer_reach()
        self.known = set(marker_map())
        self.gyro: list[tuple[float, float, float, float]] = []  # stamp, rates about x, y, z
        self.bias: tuple[float, float, float] | None = None
        self.calibrated = False  # whether gyro_calibration has passed on a reading yet
        self.now = 0.0  # simulated seconds, from the gyro's stamps, which come every 10 ms
        self.create_subscription(Odometry, "/ground_truth", self.on_truth, 50)
        for source, topic in SOURCES.items():
            self.create_subscription(
                Odometry, topic, lambda msg, s=source: self.estimates[s].append(_pose(msg)), 50
            )
        self.create_subscription(PoseWithCovarianceStamped, "/aruco/robot_pose", self.on_fix, 50)
        self.create_subscription(MarkerDetectionArray, "/aruco/detections", self.on_detections, 50)
        self.scaled = {
            topic: (scale, self.create_publisher(PoseWithCovarianceStamped, topic, 50))
            for topic, scale in SCALED_FIXES.items()
        }
        self.thinned = self.create_publisher(PoseWithCovarianceStamped, THINNED_FIXES, 50)
        self.last_thinned = -math.inf
        self.create_subscription(Imu, "/imu", self.on_imu, qos_profile_sensor_data)
        self.create_subscription(
            Imu, "/imu/calibrated", self.on_calibrated, qos_profile_sensor_data
        )
        self.cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.deadline = time.monotonic() + TIMEOUT_S

    def on_truth(self, msg: Odometry) -> None:
        p = msg.pose.pose
        self.truth.append(
            (seconds(msg.header.stamp), p.position.x, p.position.y, yaw_of(p.orientation))
        )

    def on_fix(self, msg: PoseWithCovarianceStamped) -> None:
        p, c = msg.pose.pose, msg.pose.covariance
        stamp = seconds(msg.header.stamp)
        self.fixes.append(
            (stamp, p.position.x, p.position.y, yaw_of(p.orientation),
             c[0], c[1], c[7], c[35], self.now)
        )  # fmt: skip
        for scale, publisher in self.scaled.values():
            copy = PoseWithCovarianceStamped(header=msg.header, pose=msg.pose)
            copy.pose.covariance = [scale * v for v in c]
            publisher.publish(copy)
        if stamp - self.last_thinned >= THIN_S:
            self.last_thinned = stamp
            self.thinned.publish(msg)

    def on_detections(self, msg: MarkerDetectionArray) -> None:
        self.detections[seconds(msg.header.stamp)] = [
            (d.id, d.distance, math.degrees(d.viewing_angle), d.reprojection_error, d.ambiguity,
             d.pose.position.x, d.pose.position.y, d.pose.position.z, d.pose.orientation.x,
             d.pose.orientation.y, d.pose.orientation.z, d.pose.orientation.w)
            for d in msg.markers
            if d.distance <= self.reach and d.id in self.known
        ]  # fmt: skip

    def on_imu(self, msg: Imu) -> None:
        self.now = seconds(msg.header.stamp)
        w = msg.angular_velocity
        self.gyro.append((self.now, w.x, w.y, w.z))

    def on_calibrated(self, msg: Imu) -> None:
        self.calibrated = True

    def measure_bias(self) -> None:
        """Average the raw gyro's rates over the last BIAS_S of a rest: its bias."""
        since = self.gyro[-1][0] - BIAS_S
        rates = np.array([g[1:] for g in self.gyro if g[0] >= since])
        self.bias = tuple(float(v) for v in rates.mean(axis=0))

    # -- driving ------------------------------------------------------------

    def _spin(self) -> None:
        if time.monotonic() > self.deadline:
            raise RuntimeError("the run took too long: is the simulation still running?")
        rclpy.spin_once(self, timeout_sec=0.005)

    def wait_until_live(self, sources: tuple[str, ...]) -> None:
        deadline = time.monotonic() + WAIT_TIMEOUT_S
        while not (self.truth and self.calibrated and all(self.estimates[s] for s in sources)):
            if time.monotonic() > deadline:
                missing = ", ".join(sources)
                raise RuntimeError(f"no truth, calibrated gyro or {missing}: is everything up?")
            self._spin()

    def rest(self, sim_seconds: float) -> None:
        self.cmd.publish(Twist())
        until = self.truth[-1][0] + sim_seconds
        while self.truth[-1][0] < until:
            self._spin()

    def _steer(self, done, command) -> None:
        """Publish command() until done() holds, both reading the latest true pose."""
        next_send = 0.0
        while not done():
            if time.monotonic() >= next_send:
                cmd = Twist()
                cmd.linear.x, cmd.angular.z = command()
                self.cmd.publish(cmd)
                next_send = time.monotonic() + 1 / COMMAND_HZ
            self._spin()
        self.cmd.publish(Twist())

    def face(self, heading: float) -> None:
        def error() -> float:
            return wrap(heading - self.truth[-1][3])

        def command() -> tuple[float, float]:
            rate = float(np.clip(TURN_GAIN * error(), -TURN_RATE, TURN_RATE))
            return 0.0, math.copysign(max(abs(rate), 0.08), rate)

        self._steer(lambda: abs(error()) < FACING_RAD, command)

    def drive_to(self, start: tuple[float, float], end: tuple[float, float]) -> None:
        ux, uy = end[0] - start[0], end[1] - start[1]
        length = math.hypot(ux, uy)
        ux, uy = ux / length, uy / length

        def left() -> tuple[float, float]:
            """Distance to the corner, and how much of the leg is left along it."""
            _, x, y, _ = self.truth[-1]
            return math.hypot(end[0] - x, end[1] - y), (end[0] - x) * ux + (end[1] - y) * uy

        def command() -> tuple[float, float]:
            _, x, y, yaw = self.truth[-1]
            distance, _ = left()
            error = wrap(math.atan2(end[1] - y, end[0] - x) - yaw) if distance > 0.15 else 0.0
            turn = float(np.clip(HEADING_GAIN * error, -TURN_RATE, TURN_RATE))
            return min(SPEED, max(0.1, distance)), turn

        self._steer(lambda: left()[0] < ARRIVED_M or left()[1] < 0.0, command)

    def drive_route(self) -> None:
        _, x, y, _ = self.truth[-1]
        stops = [(x, y), *CORNERS * LAPS, CORNERS[0]]
        for start, end in zip(stops[:-1], stops[1:], strict=True):
            _, x, y, _ = self.truth[-1]
            self.face(math.atan2(end[1] - y, end[0] - x))
            self.drive_to(start, end)


# ---------------------------------------------------------------------------
# The second pair of filters
# ---------------------------------------------------------------------------


def start_extra_filters(folder: Path, extra: dict) -> list[subprocess.Popen]:
    """The filters of ``extra`` (EXTRA_FILTERS or none), each a copy of an ekf.yaml filter.

    Their settings are those of ekf.yaml with the changes EXTRA_FILTERS gives, a node name of their
    own and no TF. The global ones start at the spawn pose, as the launch's does.
    """
    settings = yaml.safe_load((WORLD_PACKAGE / "config" / "ekf.yaml").read_text())
    x, y, yaw = robot_start()
    processes = []
    for source, (base, changes) in extra.items():
        params = dict(settings[base]["ros__parameters"])
        params.update(publish_tf=False, use_sim_time=True, **changes)
        if base == "ekf_global":
            params["initial_state"] = [x, y, 0.0, 0.0, 0.0, yaw] + [0.0] * 9
        node = f"ekf_{source}"
        path = folder / f"{node}.yaml"
        path.write_text(yaml.safe_dump({node: {"ros__parameters": params}}))
        command = ["ros2", "run", "robot_localization", "ekf_node", "--ros-args",
                   "-r", f"__node:={node}", "--params-file", str(path),
                   "-r", f"odometry/filtered:={SOURCES[source]}"]  # fmt: skip
        processes.append(
            subprocess.Popen(
                command,
                start_new_session=True,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        )
    return processes


def stop(processes: list[subprocess.Popen]) -> None:
    for process in processes:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
    for process in processes:
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)


# ---------------------------------------------------------------------------
# Writing
# ---------------------------------------------------------------------------


def _to_map(row: tuple, start: tuple[float, float, float]) -> tuple:
    """An odom-frame row put in the map through the spawn pose, covariance included."""
    t, x, y, yaw, cxx, cxy, cyy, cyaw = row
    sx, sy, syaw = start
    c, s = math.cos(syaw), math.sin(syaw)
    rotation = np.array([[c, -s], [s, c]])
    cov = rotation @ np.array([[cxx, cxy], [cxy, cyy]]) @ rotation.T
    return (t, sx + c * x - s * y, sy + s * x + c * y, wrap(syaw + yaw),
            cov[0, 0], cov[0, 1], cov[1, 1], cyaw)  # fmt: skip


class TruthAt:
    """The true pose at any stamp, interpolated between Gazebo's messages."""

    def __init__(self, truth: list[tuple[float, float, float, float]]) -> None:
        rows = np.array(truth)
        self.t, self.x, self.y = rows[:, 0], rows[:, 1], rows[:, 2]
        self.yaw = np.unwrap(rows[:, 3])

    def __call__(self, t: float) -> tuple[float, float, float]:
        return (float(np.interp(t, self.t, self.x)), float(np.interp(t, self.t, self.y)),
                wrap(float(np.interp(t, self.t, self.yaw))))  # fmt: skip


def _fmt(values) -> list[str]:
    return [f"{v:.6g}" for v in values]


def write_run(node: FusionRun, run: int, t0: float, t_end: float, out: Path) -> None:
    start = robot_start()
    truth_at = TruthAt(node.truth)
    out.mkdir(parents=True, exist_ok=True)
    with open(out / f"run_{run:02d}.csv", "w", newline="") as f:
        writer = csv.writer(f, lineterminator="\n")
        writer.writerow(["source", "t", "x", "y", "yaw", "cov_xx", "cov_xy", "cov_yy", "cov_yaw",
                         "truth_x", "truth_y", "truth_yaw", "arrival", *DETECTION_COLUMNS])  # fmt: skip
        for source, rows in node.estimates.items():
            next_t = t0
            for row in rows:
                if not t0 <= row[0] <= t_end or row[0] < next_t:
                    continue
                next_t = row[0] + SAMPLE_S
                if not source.startswith("global"):
                    row = _to_map(row, start)
                writer.writerow([source, f"{row[0] - t0:.3f}", *_fmt(row[1:]),
                                 *_fmt(truth_at(row[0])), "", *[""] * len(DETECTION_COLUMNS)])  # fmt: skip
        matched: dict[float, int] = {}  # fixes of each stamp written so far
        for row in node.fixes:
            index = matched.get(row[0], 0)
            matched[row[0]] = index + 1
            if not t0 <= row[0] <= t_end:
                continue
            found = node.detections.get(row[0], [])
            if index < len(found):
                marker = [found[index][0], *_fmt(found[index][1:])]
            else:
                marker = [""] * len(DETECTION_COLUMNS)
            writer.writerow(["fix", f"{row[0] - t0:.3f}", *_fmt(row[1:8]),
                             *_fmt(truth_at(row[0])), f"{row[8] - t0:.3f}", *marker])  # fmt: skip


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--run", type=int, required=True, help="number of this run")
    parser.add_argument("--out", type=Path, default=RESULTS / "fusion", help="output folder")
    parser.add_argument(
        "--no-extra", action="store_true", help="only the robot's own filters, to collect fixes"
    )
    args, ros_args = parser.parse_known_args()
    extra = {} if args.no_extra else EXTRA_FILTERS

    # A plain kill still stops the extra filters on the way out, through the finally below.
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(1))
    rclpy.init(args=ros_args)
    node = FusionRun()
    started = time.monotonic()
    node.wait_until_live(("wheels", "local", "global"))
    seed = node_parameters(node, "noisy_odometry", ["seed"])["seed"]
    node.rest(REST_S)
    node.measure_bias()
    with tempfile.TemporaryDirectory() as folder:
        filters = start_extra_filters(Path(folder), extra)
        try:
            node.wait_until_live(tuple(extra))
            node.rest(FILTER_START_S)
            t0 = node.truth[-1][0]
            node.drive_route()
            node.rest(1.0)
            t_end = node.truth[-1][0]
        finally:
            stop(filters)

    write_run(node, args.run, t0, t_end, args.out / "runs")
    path = sum(
        math.hypot(b[1] - a[1], b[2] - a[2])
        for a, b in zip(node.truth[:-1], node.truth[1:], strict=True)
        if a[0] >= t0
    )
    fixes = sum(1 for row in node.fixes if t0 <= row[0] <= t_end)
    summary = args.out / "runs.csv"
    new = not summary.exists()
    with open(summary, "a", newline="") as f:
        writer = csv.writer(f, lineterminator="\n")
        if new:
            writer.writerow(["run", "drift_seed", "gyro_bias", "drive_s", "path_m", "fixes",
                             "wall_s"])  # fmt: skip
        writer.writerow([args.run, seed, f"{node.bias[2]:.6g}", f"{t_end - t0:.1f}",
                         f"{path:.2f}", fixes, f"{time.monotonic() - started:.0f}"])  # fmt: skip
    print(f"run {args.run}: {t_end - t0:.0f} s driven, {path:.1f} m, {fixes} fixes, "
          f"gyro bias {node.bias[2]:+.5f} rad/s")  # fmt: skip
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
