#!/usr/bin/env python3
"""What the lidar guard sees, from above: the robot with the box ahead, then close to a post.

Gazebo places the robot at each pose and lets it settle. The figure draws, in the robot's frame,
what is solid in the world from its true geometry, the robot's outline, one lidar scan, and the
guard's zones as the guard itself reports them on /scan_guard/zones: solid while something is in
them, faint while clear.

Run from the repository root, with the guard on (see experiments/README.md):
    experiments/sim.sh guard:=true -- python3 experiments/guard_view.py
"""

from __future__ import annotations

import argparse
import math
import time
from collections.abc import Callable
from pathlib import Path

import numpy as np
import plot_style
import rclpy
from matplotlib.patches import Circle, Polygon, Rectangle
from nav_msgs.msg import Odometry
from plot_style import plt
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from sim_tools import RobotPlacer, guard_settings, seconds, yaw_of
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import MarkerArray
from world import RESULTS, solids

# Poses to look from: title, x, y and heading in degrees. The front edge is 0.18 m ahead of x.
VIEWS = (
    ("box in the slow zone", 0.97, 2.4, 0.0),  # 0.55 m from the box's face
    ("post in the stop zone", 0.06, 0.08, 0.0),  # 0.27 m from the post, which is off-centre
)
AHEAD_M, BEHIND_M, SIDE_M = 1.4, 0.5, 0.75  # the part of the room each panel shows
SETTLE_S = 1.0  # simulated seconds after Gazebo moves the robot
PLACED_M = 0.05  # how close to the asked position the robot must end up
OCCUPIED_ALPHA = 0.3  # the guard draws a zone more opaque than this while something is in it
WAIT_TIMEOUT_S = 10.0  # wall seconds
SOLID = "#475569"


class GuardView(Node):
    def __init__(self) -> None:
        super().__init__("guard_view")
        self.pose: tuple[float, float, float, float] | None = None  # t, x, y, yaw from Gazebo
        self.scan: LaserScan | None = None
        self.zones: MarkerArray | None = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(Odometry, "/ground_truth", self.on_truth, 10)
        self.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)
        self.create_subscription(MarkerArray, "/scan_guard/zones", self.on_zones, 10)

    def on_truth(self, msg: Odometry) -> None:
        p = msg.pose.pose
        self.pose = (seconds(msg.header.stamp), p.position.x, p.position.y, yaw_of(p.orientation))

    def on_scan(self, msg: LaserScan) -> None:
        self.scan = msg

    def on_zones(self, msg: MarkerArray) -> None:
        self.zones = msg

    def spin_until(self, done: Callable[[], bool], what: str) -> None:
        deadline = time.monotonic() + WAIT_TIMEOUT_S
        while not done():
            if time.monotonic() > deadline:
                raise RuntimeError(f"no {what}: is the simulation running with guard:=true?")
            rclpy.spin_once(self, timeout_sec=0.01)

    def lidar_mount(self, frame: str, base_frame: str = "base_footprint") -> tuple[float, float]:
        """Where the lidar sits on the robot, from TF, as the guard looks it up."""
        found: list = []

        def looked_up() -> bool:
            try:
                found.append(self.tf_buffer.lookup_transform(base_frame, frame, Time()))
            except TransformException:
                return False
            return True

        self.spin_until(looked_up, "transform to the lidar")
        offset = found[-1].transform.translation
        return offset.x, offset.y

    def look(self, placer: RobotPlacer, x: float, y: float, yaw: float) -> tuple:
        """Place the robot, let it settle, and take the next scan and zones after that.

        Returns:
            The robot's true pose (x, y, yaw), the scan, and the zones.
        """
        self.spin_until(lambda: self.pose is not None, "true pose")
        # Gazebo can miss a move sent right after start-up. The robot's true pose shows whether it
        # took, and the move is sent once more if not.
        for _ in range(2):
            placer.place(x, y, yaw)
            settled = self.pose[0] + SETTLE_S
            self.spin_until(lambda until=settled: self.pose[0] >= until, "clock")
            if math.hypot(self.pose[1] - x, self.pose[2] - y) < PLACED_M:
                break
        else:
            raise RuntimeError(f"Gazebo did not move the robot to ({x}, {y})")
        self.scan = self.zones = None
        self.spin_until(
            lambda: self.scan is not None and seconds(self.scan.header.stamp) >= settled, "scan"
        )
        self.zones = None
        self.spin_until(lambda: self.zones is not None, "zones from the guard")
        return self.pose[1:], self.scan, self.zones


def _in_robot_frame(points: np.ndarray, pose: tuple[float, float, float]) -> np.ndarray:
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    d = points - np.array([x, y])
    return np.column_stack((c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1]))


def draw(axis, title: str, pose, scan: LaserScan, zones, guard: dict, mount, blocks) -> None:
    """One panel, in the robot's frame: x forward, y to the left."""
    front, rear, half = guard["front"], guard["rear"], guard["half_width"]
    width = 2 * (half + guard["side_margin"])
    occupied = {(m.ns, m.id): m.color.a > OCCUPIED_ALPHA for m in zones.markers}

    for block in blocks.values():
        (x0, y0), (x1, y1) = block.low[:2], block.high[:2]
        corners = np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]])
        axis.add_patch(Polygon(_in_robot_frame(corners, pose), color=SOLID, lw=0, zorder=2))

    # The guard's slabs: the slow zone (id 0) reaches from the edge to slow_distance, and the stop
    # zone (id 1) is drawn over its first part. The same pair lies behind the robot.
    for ns, edge, sign in (("ahead", front, 1), ("behind", -rear, -1)):
        for zone_id, length, colour in (
            (0, guard["slow_distance"], plot_style.CYAN),
            (1, guard["stop_distance"], plot_style.AMBER),
        ):
            start = edge if sign > 0 else edge - length
            alpha = 0.45 if occupied.get((ns, zone_id), False) else 0.08
            axis.add_patch(
                Rectangle(
                    (start, -width / 2),
                    length,
                    width,
                    facecolor=colour,
                    alpha=alpha,
                    edgecolor=colour,
                    lw=1,
                )
            )

    axis.add_patch(
        Rectangle(
            (-rear, -half),
            front + rear,
            2 * half,
            facecolor=plot_style.PANEL,
            edgecolor=plot_style.MUTED,
            lw=1.5,
            zorder=2,
        )
    )
    axis.add_patch(Circle(mount, 0.025, fill=False, edgecolor=plot_style.EMERALD, lw=1.5, zorder=3))

    ranges = np.asarray(scan.ranges, dtype=float)
    angles = scan.angle_min + scan.angle_increment * np.arange(ranges.size)
    valid = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= scan.range_max)
    points = np.column_stack(
        (mount[0] + ranges * np.cos(angles), mount[1] + ranges * np.sin(angles))
    )[valid]
    axis.scatter(points[:, 0], points[:, 1], s=4, color=plot_style.EMERALD, zorder=3)

    axis.plot([-BEHIND_M + 0.1, -BEHIND_M + 0.6], [-SIDE_M + 0.12] * 2, color=plot_style.MUTED)
    axis.text(-BEHIND_M + 0.35, -SIDE_M + 0.16, "0.5 m", ha="center", color=plot_style.MUTED)
    axis.set_xlim(-BEHIND_M, AHEAD_M)
    axis.set_ylim(-SIDE_M, SIDE_M)
    axis.set_aspect("equal")
    axis.set_xticks([])
    axis.set_yticks([])
    axis.set_title(title)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--out", type=Path, default=RESULTS / "guard", help="output folder")
    args, ros_args = parser.parse_known_args()
    args.out.mkdir(parents=True, exist_ok=True)

    rclpy.init(args=ros_args)
    node = GuardView()
    placer = RobotPlacer()
    guard = guard_settings(node)
    blocks = solids()
    figure, axes = plt.subplots(1, len(VIEWS), figsize=(5 * len(VIEWS), 5.2))
    mount = None
    for axis, (title, x, y, heading) in zip(axes, VIEWS, strict=True):
        pose, scan, zones = node.look(placer, x, y, math.radians(heading))
        mount = mount or node.lidar_mount(scan.header.frame_id)
        draw(axis, title, pose, scan, zones, guard, mount, blocks)
        returns = sum(1 for r in scan.ranges if scan.range_min <= r <= scan.range_max)
        lit = sum(1 for m in zones.markers if m.color.a > OCCUPIED_ALPHA)
        print(
            f"  {title}: robot at ({pose[0]:.2f}, {pose[1]:.2f}, {math.degrees(pose[2]):.0f} deg), "
            f"{returns} returns, {lit} zones lit"
        )
    path = args.out / "guard-view.png"
    plot_style.save(figure, path)
    print(f"wrote {path}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
