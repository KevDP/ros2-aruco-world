"""Room with ArUco markers, a robot with a camera, a lidar and an IMU, the marker detector, the
localizer that places the robot from the markers, the filters that fuse it all, the lidar guard and
RViz.

The robot is described once, in urdf/camera_bot.urdf.xacro. robot_state_publisher publishes that
description and the robot's TF, and Gazebo spawns the robot from it at the start pose that
build_world.py writes. The map frame is Gazebo's world frame.

Two robot_localization filters, set up in config/ekf.yaml, publish odom -> base_footprint and
map -> odom.

With the guard on (the default), the robot obeys /cmd_vel_teleop through scan_guard, which owns
/cmd_vel. Drive it with:
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_teleop

Usage:
    ros2 launch aruco_world world.launch.py
    ros2 launch aruco_world world.launch.py detector:=false                 # raw camera only
    ros2 launch aruco_world world.launch.py rviz:=false image_view:=true    # camera window only
    ros2 launch aruco_world world.launch.py gz_gui:=false                   # RViz only, lighter
    ros2 launch aruco_world world.launch.py guard:=false                    # teleop on /cmd_vel
    ros2 launch aruco_world world.launch.py wheel_collision:=cylinder       # to compare wheels
"""

from __future__ import annotations

import os
import xml.etree.ElementTree as ET

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Nodes that read the clock use Gazebo's, bridged on /clock, and TF and images share one time base.
# With mixed clocks RViz drops data as too old. The detector stays off /clock because it only
# copies each image's stamp, and /clock arrives at every physics step.
SIM_TIME = {"use_sim_time": True}


def _enabled(name: str, context: LaunchContext) -> bool:
    return LaunchConfiguration(name).perform(context).lower() in ("true", "1")


def _footprint(urdf_xacro: str) -> dict[str, float]:
    """The robot's outline for scan_guard, from the properties the URDF builds the robot with.

    Front and rear are half the body's length. The half width reaches the outer face of the wheels
    when they stick out past the body.
    """
    tag = "{http://www.ros.org/wiki/xacro}property"
    props = {e.get("name"): e.get("value") for e in ET.parse(urdf_xacro).getroot().iter(tag)}
    half_length = float(props["body_length"]) / 2
    wheels = float(props["wheel_separation"]) / 2 + float(props["wheel_width"]) / 2
    half_width = max(float(props["body_width"]) / 2, wheels)
    return {"front": half_length, "rear": half_length, "half_width": half_width}


WHEEL_COLLISIONS = ("sphere", "cylinder")


def _robot_state_publisher(context: LaunchContext, urdf_xacro: str) -> list[Node]:
    """robot_state_publisher with the robot's description, built for the chosen wheel shape.

    The launch argument's choices already reject any other shape.
    """
    wheel_collision = LaunchConfiguration("wheel_collision").perform(context)
    description = xacro.process_file(
        urdf_xacro, mappings={"wheel_collision": wheel_collision}
    ).toxml()
    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": description}, SIM_TIME],
            output="screen",
        )
    ]


def _image_view(context: LaunchContext) -> list[Node]:
    """The camera window: the detector's annotated image when it runs, the raw camera otherwise."""
    if not _enabled("image_view", context):
        return []
    topic = "/aruco/image" if _enabled("detector", context) else "/camera/image"
    return [Node(package="rqt_image_view", executable="rqt_image_view", arguments=[topic])]


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("aruco_world")
    world = os.path.join(share, "worlds", "aruco_room.sdf")
    urdf = os.path.join(share, "urdf", "camera_bot.urdf.xacro")
    with open(os.path.join(share, "config", "robot_start.yaml")) as f:
        start = yaml.safe_load(f)

    # Two processes: closing or crashing the window leaves the server, and the camera, running.
    gz_sim = os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
    gazebo_server = GroupAction(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gz_sim),
                launch_arguments={"gz_args": f"-s -r {world}"}.items(),
            )
        ]
    )
    gazebo_window = GroupAction(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gz_sim), launch_arguments={"gz_args": "-g"}.items()
            )
        ],
        condition=IfCondition(LaunchConfiguration("gz_gui")),
    )

    robot_state_publisher = OpaqueFunction(function=_robot_state_publisher, args=[urdf])

    # Gazebo builds the robot from the description robot_state_publisher publishes.
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description",
            "-name", "camera_bot",
            "-x", str(start["x"]),
            "-y", str(start["y"]),
            "-z", str(start["z"]),
            "-Y", str(start["yaw"]),
        ],
        output="screen",
    )  # fmt: skip

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"config_file": os.path.join(share, "config", "bridge.yaml")}],
        output="screen",
    )

    # Dictionary and marker size come from the file build_world.py generates with the markers.
    detector = Node(
        package="aruco_detector",
        executable="detector",
        name="aruco_detector",
        parameters=[os.path.join(share, "config", "aruco_markers.yaml")],
        remappings=[("image", "/camera/image"), ("camera_info", "/camera/camera_info")],
        output="screen",
        condition=IfCondition(LaunchConfiguration("detector")),
    )

    # Adds to Gazebo's nearly exact wheel odometry the drift a real robot's has.
    noisy_odometry = Node(
        package="noisy_odometry",
        executable="noisy_odometry",
        name="noisy_odometry",
        parameters=[os.path.join(share, "config", "noisy_odometry.yaml")],
        remappings=[("odom_exact", "/sim/wheel_odometry"), ("odom", "/wheel/odometry")],
        output="screen",
    )

    # The two filters share one file. The global one starts where the robot was spawned, the local
    # one at its own origin, like the odometry. robot_localization's state is x, y, z, roll, pitch,
    # yaw, then six velocities and three linear accelerations, all zero at rest.
    ekf = os.path.join(share, "config", "ekf.yaml")
    start_state = [start["x"], start["y"], 0.0, 0.0, 0.0, start["yaw"]] + [0.0] * 9
    ekf_local = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_local",
        parameters=[ekf, SIM_TIME],
        remappings=[("odometry/filtered", "/odometry/local")],
        output="screen",
    )
    ekf_global = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_global",
        parameters=[ekf, SIM_TIME, {"initial_state": [float(v) for v in start_state]}],
        remappings=[("odometry/filtered", "/odometry/global")],
        output="screen",
    )

    # The marker map is generated by build_world.py as well.
    localizer = Node(
        package="aruco_localization",
        executable="localizer",
        name="aruco_localizer",
        parameters=[
            os.path.join(share, "config", "marker_map.yaml"),
            os.path.join(share, "config", "aruco_localizer.yaml"),
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("detector")),
    )

    # The guard sits between the driver and the robot, and nothing else publishes on /cmd_vel.
    # Zone sizes come from its YAML, the robot's outline from the URDF.
    guard = Node(
        package="scan_guard",
        executable="guard",
        name="scan_guard",
        parameters=[os.path.join(share, "config", "scan_guard.yaml"), _footprint(urdf)],
        remappings=[
            ("scan", "/scan"),
            ("cmd_vel_in", "/cmd_vel_teleop"),
            ("cmd_vel_out", "/cmd_vel"),
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("guard")),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(share, "rviz", "aruco_world.rviz")],
        parameters=[SIM_TIME],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "detector", default_value="true", description="Run the ArUco detector"
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Open RViz with the robot, its frames, the detections and the camera",
            ),
            DeclareLaunchArgument(
                "image_view", default_value="false", description="Open a separate camera window"
            ),
            DeclareLaunchArgument(
                "gz_gui", default_value="true", description="Open Gazebo's own window"
            ),
            DeclareLaunchArgument(
                "guard",
                default_value="true",
                description="Guard the robot with the lidar: drive on /cmd_vel_teleop",
            ),
            DeclareLaunchArgument(
                "wheel_collision",
                default_value="sphere",
                choices=list(WHEEL_COLLISIONS),
                description="Collision shape of the wheels (cylinder only to compare with sphere)",
            ),
            # Gazebo finds the markers through model:// URIs.
            AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.join(share, "models")),
            gazebo_server,
            gazebo_window,
            robot_state_publisher,
            spawn,
            bridge,
            noisy_odometry,
            ekf_local,
            ekf_global,
            detector,
            localizer,
            guard,
            rviz,
            OpaqueFunction(function=_image_view),
        ]
    )
