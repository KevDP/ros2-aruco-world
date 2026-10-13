"""Room with ArUco markers, a robot with a camera, the marker detector and RViz.

The robot is described once, in urdf/camera_bot.urdf.xacro: robot_state_publisher publishes that
description and the robot's TF, and Gazebo spawns the robot from the same description at the start
pose that build_world.py writes.

Usage:
    ros2 launch aruco_world world.launch.py
    ros2 launch aruco_world world.launch.py detector:=false                 # raw camera only
    ros2 launch aruco_world world.launch.py rviz:=false image_view:=true    # camera window only
    ros2 launch aruco_world world.launch.py gz_gui:=false                   # RViz only, lighter
"""

from __future__ import annotations

import os

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

# Nodes that read the clock run on Gazebo's, bridged on /clock, so TF and images share one time
# base; mixed clocks make RViz drop data as too old. The detector stays off /clock: it never reads
# the clock, it only copies each image's stamp, and /clock arrives at every physics step.
SIM_TIME = {"use_sim_time": True}


def _enabled(name: str, context: LaunchContext) -> bool:
    return LaunchConfiguration(name).perform(context).lower() in ("true", "1")


def _image_view(context: LaunchContext) -> list[Node]:
    """The camera window: the detector's annotated image when it runs, the raw camera otherwise."""
    if not _enabled("image_view", context):
        return []
    topic = "/aruco/image" if _enabled("detector", context) else "/camera/image"
    return [Node(package="rqt_image_view", executable="rqt_image_view", arguments=[topic])]


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("aruco_world")
    world = os.path.join(share, "worlds", "aruco_room.sdf")
    description = xacro.process_file(os.path.join(share, "urdf", "camera_bot.urdf.xacro")).toxml()
    with open(os.path.join(share, "config", "robot_start.yaml")) as f:
        start = yaml.safe_load(f)

    # Server and window run as separate processes, so losing the window never stops the simulation
    # or the camera.
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

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": description}, SIM_TIME],
        output="screen",
    )

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
            # Gazebo finds the markers through model:// URIs.
            AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.join(share, "models")),
            gazebo_server,
            gazebo_window,
            robot_state_publisher,
            spawn,
            bridge,
            detector,
            rviz,
            OpaqueFunction(function=_image_view),
        ]
    )
