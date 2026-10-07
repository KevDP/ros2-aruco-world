"""Room with ArUco markers, a robot with a camera, the marker detector and a camera window.

Usage:
    ros2 launch aruco_world world.launch.py
    ros2 launch aruco_world world.launch.py detector:=false     # raw camera only
    ros2 launch aruco_world world.launch.py image_view:=false   # no window
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": f"-r {world}"}.items(),
    )

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

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "detector", default_value="true", description="Run the ArUco detector"
            ),
            DeclareLaunchArgument(
                "image_view", default_value="true", description="Open a window with the camera"
            ),
            # Gazebo finds the robot and the markers through model:// URIs.
            AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.join(share, "models")),
            gazebo,
            bridge,
            detector,
            OpaqueFunction(function=_image_view),
        ]
    )
