"""Room with ArUco markers, a robot with a camera, and a window showing what the camera sees.

Usage:
    ros2 launch aruco_world world.launch.py
    ros2 launch aruco_world world.launch.py image_view:=false
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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

    image_view = Node(
        package="rqt_image_view",
        executable="rqt_image_view",
        arguments=["/camera/image"],
        condition=IfCondition(LaunchConfiguration("image_view")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "image_view", default_value="true", description="Open a window with the camera"
            ),
            # Gazebo finds the robot and the markers through model:// URIs.
            AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.join(share, "models")),
            gazebo,
            bridge,
            image_view,
        ]
    )
