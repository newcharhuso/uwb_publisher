#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Node declaration




    launch_group = GroupAction(
        [
            Node(
                package="uwb_publisher",
                executable="uwb_publisher_node",
                namespace="SR4T1",
                output="screen",
                respawn=True,
                respawn_delay=1.0,
                parameters=[
                    {"serial_port": "/dev/ttyUSB2"},
                    {"baud_rate": 115200},
                ],
                remappings=[
                    ("/tf", "tf"),
                    ("/tf_static", "tf_static"),
                ],
            ),
        ]
    )

    # LaunchDescription
    ld = LaunchDescription()
    ld.add_action(launch_group)
    return ld