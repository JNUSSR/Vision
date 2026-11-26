#!/usr/bin/python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory("fast_livo"), "config")
    default_config = os.path.join(config_dir, "mid360_only.yaml")
    params_file = LaunchConfiguration("mid360_params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mid360_params_file",
                default_value=default_config,
                description="FAST-LIVO2 Mid-360 LIO parameters",
            ),
            Node(
                package="fast_livo",
                executable="fastlivo_mapping",
                name="laserMapping",
                parameters=[params_file],
                output="screen",
            ),
        ]
    )
