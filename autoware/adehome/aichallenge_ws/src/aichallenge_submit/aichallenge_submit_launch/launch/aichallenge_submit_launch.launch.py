from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():
    aichallenge_sample_pkg_prefix = get_package_share_directory('aichallenge_sample')

    aichallenge_sample_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([aichallenge_sample_pkg_prefix, '/launch/aichallenge_sample.launch.py']),
        launch_arguments={}.items()
    )

    return LaunchDescription([
        aichallenge_sample_launch,
    ])
