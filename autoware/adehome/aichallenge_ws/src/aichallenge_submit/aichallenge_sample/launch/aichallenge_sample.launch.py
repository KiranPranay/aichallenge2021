from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():
    aichallgenge_sample_pkg_prefix = get_package_share_directory('aichallenge_sample')
    mpc_param_file = os.path.join(
        aichallgenge_sample_pkg_prefix, 'param/mpc.param.yaml')

    pc_filter_transform_param_file = os.path.join(
        aichallgenge_sample_pkg_prefix, 'param/pc_filter_transform.param.yaml')

    # Arguments

    mpc_param = DeclareLaunchArgument(
        'mpc_param_file',
        default_value=mpc_param_file,
        description='Path to config file for MPC'
    )
    pc_filter_transform_param = DeclareLaunchArgument(
        'pc_filter_transform_param_file',
        default_value=pc_filter_transform_param_file,
        description='Path to config file for Point Cloud Filter/Transform Nodes'
    )

    # Nodes
    filter_transform_vlp16_front = Node(
        package='point_cloud_filter_transform_nodes',
        node_executable='point_cloud_filter_transform_node_exe',
        node_name='filter_transform_vlp16_front',
        node_namespace='lidar_front',
        parameters=[LaunchConfiguration('pc_filter_transform_param_file')],
        remappings=[("points_in", "points_raw")]
    )

    aichallenge_sample = Node(
        package='aichallenge_sample',
        node_executable='aichallenge_sample_node_exe',
        node_namespace='aichallenge',
        output='screen',
        node_name='aichallenge_sample_node'
    )

    mpc = Node(
        package='mpc_controller_nodes',
        node_executable='mpc_controller_node_exe',
        node_name='mpc_controller_node',
        node_namespace='control',
        parameters=[LaunchConfiguration('mpc_param_file')]
    )

    sample_localizer = Node(
        package='sample_localizer',
        node_executable='sample_localizer_node_exe',
        node_name='sample_localizer_node',
        node_namespace='localization'
    )

    core_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([aichallgenge_sample_pkg_prefix, '/launch/ms3_core.launch.py']),
        launch_arguments={}.items()
    )

    return LaunchDescription([
        mpc_param,
        pc_filter_transform_param,
        mpc,
        filter_transform_vlp16_front,
        core_launch,
        sample_localizer,
        aichallenge_sample,
    ])
