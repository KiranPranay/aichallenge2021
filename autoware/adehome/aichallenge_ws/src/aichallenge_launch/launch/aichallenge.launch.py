from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():
    aichallenge_launch_pkg_prefix = get_package_share_directory('aichallenge_launch')
    lgsvl_param_file = os.path.join(
        aichallenge_launch_pkg_prefix, 'param/lgsvl_interface.param.yaml')
    map_publisher_param_file = os.path.join(
        aichallenge_launch_pkg_prefix, 'param/map_publisher.param.yaml')
    lanelet2_map_provider_param_file = os.path.join(
        aichallenge_launch_pkg_prefix, 'param/lanelet2_map_provider.param.yaml')

    urdf_pkg_prefix = get_package_share_directory('lexus_rx_450h_description')
    urdf_path = os.path.join(urdf_pkg_prefix, 'urdf/lexus_rx_450h.urdf')

    # Arguments

    lgsvl_interface_param = DeclareLaunchArgument(
        'lgsvl_interface_param_file',
        default_value=lgsvl_param_file,
        description='Path to config file for LGSVL Interface'
    )
    map_publisher_param = DeclareLaunchArgument(
        'map_publisher_param_file',
        default_value=map_publisher_param_file,
        description='Path to config file for Map Publisher'
    )
    lanelet2_map_provider_param = DeclareLaunchArgument(
        'lanelet2_map_provider_param_file',
        default_value=lanelet2_map_provider_param_file,
        description='Path to parameter file for Lanelet2 Map Provider'
    )

    # Nodes

    lgsvl_interface = Node(
        package='lgsvl_interface',
        node_executable='lgsvl_interface_exe',
        node_namespace='vehicle',
        node_name='lgsvl_interface_node',
        output='screen',
        parameters=[
          LaunchConfiguration('lgsvl_interface_param_file')
        ],
        remappings=[
            ("vehicle_control_cmd", "/lgsvl/vehicle_control_cmd"),
            ("vehicle_state_cmd", "/lgsvl/vehicle_state_cmd"),
            ("state_report", "/lgsvl/state_report"),
            ("state_report_out", "/vehicle/state_report"),
            ("gnss_odom", "/lgsvl/gnss_odom"),
            ("vehicle_odom", "/lgsvl/vehicle_odom")
        ]
    )
    map_publisher = Node(
        package='ndt_nodes',
        node_executable='ndt_map_publisher_exe',
        node_namespace='localization',
        parameters=[
            LaunchConfiguration('map_publisher_param_file'),
            {
                "map_pcd_file": os.path.join(aichallenge_launch_pkg_prefix, "data/IndianapolisMotorSpeedway.pcd"),
                "map_yaml_file": os.path.join(aichallenge_launch_pkg_prefix, "data/IndianapolisMotorSpeedway.yaml"),
                "viz_map": False # Set True if you want to visualize point cloud map.
            }
        ]
    )
    lanelet2_map_provider = Node(
        package='lanelet2_map_provider',
        node_executable='lanelet2_map_provider_exe',
        node_namespace='had_maps',
        node_name='lanelet2_map_provider_node',
        parameters=[
            LaunchConfiguration('lanelet2_map_provider_param_file'),
            {"map_osm_file": os.path.join(aichallenge_launch_pkg_prefix, "data/IndianapolisMotorSpeedway.osm")}
        ]
    )
    lanelet2_map_visualizer = Node(
        package='lanelet2_map_provider',
        node_executable='lanelet2_map_visualizer_exe',
        node_name='lanelet2_map_visualizer_node',
        node_namespace='had_maps'
    )

    urdf_publisher = Node(
        package='robot_state_publisher',
        node_executable='robot_state_publisher',
        node_name='robot_state_publisher',
        arguments=[str(urdf_path)]
    )

    submit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('aichallenge_submit_launch'), '/launch/aichallenge_submit_launch.launch.py']),
        launch_arguments={}.items()
    )
    evaluator = Node(
        package='aichallenge_eval',
        node_executable='aichallenge_eval_node_exe',
        node_name='aichallenge_eval_node',
        node_namespace='aichallenge'
    )
    vehicle_pose_publisher = Node(
        package='vehicle_pose_publisher',
        node_executable='vehicle_pose_publisher_node_exe',
        node_name='vehicle_pose_publisher_node',
        node_namespace='aichallenge'
    )


    return LaunchDescription([
        lgsvl_interface_param,
        map_publisher_param,
        lanelet2_map_provider_param,
        urdf_publisher,
        lgsvl_interface,
        map_publisher,
        lanelet2_map_provider,
        lanelet2_map_visualizer,
        submit_launch,
        evaluator,
        vehicle_pose_publisher
    ])
