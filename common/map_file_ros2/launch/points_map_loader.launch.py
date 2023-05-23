from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from carma_ros2_utils.launch.get_current_namespace import GetCurrentNamespace

import os

def generate_launch_description():

    log_level = LaunchConfiguration('log_level')
    declare_log_level_arg = DeclareLaunchArgument(name = 'log_level', default_value='WARN')

    points_map_loader_param_file = os.path.join(get_package_share_directory('map_file_ros2'), 'config/parameters.yaml')

    container = ComposableNodeContainer(
        package='carma_ros2_utils',
        name='map_file_nodes_container',
        namespace=GetCurrentNamespace(),
        executable='carma_component_container_mt',
        composable_node_descriptions=[

            ComposableNode(
                package='map_file_ros2',
                plugin='points_map_loader::PointsMapLoader',
                name='point_map_loader_node',
                extra_arguments=[
                    {'use_intra_process_comms': True},
                    {'log_level':log_level}
                ],
                parameters=[points_map_loader_param_file]
            ),
        ]
    )
    
    return LaunchDescription([
        declare_log_level_arg,
        container
    ])