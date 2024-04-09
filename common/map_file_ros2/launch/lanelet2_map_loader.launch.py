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

    load_type = LaunchConfiguration('load_type')
    declare_load_type = DeclareLaunchArgument(name='load_type', default_value='')
                                                          
    
    lanelet2_filename = LaunchConfiguration('lanelet2_filename')
    declare_lanelet2_filename = DeclareLaunchArgument(name='lanelet2_filename', default_value='')

    lanelet2_map_loader_container = ComposableNodeContainer(
        package='carma_ros2_utils',
        name='lanelet2_map_loader_container',
        executable='carma_component_container_mt',
        namespace=GetCurrentNamespace(),
        composable_node_descriptions=[
            ComposableNode(
                    package='map_file_ros2',
                    plugin='lanelet2_map_loader::Lanelet2MapLoader',
                    name='lanelet2_map_loader',
                    parameters=[ 
                        {'load_type' : load_type},
                        {'lanelet2_filename' : lanelet2_filename} 
                    ]
            ),
            ComposableNode(
                    package='map_file_ros2',
                    plugin='lanelet2_map_visualization::Lanelet2MapVisualization',
                    name='lanelet2_map_visualization'
            )

        ])
    
    return LaunchDescription([
        declare_log_level_arg,
        declare_load_type,
        declare_lanelet2_filename,
        lanelet2_map_loader_container,
    ])