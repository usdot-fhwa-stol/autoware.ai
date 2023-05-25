from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

from carma_ros2_utils.launch.get_current_namespace import GetCurrentNamespace
from launch_ros.actions import Node

import os

def generate_launch_description():

    log_level = LaunchConfiguration('log_level')
    declare_log_level_arg = DeclareLaunchArgument(name = 'log_level', default_value='WARN')

    load_type = LaunchConfiguration('load_type')
    declare_load_type = DeclareLaunchArgument(name='load_type', default_value='')
                                                          
    
    lanelet2_filename = LaunchConfiguration('lanelet2_filename')
    declare_lanelet2_filename = DeclareLaunchArgument(name='lanelet2_filename', default_value='/workspaces/carma_ws/install/map_file_ros2/share/map_file_ros2/config/vector_map_UC1_01_19_speed_edited_2.osm')

    lanelet2_map_loader_node = Node(
        package='map_file_ros2',
        namespace='',
        executable='lanelet2_map_loader_exec',
        name = 'lanelet2_map_loader',
        parameters=[
            {'load_type' : load_type},
            {'lanelet2_filename' : lanelet2_filename}
        ]
    )

    lanelet2_map_visualization_node = Node(
        package='map_file_ros2',
        namespace='',
        executable='lanelet2_map_visualization_exec',
        name='lanelet2_map_visualization'
    )
    
    return LaunchDescription([
        declare_log_level_arg,
        declare_load_type,
        declare_lanelet2_filename,
        lanelet2_map_loader_node,
        lanelet2_map_visualization_node
    ])