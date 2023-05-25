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

    
    node = Node(
        package='map_file_ros2',
        namespace='',
        executable='lanelet2_map_visualization_exec',
    )
    
    return LaunchDescription([
        declare_log_level_arg,
        node
    ])