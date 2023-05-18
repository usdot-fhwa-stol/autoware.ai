from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import os

def generate_launch_description():

    log_level = LaunchConfiguration('log_level')
    declare_log_level_arg = DeclareLaunchArgument(name = 'log_level', default_value='WARN')

    points_map_loader_param_file = os.path.join(get_package_share_directory('map_file_ros2'), 'config/parameters.yaml')

    pcd_map_loader_node = Node(
            package='map_file_ros2',
            #namespace='points_map_loader',
            executable='points_map_loader_exec',
            name='points_map_loader',
            parameters=[{'log_level':log_level},
                points_map_loader_param_file]
        )

    return LaunchDescription([
        declare_log_level_arg,
        pcd_map_loader_node
    ])