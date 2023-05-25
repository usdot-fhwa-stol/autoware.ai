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

    broadcast_earth_frame = LaunchConfiguration('broadcast_earth_frame')
    declare_broadcast_earth_frame = DeclareLaunchArgument(name='broadcast_earth_frame', default_value='false', 
                                                          description='If true this node will attempt to compute a homogenious transformation matrix between earth and the map frame origin. NOTE: Not all projections can be linearly described in this way and such a transformation may be inaccurate')
    
    file_name = LaunchConfiguration('file_name')
    declare_file_name = DeclareLaunchArgument(name='file_name', default_value='')

    node = Node(
        package='map_file_ros2',
        namespace='',
        executable='map_param_loader_exec',
        parameters=[
            {'broadcast_earth_frame' : broadcast_earth_frame},
            {'file_name' : '/workspaces/carma_ws/install/map_file_ros2/share/map_file_ros2/config/vector_map_UC1_01_19_speed_edited_2.osm'}
        ]
    )
    
    return LaunchDescription([
        declare_log_level_arg,
        declare_broadcast_earth_frame,
        declare_file_name,
        node
    ])