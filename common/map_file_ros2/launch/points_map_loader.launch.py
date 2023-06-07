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

    update_rate = LaunchConfiguration('update_rate')
    declare_update_rate = DeclareLaunchArgument(name='update_rate', default_value='1000')

    area = LaunchConfiguration('area')
    declare_area = DeclareLaunchArgument(name='area', default_value="")

    load_type = LaunchConfiguration('load_type')
    declare_load_type = DeclareLaunchArgument(name='load_type', default_value="")

    path_area_list = LaunchConfiguration('path_area_list')
    declare_path_area_list = DeclareLaunchArgument(name='path_area_list', default_value="")

    pcd_path_parameter = LaunchConfiguration('pcd_path_parameter')
    declare_pcd_path_parameter = DeclareLaunchArgument(name='pcd_path_parameter', default_value='["",""]')

    host_name = LaunchConfiguration('host_name')
    declare_host_name = DeclareLaunchArgument(name='host_name', default_value="133.6.148.90")

    port = LaunchConfiguration('port')
    declare_port = DeclareLaunchArgument(name='port', default_value='80')

    user = LaunchConfiguration('user')
    declare_user = DeclareLaunchArgument(name='user', default_value="")

    password = LaunchConfiguration('password')
    declare_password = DeclareLaunchArgument(name='password', default_value="")

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
                parameters=[
                    {'update_rate': update_rate},
                    {'area': area},
                    {'load_type': load_type},
                    {'path_area_list': path_area_list},
                    {'pcd_path_parameter': pcd_path_parameter},
                    {'host_name': host_name},
                    {'port' : port},
                    {'user': user},
                    {'password': password}
                    ]
            ),
        ]
    )
    
    return LaunchDescription([
        declare_log_level_arg,
        declare_update_rate,
        declare_area,
        declare_load_type,
        declare_path_area_list,
        declare_pcd_path_parameter,
        declare_host_name,
        declare_port,
        declare_user,
        declare_password,
        container
    ])