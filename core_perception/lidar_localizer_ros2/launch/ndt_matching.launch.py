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

    method_type = LaunchConfiguration('method_type')
    declare_method_type = DeclareLaunchArgument(name='method_type', default_value=0)

    use_gnss = LaunchConfiguration('use_gnss')
    declare_use_gnss = DeclareLaunchArgument(name='use_gnss', default_value=1)

    use_odom = LaunchConfiguration('use_odom')
    declare_use_odom = DeclareLaunchArgument(name='use_odom', default_value="false")

    use_imu = LaunchConfiguration('use_imu')
    declare_use_imu = DeclareLaunchArgument(name='use_imu', default_value="false")

    imu_upside_down = LaunchConfiguration('imu_upside_down')
    declare_imu_upside_down = DeclareLaunchArgument(name='imu_upside_down', default_value='false')

    imu_topic = LaunchConfiguration('imu_topic')
    declare_imu_topic = DeclareLaunchArgument(name='imu_topic', default_value="/imu_raw")

    queue_size = LaunchConfiguration('queue_size')
    declare_queue_size = DeclareLaunchArgument(name='queue_size', default_value=1)

    offset = LaunchConfiguration('offset')
    declare_offset = DeclareLaunchArgument(name='offset', default_value="linear")

    get_height = LaunchConfiguration('get_height')
    declare_get_height = DeclareLaunchArgument(name='get_height', default_value="false")

    use_local_transform = LaunchConfiguration('use_local_transform')
    declare_use_local_transform = DeclareLaunchArgument(name='use_local_transform', default_value="false")

    sync = LaunchConfiguration('sync')
    declare_sync = DeclareLaunchArgument(name='sync', default_value="false")

    output_log_data = LaunchConfiguration('output_log_data')
    declare_output_log_data = DeclareLaunchArgument(name='output_log_data', default_value="false")

    gnss_reinit_fitness = LaunchConfiguration('gnss_reinit_fitness')
    declare_gnss_reinit_fitness = DeclareLaunchArgument(name='gnss_reinit_fitness', default_value="500.0")

    base_frame = LaunchConfiguration('base_frame')
    declare_base_frame = DeclareLaunchArgument(name='base_frame', default_value="base_link")


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
                    {'method_type': method_type},
                    {'use_gnss': use_gnss},
                    {'use_odom': use_odom},
                    {'use_imu': use_imu},
                    {'imu_upside_down': imu_upside_down},
                    {'imu_topic': imu_topic},
                    {'queue_size' : queue_size},
                    {'offset': offset},
                    {'get_height': get_height},
                    {'use_local_transform': use_local_transform},
                    {'sync': sync},
                    {'output_log_data': output_log_data},
                    {'gnss_reinit_fitness': gnss_reinit_fitness},
                    {'base_frame': base_frame}
                    ]
            ),
        ]
    )
    
    return LaunchDescription([
        declare_log_level_arg,
        declare_method_type,
        declare_use_gnss,
        declare_use_odom,
        declare_use_imu,
        declare_imu_upside_down,
        declare_imu_topic,
        declare_queue_size,
        declare_offset,
        declare_get_height,
        declare_use_local_transform,
        declare_sync,
        declare_output_log_data,
        declare_gnss_reinit_fitness,
        declare_base_frame,
        container
    ])