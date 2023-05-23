from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.substitutions import ThisLaunchFileDir
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression

import os


def generate_launch_description():


    load_type = LaunchConfiguration('load_type')
    declare_load_type = DeclareLaunchArgument(name='load_type', default_value='noupdate', description="Enum of the map loading approach to use. Can be 'download', 'noupdate', or 'arealist'")

    single_pcd_path = LaunchConfiguration('single_pcd_path')
    declare_single_pcd_path = DeclareLaunchArgument(name='single_pcd_path', default_value='pcd_map.pcd', description='Path to the map pcd file if using the noupdate load type')

    area = LaunchConfiguration('area')
    declare_area = DeclareLaunchArgument(name='area', default_value='1x1', description='Dimensions of the square of cells loaded at runtime using the download and arealist load types')

    arealist_path = LaunchConfiguration('arealist_path')
    declare_arealist_path = DeclareLaunchArgument(name='arealist_path', default_value='arealist.txt', description='Path to the arealist.txt file which contains the paths and dimensions of each map cell to load with the arealist load_type')

    area_paths=LaunchConfiguration('area_paths')
    declare_area_paths = DeclareLaunchArgument(name='area_paths', default_value='', description='List of cell paths to load when using the arealist load type. If this is not filled all the cells will be loaded based on the arealist.txt file')


    download_launch_group = GroupAction(
        condition = IfCondition(PythonExpression(["'",load_type,"' == 'download'"])),
        actions = [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/launch/points_map_loader.launch.py']),
                launch_arguments = {
                    'load_type' : load_type,
                    'area' : area,
                }.items()
            )
        ]
    )

    noupdate_launch_group = GroupAction(
        condition = IfCondition(PythonExpression(["'",load_type,"' == 'noupdate'"])),
        actions = [
            IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/launch/points_map_loader.launch.py']),
            launch_arguments = {
                        'load_type' : load_type,
                        'pcd_path_parameter' : single_pcd_path,
                    }.items(),
            ),
        ]
    )

    arealist_launch_group = GroupAction(
        condition = IfCondition(PythonExpression(["'",load_type,"' == 'arealist'"])),
        actions = [
            IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/launch/points_map_loader.launch.py']),
            launch_arguments = {
                        'load_type' : load_type,
                        'area' : area,
                        'arealist_path' : arealist_path,
                        'area_paths' : area_paths,
                    }.items(),
            )
        ]
    )

    return LaunchDescription([
        declare_load_type,
        declare_single_pcd_path,
        declare_area,
        declare_arealist_path,
        declare_area_paths,
        download_launch_group,
        noupdate_launch_group,
        arealist_launch_group
    ])