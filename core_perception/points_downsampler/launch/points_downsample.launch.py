# Copyright (C) 2023 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.

import ament_index_python
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from carma_ros2_utils.launch.get_current_namespace import GetCurrentNamespace

import os

def generate_launch_description():

    # Declare the log_level launch argument
    #log_level = LaunchConfiguration('log_level')
    #declare_log_level_arg = DeclareLaunchArgument(
    #    name ='log_level', default_value='DEBUG')
        
    # Launch node(s) in a carma container to allow logging to be configured
    container = ComposableNodeContainer(
        package='carma_ros2_utils',
        name='points_downsampler_container',
        namespace=GetCurrentNamespace(),
        executable='carma_component_container_mt',
        composable_node_descriptions=[
            
            # Launch the core node(s)
            ComposableNode(
                    package='points_downsampler',
                    plugin='voxel_grid_filter::VoxelGridFilter',
                    name='voxel_grid_filter_node',
                    extra_arguments=[
                        {'use_intra_process_comms': True},
                    #    {'--log-level' : log_level }
                    ],
                    parameters=[
                    #ament_index_python.get_package_share_directory('points_downsampler') + '/config/interface.yaml',
                    {"points_topic": "points_raw"},  
                    {"output_log": False},
                    {"measurement_range": 200},  
                ],
            ),
        ]
    )

    return LaunchDescription([
        #declare_log_level_arg,
        container
    ])
