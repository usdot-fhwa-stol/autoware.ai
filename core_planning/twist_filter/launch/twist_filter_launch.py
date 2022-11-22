# Copyright (C) 2022 LEIDOS.
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


from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from carma_ros2_utils.launch.get_current_namespace import GetCurrentNamespace

import os


'''
This file is can be used to launch the twist_filter_node.
  Though in carma-platform it may be launched directly from the base launch file.
'''

def generate_launch_description():

    # Declare the log_level launch argument
    log_level = LaunchConfiguration('log_level')
    declare_log_level_arg = DeclareLaunchArgument(name ='log_level', default_value='WARN')

    loop_rate = LaunchConfiguration('loop_rate')
    declare_loop_rate = DeclareLaunchArgument(name='loop_rate', default_value='30.0')

    use_decision_maker = LaunchConfiguration('use_decision_maker')
    declare_use_decision_maker = DeclareLaunchArgument(name='use_decision_maker', default_value='False')

    lateral_accel_limit = LaunchConfiguration('lateral_accel_limit')
    declare_lateral_accel_limit = DeclareLaunchArgument(name = 'lateral_accel_limit', default_value='5.0')

    lateral_jerk_limit = LaunchConfiguration('lateral_jerk_limit')
    declare_lateral_jerk_limit = DeclareLaunchArgument(name = 'lateral_jerk_limit', default_value='5.0')

    lowpass_gain_linear_x = LaunchConfiguration('lowpass_gain_linear_x')
    declare_lowpass_gain_linear_x = DeclareLaunchArgument(name = 'lowpass_gain_linear_x', default_value='0.0')

    lowpass_gain_angular_z = LaunchConfiguration('lowpass_gain_angular_z')
    declare_lowpass_gain_angular_z = DeclareLaunchArgument(name='lowpass_gain_angular_z', default_value='0.0')

    lowpass_gain_steering_angle = LaunchConfiguration('lowpass_gain_steering_angle')
    declare_lowpass_gain_steering_angle = DeclareLaunchArgument(name='lowpass_gain_steering_angle', default_value='0.0')

    longitudinal_velocity_limit = LaunchConfiguration('longitudinal_velocity_limit')
    declare_longitudinal_velocity_limit = DeclareLaunchArgument(name = 'longitudinal_velocity_limit', default_value='80.0') #mph

    longitudinal_accel_limit = LaunchConfiguration('longitudinal_accel_limit')
    declare_longitudinal_accel_limit = DeclareLaunchArgument(name= 'longitudinal_accel_limit', default_value='3.5')

        
    # Launch node(s) in a carma container to allow logging to be configured
    container = ComposableNodeContainer(
        package='carma_ros2_utils',
        name='twist_filter_container',
        namespace=GetCurrentNamespace(),
        executable='carma_component_container_mt',
        composable_node_descriptions=[
            
            # Launch the core node(s)
            ComposableNode(
                    package='twist_filter',
                    plugin='twist_filter::TwistFilter',
                    name='twist_filter_node',
                    extra_arguments=[
                        {'use_intra_process_comms': True},
                        {'--log-level' : log_level },
                    ],
                    parameters = [
                        {'lateral_accel_limit' : lateral_accel_limit},
                        {'lateral_jerk_limit' : lateral_jerk_limit},
                        {'lowpass_gain_linear_x' : lowpass_gain_linear_x},
                        {'lowpass_gain_angular_z' : lowpass_gain_angular_z},
                        {'lowpass_gain_steering_angle' : lowpass_gain_steering_angle},
                        {'longitudinal_velocity_limit' : longitudinal_velocity_limit},
                        {'longitudinal_accel_limit' : longitudinal_accel_limit}
                    ]
            ),
            ComposableNode(
                package='twist_gate',
                plugin='TwistGate',
                name='twist_gate_node',
                extra_arguments=[
                    {'use_intra_process_comms': True},
                    {'--log-level' : log_level },
                ],
                parameters = [
                    {'loop_rate': loop_rate},
                    {'use_decision_maker' : use_decision_maker}
                ]
            )
        ]
    )

    return LaunchDescription([
        declare_log_level_arg,
        declare_loop_rate,
        declare_use_decision_maker,
        declare_lateral_accel_limit,
        declare_lateral_jerk_limit,
        declare_lowpass_gain_linear_x,
        declare_lowpass_gain_angular_z,
        declare_lowpass_gain_steering_angle,
        declare_longitudinal_velocity_limit,
        declare_longitudinal_accel_limit,
        container
    ])