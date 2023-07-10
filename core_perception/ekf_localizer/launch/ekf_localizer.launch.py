# Copyright (C) <SUB><year> LEIDOS.
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
This file is can be used to launch the CARMA <SUB><package_name>_node.
  Though in carma-platform it may be launched directly from the base launch file.
'''

def generate_launch_description():

    # Declare the log_level launch argument
    log_level = LaunchConfiguration('log_level')
    declare_log_level_arg = DeclareLaunchArgument(
        name ='log_level', default_value='WARN')
    
    show_debug_info = LaunchConfiguration('show_debug_info')
    declare_show_debug_info = DeclareLaunchArgument(name='show_debug_info', default_value = "False")

    predict_frequency = LaunchConfiguration('predict_frequency')
    declare_predict_frequency = DeclareLaunchArgument(name='predict_frequency', default_value = '50.0')

    enable_yaw_bias_estimation = LaunchConfiguration('enable_yaw_bias_estimation')
    declare_enable_yaw_bias_estimation = DeclareLaunchArgument(name='enable_yaw_bias_estimation', default_value = "True")

    extend_state_step = LaunchConfiguration('extend_state_step')
    declare_extend_state_step = DeclareLaunchArgument(name='extend_state_step', default_value = '50')

    pose_frame_id = LaunchConfiguration('pose_frame_id')
    declare_pose_frame_id = DeclareLaunchArgument(name='pose_frame_id', default_value='/map')

    child_frame_id = LaunchConfiguration('child_frame_id')
    declare_child_frame_id = DeclareLaunchArgument(name='child_frame_id', default_value ='base_link')

    pose_additional_delay = LaunchConfiguration('pose_additional_delay')
    declare_pose_additional_delay = DeclareLaunchArgument(name='pose_additional_delay', default_value = '0.0')

    pose_measure_uncertainty_time = LaunchConfiguration('pose_measure_uncertainty_time')
    declare_pose_measure_uncertainty_time = DeclareLaunchArgument(name='pose_measure_uncertainty_time', default_value = '0.01')

    pose_rate = LaunchConfiguration('pose_rate')
    declare_pose_rate = DeclareLaunchArgument(name='pose_rate', default_value = '10.0')

    pose_gate_dist = LaunchConfiguration('pose_gate_dist')
    declare_pose_gate_dist = DeclareLaunchArgument(name='pose_gate_dist', default_value = '10000.0')

    pose_stddev_x = LaunchConfiguration('pose_stddev_x')
    declare_pose_stddev_x = DeclareLaunchArgument(name='pose_stddev_x', default_value= '0.05')

    pose_stddev_y = LaunchConfiguration('pose_stddev_y')
    declare_pose_stddev_y = DeclareLaunchArgument(name='pose_stddev_y', default_value= '0.05')

    pose_stddev_yaw = LaunchConfiguration('pose_stddev_yaw')
    declare_pose_stddev_yaw = DeclareLaunchArgument(name='pose_stddev_yaw', default_value = '0.035')

    use_pose_with_covariance = LaunchConfiguration('use_pose_with_covariance')
    declare_use_pose_with_covariance = DeclareLaunchArgument(name='use_pose_with_covariance', default_value="False")

    twist_additional_delay = LaunchConfiguration('twist_additional_delay')
    declare_twist_additional_delay = DeclareLaunchArgument(name='twist_additional_delay', default_value='0.0')

    twist_rate = LaunchConfiguration('twist_rate')
    declare_twist_rate = DeclareLaunchArgument(name='twist_rate', default_value='10.0')

    twist_gate_dist = LaunchConfiguration('twist_gate_dist')
    declare_twist_gate_dist = DeclareLaunchArgument(name = 'twist_gate_dist', default_value = '10000.0')

    twist_stddev_vx = LaunchConfiguration('twist_stddev_vx')
    declare_twist_stddev_vx = DeclareLaunchArgument(name='twist_stddev_vx', default_value = '0.2')

    twist_stddev_wz = LaunchConfiguration('twist_stddev_wz')
    declare_twist_stddev_wz = DeclareLaunchArgument(name='twist_stddev_wz', default_value='0.03')

    use_twist_with_covariance = LaunchConfiguration('use_twist_with_covariance')
    declare_use_twist_with_covariance = DeclareLaunchArgument(name='use_twist_with_covariance', default_value="False")

    proc_stddev_yaw_c = LaunchConfiguration('proc_stddev_yaw_c')
    declare_proc_stddev_yaw_c = DeclareLaunchArgument(name='proc_stddev_yaw_c', default_value='0.005')

    proc_stddev_yaw_bias_c = LaunchConfiguration('proc_stddev_yaw_bias_c')
    declare_proc_stddev_yaw_bias_c = DeclareLaunchArgument(name='proc_stddev_yaw_bias_c', default_value = '0.001')

    proc_stddev_vx_c = LaunchConfiguration('proc_stddev_vx_c')
    declare_proc_stddev_vx_c = DeclareLaunchArgument(name='proc_stddev_vx_c', default_value = '2.0')

    proc_stddev_wz_c = LaunchConfiguration('proc_stddev_wz_c')
    declare_proc_stddev_wz_c = DeclareLaunchArgument(name='proc_stddev_wz_c', default_value='0.2')
        
    # Launch node(s) in a carma container to allow logging to be configured
    container = ComposableNodeContainer(
        package='carma_ros2_utils',
        name='ekf_localizer_container',
        namespace=GetCurrentNamespace(),
        executable='carma_component_container_mt',
        composable_node_descriptions=[
            
            # Launch the core node(s)
            ComposableNode(
                    package='ekf_localizer',
                    plugin='ekf_localizer::EKFLocalizer',
                    name='ekf_localizer_node',
                    extra_arguments=[
                        {'use_intra_process_comms': True},
                        {'--log-level' : log_level }
                    ],
                    parameters=[  
                        {'show_debug_info': show_debug_info},
                        {'predict_frequency': predict_frequency},
                        {'enable_yaw_bias_estimation': enable_yaw_bias_estimation},
                        {'extend_state_step': extend_state_step},
                        {'pose_frame_id': pose_frame_id},
                        {'child_frame_id': child_frame_id},
                        {'pose_additional_delay': pose_additional_delay},
                        {'pose_measure_uncertainty_time': pose_measure_uncertainty_time},
                        {'pose_rate': pose_rate},
                        {'pose_gate_dist': pose_gate_dist},
                        {'pose_stddev_x': pose_stddev_x},
                        {'pose_stddev_y': pose_stddev_y},
                        {'pose_stddev_yaw': pose_stddev_yaw},
                        {'use_pose_with_covariance': use_pose_with_covariance},
                        {'twist_additional_delay': twist_additional_delay},
                        {'twist_rate': twist_rate},
                        {'twist_gate_dist': twist_gate_dist},
                        {'twist_stddev_vx': twist_stddev_vx},
                        {'twist_stddev_wz': twist_stddev_wz},
                        {'use_twist_with_covariance': use_twist_with_covariance},
                        {'proc_stddev_yaw_c': proc_stddev_yaw_c},
                        {'proc_stddev_yaw_bias_c': proc_stddev_yaw_bias_c},
                        {'proc_stddev_vx_c': proc_stddev_vx_c},
                        {'proc_stddev_wz_c': proc_stddev_wz_c}
                    ]
            ),
        ]
    )

    return LaunchDescription([
        declare_log_level_arg,
        declare_show_debug_info,
        declare_predict_frequency,
        declare_enable_yaw_bias_estimation,
        declare_extend_state_step,
        declare_pose_frame_id,
        declare_child_frame_id,
        declare_pose_additional_delay,
        declare_pose_measure_uncertainty_time,
        declare_pose_rate,
        declare_pose_gate_dist,
        declare_pose_stddev_x,
        declare_pose_stddev_y,
        declare_pose_stddev_yaw,
        declare_use_pose_with_covariance,
        declare_twist_additional_delay,
        declare_twist_rate,
        declare_twist_gate_dist,
        declare_twist_stddev_vx,
        declare_twist_stddev_wz,
        declare_use_twist_with_covariance,
        declare_proc_stddev_yaw_c,
        declare_proc_stddev_yaw_bias_c,
        declare_proc_stddev_vx_c,
        declare_proc_stddev_wz_c,
        container
    ])