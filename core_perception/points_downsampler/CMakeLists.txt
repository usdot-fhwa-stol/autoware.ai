# Copyright (C) 2023 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.

cmake_minimum_required(VERSION 3.5)
project(points_downsampler)

# Declare carma package and check ROS version
find_package(carma_cmake_common REQUIRED)
carma_check_ros_version(2)
carma_package()

## Find dependencies using ament auto
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)


rosidl_generate_interfaces(points_downsampler
  "msg/PointsDownsamplerInfo.msg"
  DEPENDENCIES std_msgs
)

# Includes
include_directories(
  include
)

ament_export_include_directories(include)

###########
## Build ##
###########

ament_auto_add_library(random_filter SHARED
        nodes/random_filter/random_filter.cpp
)
ament_auto_add_library(voxel_grid_filter SHARED
        nodes/voxel_grid_filter/voxel_grid_filter.cpp
)
ament_auto_add_executable(random_filter_exec nodes/random_filter/main.cpp)
ament_auto_add_executable(voxel_grid_filter_exec nodes/voxel_grid_filter/main.cpp)


rosidl_target_interfaces(random_filter ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(voxel_grid_filter ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Register component
rclcpp_components_register_nodes(random_filter "random_filter::RandomFilter")
rclcpp_components_register_nodes(voxel_grid_filter "voxel_grid_filter::VoxelGridFilter")

target_link_libraries(random_filter_exec
  random_filter
)

target_link_libraries(voxel_grid_filter_exec
  voxel_grid_filter
)

ament_auto_package(
  INSTALL_TO_SHARE launch config
)