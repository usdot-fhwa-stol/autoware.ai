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
project(map_file_ros2)

# Declare carma package and check ROS version
find_package(carma_cmake_common REQUIRED)
carma_check_ros_version(2)
carma_package()

find_package(PCL COMPONENTS REQUIRED)
find_package(CURL REQUIRED)

## Find dependencies using ament auto
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()


# Includes
include_directories(
    include
    ${PCL_INCLUDE_DIRS}
    ${CURL_INCLUDE_DIRS}
)

# Build
ament_auto_add_library(get_file SHARED
    lib/map_file/get_file.cpp
)
target_link_libraries(get_file ${CURL_LIBRARIES})

ament_auto_add_library(points_map_loader_lib SHARED
    nodes/points_map_loader/points_map_loader.cpp
)
ament_auto_add_executable(points_map_loader_exec
    nodes/points_map_loader/points_map_loader_node.cpp
)

ament_auto_add_library(map_param_loader_lib SHARED
    nodes/map_param_loader/map_param_loader.cpp
)
ament_auto_add_executable(map_param_loader_exec
    nodes/map_param_loader/map_param_loader_node.cpp
)

ament_auto_add_library(lanelet2_map_loader_lib SHARED
    nodes/lanelet2_map_loader/lanelet2_map_loader.cpp
)
ament_auto_add_executable(lanelet2_map_loader_exec
    nodes/lanelet2_map_loader/lanelet2_map_loader_node.cpp
)

ament_auto_add_library(lanelet2_map_visualization_lib SHARED
    nodes/lanelet2_map_visualization/lanelet2_map_visualization.cpp
)
ament_auto_add_executable(lanelet2_map_visualization_exec
    nodes/lanelet2_map_visualization/lanelet2_map_visualization_node.cpp
)

# Register component
rclcpp_components_register_nodes(points_map_loader_lib "points_map_loader::PointsMapLoader")

rclcpp_components_register_nodes(map_param_loader_lib "map_param_loader::MapParamLoader")

rclcpp_components_register_nodes(lanelet2_map_loader_lib "lanelet2_map_loader::Lanelet2MapLoader")

rclcpp_components_register_nodes(lanelet2_map_visualization_lib "lanelet2_map_visualization::Lanelet2MapVisualization")

# All locally created targets will need to be manually linked
# ament auto will handle linking of external dependencies
target_link_libraries(points_map_loader_lib get_file ${PCL_LIBRARIES})

target_link_libraries(
    points_map_loader_exec
    points_map_loader_lib
)

target_link_libraries(
    map_param_loader_exec
    map_param_loader_lib
)

target_link_libraries(
    lanelet2_map_loader_exec
    lanelet2_map_loader_lib
)

target_link_libraries(
    lanelet2_map_visualization_exec
    lanelet2_map_visualization_lib
)

# Testing
if(BUILD_TESTING)  

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies() # This populates the ${${PROJECT_NAME}_FOUND_TEST_DEPENDS} variable

  ament_add_gtest(test_route 
        test/test_get_transform.cpp
  )

  ament_target_dependencies(test_route ${${PROJECT_NAME}_FOUND_TEST_DEPENDS})

  target_link_libraries(test_route map_param_loader_lib)

endif()

ament_auto_package(
    INSTALL_TO_SHARE config launch
)