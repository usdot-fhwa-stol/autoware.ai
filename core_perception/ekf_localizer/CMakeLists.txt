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
project(ekf_localizer)

# Declare carma package and check ROS version
find_package(carma_cmake_common REQUIRED)
carma_check_ros_version(2)
carma_package()

find_package(Eigen3 REQUIRED)

## Find dependencies using ament auto
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Includes
include_directories(
    include
    ${EIGEN3_INCLUDE_DIR}
)

ament_auto_add_library(ekf_localizer_lib SHARED
    src/ekf_localizer.cpp)

ament_auto_add_executable(ekf_localizer_exec
    src/ekf_localizer_node.cpp)

# Register component
rclcpp_components_register_nodes(ekf_localizer_lib "ekf_localizer::EKFLocalizer")

target_link_libraries(
    ekf_localizer_exec
    ekf_localizer_lib
)

# Testing
if(BUILD_TESTING)  

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies() # This populates the ${${PROJECT_NAME}_FOUND_TEST_DEPENDS} variable

  ament_add_gtest(test_ekf_localizer test/test_ekf_localizer.cpp)

  ament_target_dependencies(test_ekf_localizer ${${PROJECT_NAME}_FOUND_TEST_DEPENDS})

  target_link_libraries(test_ekf_localizer ekf_localizer_lib)

endif()

ament_auto_package(
    INSTALL_TO_SHARE launch
)
