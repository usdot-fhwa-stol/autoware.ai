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
project(lidar_localizer_ros2)

# Declare carma package and check ROS version
find_package(carma_cmake_common REQUIRED)
carma_check_ros_version(2)
carma_package()

find_package(autoware_build_flags REQUIRED)
find_package(PCL COMPONENTS REQUIRED)
find_package(CUDA)
find_package(Eigen3 QUIET)

AW_CHECK_CUDA()

if(USE_CUDA)
  add_definitions(-DCUDA_FOUND)
  list(APPEND PCL_OPENMP_PACKAGES ndt_gpu_ros2)
endif()

if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
  # Possibly map additional variables to the EIGEN3_ prefix.
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()


## Find dependencies using ament auto
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()



# Includes
include_directories(
  ${PCL_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  include
  ${EIGEN3_INCLUDE_DIRS}
)

ament_auto_add_library(ndt_matching_lib SHARED
  src/ndt_matching.cpp)

rclcpp_components_register_nodes(ndt_matching_lib "ndt_matching::NDTMatching")

target_link_libraries(ndt_matching_lib
${PCL_LIBRARIES}
)




ament_auto_package(
  INSTALL_TO_SHARE launch
)