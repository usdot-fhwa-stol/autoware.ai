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
project(ndt_cpu_ros2)

find_package(carma_cmake_common REQUIRED)
carma_check_ros_version(2)
carma_package()

find_package(catkin REQUIRED)
find_package(PCL COMPONENTS REQUIRED)

find_package(Eigen3 QUIET)

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

include_directories(
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  include
)

set(srcs
  src/NormalDistributionsTransform.cpp
  src/Registration.cpp
  src/VoxelGrid.cpp
  src/Octree.cpp
)

set(incs
  include/ndt_cpu_ros2/debug.h
  include/ndt_cpu_ros2/NormalDistributionsTransform.h
  include/ndt_cpu_ros2/Registration.h
  include/ndt_cpu_ros2/SymmetricEigenSolver.h
  include/ndt_cpu_ros2/VoxelGrid.h
  include/ndt_cpu_ros2/Octree.h
)

if(NOT ("${CMAKE_BUILD_TYPE}" STREQUAL "Release"))
  message(WARNING "Not building for release, performance will be slow")

  message(WARNING
  "Adding 'EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT' macro to prevent ndt_matching's runtime error in debug mode.
    The bug reasons and solutions are written in http://eigen.tuxfamily.org/dox-devel/group__TopicUnalignedArrayAssert.html .
    This workaround was discussed on https://gitlab.com/autowarefoundation/autoware.ai/core_perception/merge_requests/57 .")
  add_definitions(-DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT)
endif()

ament_auto_add_library(ndt_cpu_ros2 ${srcs} ${incs})

target_link_libraries(ndt_cpu_ros2
  ${PCL_LIBRARIES}
)

install(DIRECTORY include/
        DESTINATION include
        FILES_MATCHING PATTERN "*.h"
        )

install(TARGETS ndt_cpu_ros2
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_auto_package(
    INSTALL_TO_SHARE config
)
