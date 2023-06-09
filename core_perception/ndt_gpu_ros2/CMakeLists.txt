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
project(ndt_gpu_ros2)

find_package(carma_cmake_common REQUIRED)
carma_check_ros_version(2)
carma_package()

find_package(autoware_build_flags REQUIRED COMPONENTS)
find_package(PCL COMPONENTS REQUIRED)
find_package(CUDA)

find_package(Eigen3 QUIET)

if (NOT EIGEN3_FOUND)
    # Fallback to cmake_modules
    find_package(cmake_modules REQUIRED)
    find_package(Eigen REQUIRED)
    set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
    set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
    # Possibly map additional variables to the EIGEN3_ prefix.
else ()
    set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif ()

## Find dependencies using ament auto
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

AW_CHECK_CUDA()

if (USE_CUDA)
    set_directory_properties(PROPERTIES COMPILE_DEFINITIONS "")

    if(CMAKE_CROSSCOMPILING)
        if(NOT CUDA_ARCH)
                message(FATAL_ERROR "Please define the CUDA_ARCH CMake variable")
        endif()
    else()
        if (NOT DEFINED CUDA_CAPABILITY_VERSION_CHECKER)
            set(CUDA_CAPABILITY_VERSION_CHECKER
                    "${CATKIN_DEVEL_PREFIX}/lib/capability_version_checker")
        endif ()

        execute_process(COMMAND ${CUDA_CAPABILITY_VERSION_CHECKER}
                OUTPUT_VARIABLE CUDA_CAPABILITY_VERSION
                OUTPUT_STRIP_TRAILING_WHITESPACE)

        if ("${CUDA_CAPABILITY_VERSION}" MATCHES "^[1-9][0-9]+$")
            set(CUDA_ARCH "sm_${CUDA_CAPABILITY_VERSION}")
        else ()
            set(CUDA_ARCH "sm_52")
        endif ()
    endif()

    set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-arch=${CUDA_ARCH};-std=c++14;--ptxas-options=-v)

    set(SUBSYS_DESC "Point cloud ndt gpu library")


    include_directories(
            ${PCL_INCLUDE_DIRS}
            ${catkin_INCLUDE_DIRS}
            ${CUDA_INCLUDE_DIRS}
            include
            ${EIGEN3_INCLUDE_DIRS}
    )

    set(srcs
            src/MatrixDevice.cu
            src/MatrixHost.cu
            src/NormalDistributionsTransform.cu
            src/Registration.cu
            src/VoxelGrid.cu
            src/SymmetricEigenSolver.cu
            )

    set(incs
            include/ndt_gpu_ros2/common.h
            include/ndt_gpu_ros2/debug.h
            include/ndt_gpu_ros2/Matrix.h
            include/ndt_gpu_ros2/MatrixDevice.h
            include/ndt_gpu_ros2/MatrixHost.h
            include/ndt_gpu_ros2/NormalDistributionsTransform.h
            include/ndt_gpu_ros2/Registration.h
            include/ndt_gpu_ros2/SymmetricEigenSolver.h
            include/ndt_gpu_ros2/VoxelGrid.h
            )

    cuda_add_library(ndt_gpu_ros2 ${srcs} ${incs})

    target_link_libraries(ndt_gpu_ros2
            ${CUDA_LIBRARIES}
            ${CUDA_CUBLAS_LIBRARIES}
            ${CUDA_curand_LIBRARY}
            ${PCL_LIBRARIES}
            )

    install(DIRECTORY include/
            DESTINATION include
            FILES_MATCHING PATTERN "*.h"
            )


    install(TARGETS ndt_gpu_ros2
            ARCHIVE DESTINATION lib
            LIBRARY DESTINATION lib
            RUNTIME DESTINATION bin
            )

else ()
    message("ndt_gpu_ros2 will not be built, CUDA was not found.")
endif ()

ament_auto_package(
    INSTALL_TO_SHARE config
)