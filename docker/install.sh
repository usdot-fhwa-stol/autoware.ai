#!/bin/bash

# Copyright (C) 2019-2021 LEIDOS.
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


###
# ROS 1 Build
###
# Source environment variables
if [[ ! -z "$ROS1_PACKAGES$ROS2_PACKAGES" ]]; then
    echo "Sourcing previous build for incremental build start point..."
    source /opt/autoware.ai/ros/install/setup.bash
else
    echo "Sourcing base image for full build..."
    source /home/carma/.base-image/init-env.sh
fi

# Enter source directory
cd /home/carma/autoware.ai

# Build with CUDA
echo "ROS 1 Build with CUDA"
sudo mkdir /opt/autoware.ai # Create install directory
sudo chown carma /opt/autoware.ai # Set owner to expose permissions for build
sudo chgrp carma /opt/autoware.ai # Set group to expose permissions for build

if [[ ! -z "$ROS1_PACKAGES$ROS2_PACKAGES" ]]; then
    if [[ ! -z "$ROS1_PACKAGES" ]]; then
        echo "Incrementally building ROS1 packages: $ROS1_PACKAGES"
        AUTOWARE_COMPILE_WITH_CUDA=1 colcon build --build-base build_ros1 --install-base /opt/autoware.ai/ros/install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs -DCMAKE_CXX_FLAGS=-Wall -DCMAKE_C_FLAGS=-Wall --packages-above $ROS1_PACKAGES
    else
        echo "Build type is incremental but no ROS1 packages specified, skipping ROS1 build..."
    fi
else
    echo "Building all ROS1 Autoware.AI Components"
    AUTOWARE_COMPILE_WITH_CUDA=1 colcon build --build-base build_ros1 --install-base /opt/autoware.ai/ros/install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs -DCMAKE_CXX_FLAGS=-Wall -DCMAKE_C_FLAGS=-Wall
fi

# Get the exit code from the ROS1 build so we can skip the ROS2 build if the ROS1 build failed
status=$?

if [[ $status -ne 0 ]]; then
    echo "Autoware.ai build failed."
    exit $status
fi
echo "Build of ROS1 Autoware.AI Components Complete"


###
# ROS 2 Build
###
source /opt/ros/foxy/setup.bash
if [[ ! -z "$ROS1_PACKAGES$ROS2_PACKAGES" ]]; then
    echo "Sourcing previous build for incremental build start point..."
    source /opt/autoware.ai/ros/install_ros2/setup.bash
else
    echo "Sourcing base image for full build..."
    source /home/carma/catkin/setup.bash
fi

echo "ROS 2 Build"
if [[ ! -z "$ROS1_PACKAGES$ROS2_PACKAGES" ]]; then
    if [[ ! -z "$ROS1_PACKAGES" ]]; then
        echo "Incrementally building ROS2 packages: $ROS1_PACKAGES"
        colcon build --install-base /opt/autoware.ai/ros/install_ros2 --build-base build_ros2 --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-aboe $ROS2_PACKAGES
    else
        echo "Build type is incremental but no ROS2 packages specified, skipping ROS2 build..."
    fi
else
    echo "Building all ROS2 Autoware.AI Components"
    colcon build --install-base /opt/autoware.ai/ros/install_ros2 --build-base build_ros2 --cmake-args -DCMAKE_BUILD_TYPE=Release
fi
