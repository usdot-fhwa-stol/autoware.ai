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
source /home/carma/.base-image/init-env.sh

# Enter source directory
cd /home/carma/autoware.ai

# Build with CUDA
echo "ROS 1 Build with CUDA"
sudo mkdir /opt/autoware.ai # Create install directory
sudo chown carma /opt/autoware.ai # Set owner to expose permissions for build
sudo chgrp carma /opt/autoware.ai # Set group to expose permissions for build
AUTOWARE_COMPILE_WITH_CUDA=1 colcon build --build-base build_ros1 --install-base /opt/autoware.ai/ros/install --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs -DCMAKE_CXX_FLAGS=-Wall -DCMAKE_C_FLAGS=-Wall

# Get the exit code from the ROS1 build so we can skip the ROS2 build if the ROS1 build failed
status=$?

if [[ $status -ne 0 ]]; then
    echo "Autoware.ai build failed."
    exit $status
fi


###
# ROS 2 Build
###
source /opt/ros/foxy/setup.bash
source /home/carma/catkin/setup.bash

echo "ROS 2 Build"
colcon build --install-base /opt/autoware.ai/ros/install_ros2 --build-base build_ros2 --cmake-args -DCMAKE_BUILD_TYPE=Release
