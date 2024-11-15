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

if [[ ! -z "$ROS1_PACKAGES$ROS2_PACKAGES" ]]; then
    echo "Sourcing previous build for incremental build start point..."
    source /opt/autoware.ai/ros/install/setup.bash
else
    echo "Sourcing base image for full build..."
    source /home/carma/.base-image/init-env.sh
fi

# Enter source directory
cd /home/carma/autoware.ai

sudo mkdir /opt/autoware.ai # Create install directory
sudo chown carma /opt/autoware.ai # Set owner to expose permissions for build
sudo chgrp carma /opt/autoware.ai # Set group to expose permissions for build

echo "ROS 2 Build"
if [[ ! -z "$ROS1_PACKAGES$ROS2_PACKAGES" ]]; then
    if [[ ! -z "$ROS2_PACKAGES" ]]; then
        echo "Incrementally building ROS2 packages: $ROS1_PACKAGES"
        # Build with CUDA compile option 
        AUTOWARE_COMPILE_WITH_CUDA=1 colcon build --install-base /opt/autoware.ai/ros/install --build-base build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-above $ROS2_PACKAGES --allow-overriding $ROS2_PACKAGES --packages-ignore parking_planner parking_planner_nodes
    else
        echo "Build type is incremental but no ROS2 packages specified, skipping ROS2 build..."
    fi
else
    echo "Building all ROS2 Autoware.AI Components"
    # Build with CUDA compile option
    AUTOWARE_COMPILE_WITH_CUDA=1 colcon build --install-base /opt/autoware.ai/ros/install --build-base build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-ignore parking_planner parking_planner_nodes
fi
