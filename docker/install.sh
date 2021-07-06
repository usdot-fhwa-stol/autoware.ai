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

# Source environment variables
source /home/carma/.base-image/init-env.sh

# Enter source directory
cd /home/carma/autoware.ai

# Build with CUDA
echo "Build with CUDA"
sudo mkdir /opt/autoware.ai # Create install directory
sudo chown carma /opt/autoware.ai # Set owner to expose permissions for build
sudo chgrp carma /opt/autoware.ai # Set group to expose permissions for build
AUTOWARE_COMPILE_WITH_CUDA=1 colcon build --install-base /opt/autoware.ai/ros/install --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs -DCMAKE_CXX_FLAGS=-Wall -DCMAKE_C_FLAGS=-Wall
