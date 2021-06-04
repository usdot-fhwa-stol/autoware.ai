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

source /home/carma/.base-image/init-env.sh
autoware_src="/home/carma/autoware.ai"
cd ${autoware_src}

# build autoware messages first (noetic has it's own version)
colcon build --packages-up-to autoware_msgs --cmake-args "-DCMAKE_BUILD_TYPE=Debug" --executor sequential --install-base ./ros/install
source ./ros/install/setup.bash

# create colcon ignore file 
touch messages/autoware_msgs/COLCON_IGNORE

#build the rest of autoware
./autoware/ros/carma_autoware_build -a ${autoware_src}
