/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * Modifications:
 * - Added limiting for longitudinal velocity per CARMA specifications. Refactored
 *   namespacing and header as needed to support unit testing.
 *   - Kyle Rush 
 *   - 9/11/2020
 */

#include <ros/ros.h>
#include "twist_filter.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "twist_filter");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  twist_filter::TwistFilter twist_filter(&nh, &pnh);
  ros::spin();
  return 0;
}
