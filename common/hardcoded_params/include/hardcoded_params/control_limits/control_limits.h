#pragma once

/*
 * Copyright (C) 2020-2021 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

namespace hardcoded_params
{
namespace control_limits
{
/**
 * The maximum allowable longitudinal velocity of the vehicle in m/s
 */
constexpr double MAX_LONGITUDINAL_VELOCITY_MPS = 35.7632;
constexpr double MAX_LONGITUDINAL_ACCEL_MPS2 = 3.5;

/**
* The maximum allowable simulation parameters
* At the time of writing this, platform uses CARLA 0.9.10 as the vehicle's physics simulator.
* CARLA 0.9.10 uses below hardcoded value for its vehicles (so even if increasing this value,
* actual simulation won't support larger values unless forked and compiled)
* https://github.com/carla-simulator/ros-bridge/blob/0.9.10.1/carla_ackermann_control/src/carla_ackermann_control/carla_control_physics.py#L254
*/
constexpr double MAX_SIMULATION_LONGITUDINAL_ACCEL_MPS2 = 8.0;

}
}  // namespace hardcoded_params
