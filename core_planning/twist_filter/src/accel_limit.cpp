/*
 * Copyright (C) 2018-2021 LEIDOS.
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

#include "accel_limiter.hpp"
#include <limits>
#include <boost/math/special_functions/sign.hpp>

namespace twist_filter {

autoware_msgs::ControlCommandStamped
LongitudinalAccelLimiter::longitudinalAccelLimitCtrl(const autoware_msgs::ControlCommandStamped& msg) {
    if (!_initialized) {
        // Record current command and then return unmodified
        _prev_v = msg.cmd.linear_velocity;
        _prev_t = msg.header.stamp;
        _initialized = true;

        return msg;
    }

    autoware_msgs::ControlCommandStamped out{msg};
    // Else we've initialized previously, let's compare.
    double delta_v = msg.cmd.linear_velocity - _prev_v;
    double delta_t = (msg.header.stamp - _prev_t).toSec();
    double accel = delta_v / delta_t;

    if (std::abs(accel) > _accel_limit) {
        out.cmd.linear_acceleration = _accel_limit * boost::math::sign<double>(accel);
        out.cmd.linear_velocity = _prev_v + (_accel_limit * delta_t * boost::math::sign<double>(accel));
    }

    // Update filter for next timestep
    _prev_v = out.cmd.linear_velocity;
    _prev_t = out.header.stamp;

    return out;
}

geometry_msgs::TwistStamped
LongitudinalAccelLimiter::longitudinalAccelLimitTwist(const geometry_msgs::TwistStamped& msg) {
    if (!_initialized) {
        // Record current command and then return unmodified
        _prev_v = msg.twist.linear.x;
        _prev_t = msg.header.stamp;
        _initialized = true;

        return msg;
    }

    geometry_msgs::TwistStamped out{msg};
    // Else we've initialized previously, let's compare.
    double delta_v = msg.twist.linear.x - _prev_v;
    double delta_t = (msg.header.stamp - _prev_t).toSec();
    double accel = delta_v / delta_t;

    if (std::abs(accel) > _accel_limit) {
        out.twist.linear.x = _prev_v + (_accel_limit * delta_t * boost::math::sign<double>(accel));
    }

    // Update filter for next timestep
    _prev_v = out.twist.linear.x;
    _prev_t = out.header.stamp;

    return out;
}

}