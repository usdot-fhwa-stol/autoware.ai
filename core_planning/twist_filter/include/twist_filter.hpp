
/*------------------------------------------------------------------------------
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

------------------------------------------------------------------------------*/

#pragma once
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

#include <iostream>
#include <boost/optional.hpp>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/TwistStamped.h>
#include <autoware_msgs/ControlCommandStamped.h>
#include <autoware_config_msgs/ConfigTwistFilter.h>
#include <autoware_health_checker/health_checker/health_checker.h>
#include <gtest/gtest_prod.h>
#include "accel_limiter.hpp"

namespace twist_filter
{
// const values
constexpr double MIN_LINEAR_X = 1e-3;
constexpr double MIN_LENGTH = 1e-3;
constexpr double MIN_DURATION = 1e-3;

struct StampedValue
{
  ros::Time time;
  double dt;
  double val;
  StampedValue() : time(0.0), dt(0.0), val(0.0) {}
  void reset()
  {
    time = ros::Time(0.0);
    val = 0.0;
  }
};

class TwistFilter
{
public:
  TwistFilter(ros::NodeHandle* nh, ros::NodeHandle* private_nh);
private:
  ros::NodeHandle* nh_ = nullptr;
  ros::NodeHandle* private_nh_ = nullptr;

  // publishers
  ros::Publisher twist_pub_, ctrl_pub_;
  ros::Publisher twist_lacc_limit_debug_pub_, twist_ljerk_limit_debug_pub_;
  ros::Publisher ctrl_lacc_limit_debug_pub_, ctrl_ljerk_limit_debug_pub_;
  ros::Publisher twist_lacc_result_pub_, twist_ljerk_result_pub_;
  ros::Publisher ctrl_lacc_result_pub_, ctrl_ljerk_result_pub_;

  // subscribers
  ros::Subscriber twist_sub_, ctrl_sub_, config_sub_;

  // ros params
  double wheel_base_;
  double longitudinal_velocity_limit_;
  double longitudinal_accel_limit_;
  double lateral_accel_limit_;
  double lateral_jerk_limit_;
  double lowpass_gain_linear_x_;
  double lowpass_gain_angular_z_;
  double lowpass_gain_steering_angle_;

  // dataset
  StampedValue az_prev_;
  StampedValue sa_prev_;

  LongitudinalAccelLimiter _lon_accel_limiter;

  // health_checker
  autoware_health_checker::HealthChecker health_checker_;

  boost::optional<double>
    calcLaccWithAngularZ(const double& lv, const double& az) const;
  boost::optional<double>
    calcLjerkWithAngularZ(const double& lv, const double& az) const;
  boost::optional<double>
    calcLaccWithSteeringAngle(const double& lv, const double& sa) const;
  boost::optional<double>
    calcLjerkWithSteeringAngle(const double& lv, const double& sa) const;
  void publishLateralResultsWithTwist(
    const geometry_msgs::TwistStamped& msg) const;
  void publishLateralResultsWithCtrl(
    const autoware_msgs::ControlCommandStamped& msg) const;
  void checkTwist(const geometry_msgs::TwistStamped& msg);
  void checkCtrl(const autoware_msgs::ControlCommandStamped& msg);
  geometry_msgs::TwistStamped
    lateralLimitTwist(const geometry_msgs::TwistStamped& msg);
  geometry_msgs::TwistStamped
    smoothTwist(const geometry_msgs::TwistStamped& msg);
  autoware_msgs::ControlCommandStamped
    lateralLimitCtrl(const autoware_msgs::ControlCommandStamped& msg);
  autoware_msgs::ControlCommandStamped
    smoothCtrl(const autoware_msgs::ControlCommandStamped& msg);
  void updatePrevTwist(const geometry_msgs::TwistStamped& msg);
  void updatePrevCtrl(const autoware_msgs::ControlCommandStamped& msg);
  void configCallback(
    const autoware_config_msgs::ConfigTwistFilterConstPtr& config);
  void TwistCmdCallback(const geometry_msgs::TwistStampedConstPtr& msg);
  void CtrlCmdCallback(const autoware_msgs::ControlCommandStampedConstPtr& msg);
  
  // Friend test setup for unit testing of private methods
  FRIEND_TEST(TwistFilterTest, test_longitudinal_twist_filter);
  FRIEND_TEST(TwistFilterTest, test_longitudinal_ctrl_filter);
};

}
