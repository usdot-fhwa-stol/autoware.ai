/*
 * Copyright (C) 2023 LEIDOS.
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

#ifndef DEADRECKONER_H
#define DEADRECKONER_H

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <carma_ros2_utils/carma_lifecycle_node.hpp>

namespace dead_reckoner
{

class DeadReckoner : public carma_ros2_utils::CarmaLifecycleNode
{
public:
  explicit DeadReckoner(const rclcpp::NodeOptions& options);

  ////
  // Overrides
  ////
  carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State &);
  carma_ros2_utils::CallbackReturn handle_on_activate(const rclcpp_lifecycle::State &);

private:
  carma_ros2_utils::SubPtr<geometry_msgs::msg::TwistStamped> twist_sub_;
  carma_ros2_utils::PubPtr<nav_msgs::msg::Odometry> odom_pub_;
  
  void twist_cb(geometry_msgs::msg::TwistStamped::UniquePtr msg);
};

} // namespace dead_reckoner

#endif  // DEADRECKONER_H
