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

#include "dead_reckoner.hpp"


namespace dead_reckoner
{

namespace std_ph = std::placeholders;

DeadReckoner::DeadReckoner(const rclcpp::NodeOptions& options) : carma_ros2_utils::CarmaLifecycleNode(options)
{
}

carma_ros2_utils::CallbackReturn DeadReckoner::handle_on_configure(const rclcpp_lifecycle::State &)
{
  twist_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>("current_twist", 1, std::bind(&DeadReckoner::twist_cb, this, std_ph::_1));
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("current_odom", 10);
  return CallbackReturn::SUCCESS;
}

carma_ros2_utils::CallbackReturn DeadReckoner::handle_on_activate(const rclcpp_lifecycle::State &)
{
    return CallbackReturn::SUCCESS;
}

void DeadReckoner::twist_cb(geometry_msgs::msg::TwistStamped::UniquePtr msg)
{
  // TODO: calculate odom.pose.pose by accumulating
  nav_msgs::msg::Odometry odom;
  odom.header = msg->header;
  odom.twist.twist = msg->twist;
  odom_pub_->publish(odom);
}

} // namespace dead_reckoner


#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(dead_reckoner::DeadReckoner)
