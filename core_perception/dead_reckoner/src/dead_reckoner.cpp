/*
 *  Copyright (c) 2017, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
