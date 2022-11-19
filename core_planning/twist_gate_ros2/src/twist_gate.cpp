/*
 *  Copyright (c) 2017, Tier IV, Inc.
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

#include "twist_gate/twist_gate.hpp"

#include <memory>
#include <string>

namespace std_ph = std::placeholders;

TwistGate::TwistGate(const rclcpp::NodeOptions &options)
    : carma_ros2_utils::CarmaLifecycleNode(options), timeout_period_(10,0)
{

    //Declare parameters
    use_decision_maker_ = declare_parameter<bool>("~use_decision_maker", use_decision_maker_);
    use_lgsim_ = declare_parameter<bool>("~use_lgsim", use_lgsim_);
    use_twist_ = declare_parameter<bool>("~use_twist", use_twist_);

}

carma_ros2_utils::CallbackReturn TwistGate::handle_on_configure(const rclcpp_lifecycle::State &prev_state)
{
    //get parameters
    get_parameter<bool>("~use_decision_maker", use_decision_maker_);
    get_parameter<bool>("~use_lgsim", use_lgsim_);
    get_parameter<bool>("~use_twist", use_twist_);
    
    // TODO: control_command_pub_ is unused
    control_command_pub_ = create_publisher<std_msgs::msg::String>("ctrl_mode", 1);
    vehicle_cmd_pub_ = create_publisher<vehicle_cmd_msg_t>("vehicle_cmd",1);

    config_sub_ = create_subscription<autoware_config_msgs::msg::ConfigTwistFilter>("config/twist_filter", 1, std::bind(&TwistGate::configCallback, this, std_ph::_1));
    twist_cmd_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>("/twist_cmd", 1, std::bind(&TwistGate::twistCmdCallback, this, std_ph::_1));
    ctrl_cmd_sub_ = create_subscription<autoware_msgs::msg::ControlCommandStamped>("/ctrl_cmd", 1 , std::bind(&TwistGate::ctrlCmdCallback, this, std_ph::_1));
    lamp_cmd_sub_ = create_subscription<autoware_msgs::msg::LampCmd>("/lamp_cmd", 1, std::bind(&TwistGate::lampCmdCallback, this, std_ph::_1));
    state_sub_ = create_subscription<std_msgs::msg::String>("/decision_maker/state", 1, std::bind(&TwistGate::stateCallback, this, std_ph::_1));
    vehicle_cmd_sub_ = create_subscription<vehicle_cmd_msg_t>("emergency_velocity", 1, std::bind(&TwistGate::emergencyCmdCallback, this, std_ph::_1));

    emergency_stop_msg_.data = false;

    emergency_handling_time_ = this->now();

    return CallbackReturn::SUCCESS;

}

void TwistGate::twistCmdCallback(const geometry_msgs::msg::TwistStamped::UniquePtr input_msg)
{  
  if (!use_twist_) {
    return; // Ignore message if not using twist as input
  }

  updateEmergencyState();
  if (!emergency_handling_active_)
  {
    output_msg_.header.frame_id = input_msg->header.frame_id;
    output_msg_.header.stamp = input_msg->header.stamp;
    output_msg_.twist_cmd.twist = input_msg->twist;
    updateStateAndPublish();
  }
}

void TwistGate::ctrlCmdCallback(const autoware_msgs::msg::ControlCommandStamped::UniquePtr input_msg)
{
  if (use_twist_) {
    return; // Ignore message if using twist as input
  }

  updateEmergencyState();
  if (!emergency_handling_active_)
  {
    output_msg_.header.frame_id = input_msg->header.frame_id;
    output_msg_.header.stamp = input_msg->header.stamp;
    output_msg_.ctrl_cmd = input_msg->cmd;

    updateStateAndPublish();
  }
}

void TwistGate::lampCmdCallback(const autoware_msgs::msg::LampCmd::UniquePtr input_msg)
{
  updateEmergencyState();
  if (!emergency_handling_active_)
  {
    output_msg_.lamp_cmd.l = input_msg->l;
    output_msg_.lamp_cmd.r = input_msg->r;
  }
}

void TwistGate::stateCallback(const std_msgs::msg::String::UniquePtr input_msg)
{
  if (!emergency_handling_active_)
  {
    // Set gear based on motion states
    if (input_msg->data.find("WaitEngage") != std::string::npos ||
        input_msg->data.find("WaitDriveReady") != std::string::npos)
    {
      output_msg_.gear_cmd.gear = autoware_msgs::msg::Gear::PARK;
    }
    // Set Drive Gear
    else
    {
      if (use_lgsim_)
      {
        output_msg_.gear_cmd.gear = autoware_msgs::msg::Gear::NONE;
      }
      else
      {
        output_msg_.gear_cmd.gear = autoware_msgs::msg::Gear::DRIVE;
      }
    }

    // get drive state
    if (input_msg->data.find("Drive\n") != std::string::npos &&
        input_msg->data.find("VehicleReady\n") != std::string::npos)
    {
      is_state_drive_ = true;
    }
    else
    {
      is_state_drive_ = false;
    }

    // reset emergency flags
    if (input_msg->data.find("VehicleReady") != std::string::npos)
    {
      emergency_stop_msg_.data = false;
    }
  }
}

void TwistGate::emergencyCmdCallback(const vehicle_cmd_msg_t::UniquePtr input_msg)
{
  emergency_handling_time_ = this->now();
  emergency_handling_active_ = true;
  output_msg_ = *input_msg;

  updateStateAndPublish();
}

void TwistGate::updateEmergencyState()
{
  // Reset emergency handling
  if (emergency_handling_active_)
  {
    RCLCPP_ERROR_STREAM(get_logger(), "EMERGENCY HANDLING IS ACTIVE");
    // If no emergency message received for more than timeout_period_
    if ((this->now() - emergency_handling_time_) > timeout_period_)
    {
      emergency_handling_active_ = false;
    }
  }
}

void TwistGate::updateStateAndPublish()
{
  // Clear commands if we aren't in drive state
  // ssc_interface will handle autonomy disengagements if the control commands timeout
  if (use_decision_maker_ && (!is_state_drive_))
  {
    output_msg_.twist_cmd.twist = geometry_msgs::msg::Twist();
    output_msg_.ctrl_cmd = autoware_msgs::msg::ControlCommand();
  }

  vehicle_cmd_pub_->publish(output_msg_);
}

void TwistGate::configCallback(const autoware_config_msgs::msg::ConfigTwistFilter::UniquePtr msg)
{
  use_decision_maker_ = msg->use_decision_maker;
}