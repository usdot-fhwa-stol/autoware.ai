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

#ifndef TWIST_GATE_TWIST_GATE_H
#define TWIST_GATE_TWIST_GATE_H

#include <string>
#include <iostream>
#include <map>
#include <thread>
#include <memory>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

#include <autoware_config_msgs/msg/config_twist_filter.hpp>
#include <autoware_msgs/msg/control_command_stamped.hpp>
#include <autoware_msgs/msg/vehicle_cmd.hpp>
#include <autoware_msgs/msg/gear.hpp>
#include <carma_ros2_utils/carma_lifecycle_node.hpp>


class TwistGate : public carma_ros2_utils::CarmaLifecycleNode
{
    using vehicle_cmd_msg_t = autoware_msgs::msg::VehicleCmd;

    friend class TwistGateTestClass;

public:
    
    explicit TwistGate(const rclcpp::NodeOptions &);

    ////
    // Overrides
    ////
    carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State &prev_state);

private:
    void twistCmdCallback(const geometry_msgs::msg::TwistStamped::UniquePtr input_msg);
    void ctrlCmdCallback(const autoware_msgs::msg::ControlCommandStamped::UniquePtr input_msg);
    void lampCmdCallback(const autoware_msgs::msg::LampCmd::UniquePtr input_msg);
    void stateCallback(const std_msgs::msg::String::UniquePtr input_msg);
    void emergencyCmdCallback(const vehicle_cmd_msg_t::UniquePtr input_msg);
    void updateEmergencyState();

    void updateStateAndPublish();
    void configCallback(const autoware_config_msgs::msg::ConfigTwistFilter::UniquePtr msg); 

    
    carma_ros2_utils::PubPtr<std_msgs::msg::String> control_command_pub_;
    carma_ros2_utils::PubPtr<vehicle_cmd_msg_t> vehicle_cmd_pub_;

    carma_ros2_utils::SubPtr<autoware_config_msgs::msg::ConfigTwistFilter> config_sub_;
    carma_ros2_utils::SubPtr<geometry_msgs::msg::TwistStamped> twist_cmd_sub_;
    carma_ros2_utils::SubPtr<autoware_msgs::msg::ControlCommandStamped> ctrl_cmd_sub_;
    carma_ros2_utils::SubPtr<autoware_msgs::msg::LampCmd> lamp_cmd_sub_;
    carma_ros2_utils::SubPtr<std_msgs::msg::String> state_sub_;
    carma_ros2_utils::SubPtr<vehicle_cmd_msg_t> vehicle_cmd_sub_;
    // std::map<std::string, ros::Subscriber> auto_cmd_sub_stdmap_;

    vehicle_cmd_msg_t output_msg_;
    std_msgs::msg::Bool emergency_stop_msg_;
    rclcpp::Time emergency_handling_time_;
    rclcpp::Duration timeout_period_;

    bool is_state_drive_ = false;
    bool use_decision_maker_ = false;
    bool use_lgsim_ = false;

    bool emergency_handling_active_ = false;

    bool use_twist_ = false; // If true then the twist topic will be forwarded as the vehicle command. If false then the ctrl topic will be forwarded as the vehicle command
    
};

#endif  // TWIST_GATE_TWIST_GATE_H