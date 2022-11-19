/*
 * Copyright 2019 Autoware Foundation. All rights reserved.
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

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "twist_gate/twist_gate.hpp"

class TwistGateTestClass : public rclcpp::Node
{
public:
    TwistGateTestClass() : Node("TwistGateTestClass")
    {
        twist_cmd_publisher = create_publisher<geometry_msgs::msg::TwistStamped>("twist_cmd", 0);
        control_cmd_publisher = create_publisher<autoware_msgs::msg::ControlCommandStamped>("ctrl_cmd", 0);
        decision_maker_state_publisher = create_publisher<std_msgs::msg::String>("decision_maker/state", 0);
        lamp_cmd_publisher = create_publisher<autoware_msgs::msg::LampCmd>("lamp_cmd", 0);
        emergency_vehicle_cmd_publisher = create_publisher<autoware_msgs::msg::VehicleCmd>("emergency_velocity", 0);

        vehicle_cmd_subscriber = create_subscription<autoware_msgs::msg::VehicleCmd>("/vehicle_cmd", 1 , std::bind(&TwistGateTestClass::vehicleCmdCallback, this, std::placeholders::_1));

        rclcpp::NodeOptions options;
        tg = std::make_shared<TwistGate>(options);

        tg->configure(); //Call configure state transition
        tg->activate();  //Call activate state transition to get not read for runtime

    }

    std::shared_ptr<TwistGate> tg;
    autoware_msgs::msg::VehicleCmd cb_vehicle_cmd;

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_cmd_publisher;
    rclcpp::Publisher<autoware_msgs::msg::ControlCommandStamped>::SharedPtr control_cmd_publisher;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr decision_maker_state_publisher;
    rclcpp::Publisher<autoware_msgs::msg::LampCmd>::SharedPtr lamp_cmd_publisher;
    rclcpp::Publisher<autoware_msgs::msg::VehicleCmd>::SharedPtr emergency_vehicle_cmd_publisher;
    
    rclcpp::Subscription<autoware_msgs::msg::VehicleCmd>::SharedPtr vehicle_cmd_subscriber;


    autoware_msgs::msg::VehicleCmd setTgTwistGateMsg(double d_value, int i_value) {
        tg->output_msg_.twist_cmd.twist.linear.x = d_value;
        tg->output_msg_.twist_cmd.twist.angular.z = d_value;
        tg->output_msg_.mode = i_value;
        tg->output_msg_.gear_cmd.gear = i_value;
        tg->output_msg_.lamp_cmd.l = i_value;
        tg->output_msg_.lamp_cmd.r = i_value;
        tg->output_msg_.accel_cmd.accel = i_value;
        tg->output_msg_.brake_cmd.brake = i_value;
        tg->output_msg_.steer_cmd.steer = i_value;
        tg->output_msg_.ctrl_cmd.linear_velocity = i_value;
        tg->output_msg_.ctrl_cmd.steering_angle = i_value;
        tg->output_msg_.emergency = i_value;

        return tg->output_msg_;
    }

    autoware_msgs::msg::VehicleCmd getTgTwistGateMsg() {return tg->output_msg_;}

    void publishTwistCmd(double linear_x, double angular_z) {
        geometry_msgs::msg::TwistStamped msg;
        msg.header.stamp = this->now();
        msg.twist.linear.x = linear_x;
        msg.twist.angular.z = angular_z;

        twist_cmd_publisher->publish(msg);
    }

    void publishControlCmd(double linear_vel, double linear_acc,
                         double steer_angle) {
        autoware_msgs::msg::ControlCommandStamped msg;
        msg.header.stamp = this->now();
        msg.cmd.linear_velocity = linear_vel;
        msg.cmd.linear_acceleration = linear_acc;
        msg.cmd.steering_angle = steer_angle;

        control_cmd_publisher->publish(msg);
    }

    void publishLampCmd(int lamp_l, int lamp_r){
        autoware_msgs::msg::LampCmd msg;
        msg.header.stamp = this->now();
        msg.l = lamp_l;
        msg.r = lamp_r;

        lamp_cmd_publisher->publish(msg);
    }

    void publishEmergencyVehicleCmd(int emergency, double linear_vel){
        autoware_msgs::msg::VehicleCmd msg;
        msg.header.stamp = this->now();
        msg.emergency = emergency;
        msg.ctrl_cmd.linear_velocity = linear_vel;

        emergency_vehicle_cmd_publisher->publish(msg);
    }

    void publishDecisionMakerState(std::string states) {
        std_msgs::msg::String msg;
        msg.data = states;

        decision_maker_state_publisher->publish(msg);
    }

    void vehicleCmdCallback(autoware_msgs::msg::VehicleCmd::SharedPtr msg) {
        cb_vehicle_cmd = *msg;
    }

    autoware_msgs::msg::VehicleCmd getTwistGateMsg() { return tg->output_msg_; }

    bool getIsStateDriveFlag() { return tg->is_state_drive_; }
};
