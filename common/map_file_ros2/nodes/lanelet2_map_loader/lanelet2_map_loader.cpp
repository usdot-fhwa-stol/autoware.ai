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

#include <rclcpp/rclcpp.hpp>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_core/LaneletMap.h>

#include <autoware_lanelet2_ros2_interface/utility/message_conversion.hpp>
#include <lanelet2_extension/projection/mgrs_projector.h>
#include <lanelet2_extension/projection/local_frame_projector.h>
#include <lanelet2_extension/io/autoware_osm_parser.h>
#include <autoware_lanelet2_ros2_interface/utility/utilities.hpp>

#include <autoware_lanelet2_msgs/msg/map_bin.hpp>
#include <string>
#include <lanelet2_map_loader.hpp>


namespace lanelet2_map_loader{

    Lanelet2MapLoader::Lanelet2MapLoader(const rclcpp::NodeOptions &options) : Node("lanelet2_map_loader")
    {
        // Declare Parameters
        load_type = declare_parameter<std::string>("load_type", load_type);
        lanelet2_filename = declare_parameter<std::string>("lanelet2_filename", lanelet2_filename);
        // Get Parameters
        get_parameter<std::string>("load_type", load_type);
        get_parameter<std::string>("lanelet2_filename", lanelet2_filename);

        // Set Publishers
        // NOTE: Currently, intra-process comms must be disabled for the following two publishers that are transient_local: https://github.com/ros2/rclcpp/issues/1753
        rclcpp::PublisherOptions intra_proc_disabled; 
        intra_proc_disabled.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable; // Disable intra-process comms for this PublisherOptions object
        // Create a publisher that will send all previously published messages to late-joining subscribers ONLY If the subscriber is transient_local too
        auto pub_qos_transient_local = rclcpp::QoS(rclcpp::KeepLast(1)); // A publisher with this QoS will store the "Last" message that it has sent on the topic

        map_bin_pub = create_publisher<autoware_lanelet2_msgs::msg::MapBin>("/lanelet_map_bin", pub_qos_transient_local, intra_proc_disabled); //Make latched

        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&Lanelet2MapLoader::timer_callback, this));
                    
    }
            
    void Lanelet2MapLoader::set_map_bin_msg(autoware_lanelet2_msgs::msg::MapBin& msg){
        map_bin_msg_ = msg;
    }

    void Lanelet2MapLoader::timer_callback(){

        if(!map_bin_msg_.header.frame_id.empty()){
            map_bin_msg_.header.stamp = this->now();
            map_bin_pub->publish(map_bin_msg_);
            // RCLCPP_INFO(get_logger(), "msg header frame_id: '%s'", map_bin_msg_.header.frame_id.c_str());
        }
    }


    std::string Lanelet2MapLoader::get_filename()
    {
        return lanelet2_filename;
    }

} //namespace lanelet2_map_loader


#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(lanelet2_map_loader::Lanelet2MapLoader)