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
#include <carma_ros2_utils/carma_lifecycle_node.hpp>
#include <autoware_lanelet2_ros2_interface/utility/message_conversion.hpp>
#include <lanelet2_extension/projection/mgrs_projector.h>
#include <lanelet2_extension/projection/local_frame_projector.h>
#include <lanelet2_extension/io/autoware_osm_parser.h>
#include <autoware_lanelet2_ros2_interface/utility/utilities.hpp>

#include <autoware_lanelet2_msgs/msg/map_bin.hpp>
#include <string>

namespace lanelet2_map_loader{
void printUsage()
{
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("map_file_ros2"), "Usage:");
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("map_file_ros2"), "ros2 launch map_file lanelet2_map_loader [.OSM]:");
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("map_file_ros2"), "rosrun map_file lanelet2_map_loader download [X] [Y]: WARNING not implemented");
}

class Lanelet2MapLoader : public carma_ros2_utils::CarmaLifecycleNode
{
    private:
        rclcpp::Publisher<autoware_lanelet2_msgs::msg::MapBin>::SharedPtr map_bin_pub;

        // Parameters
        std::string load_type;
        std::string lanelet2_filename;
        
        rclcpp::TimerBase::SharedPtr timer_;
        autoware_lanelet2_msgs::msg::MapBin map_bin_msg_;
        
        void timer_callback();


    public:
        Lanelet2MapLoader(const rclcpp::NodeOptions &options);

        carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State &);

}; 

} //namespace lanelet2_map_loader

