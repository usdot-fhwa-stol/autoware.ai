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
#include <lanelet2_map_loader.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<lanelet2_map_loader::Lanelet2MapLoader>(rclcpp::NodeOptions());

    // Get parameters from node
    std::string lanelet2_filename = node->get_filename();

    lanelet::ErrorMessages errors;
    lanelet::LaneletMapPtr map;

    int projector_type = 0;
    std::string target_frame;

    // Parse geo reference info from the lanelet map (.osm)
    lanelet::io_handlers::AutowareOsmParser::parseMapParams(lanelet2_filename, &projector_type, &target_frame);

    if(projector_type == 0)
    {
        lanelet::projection::MGRSProjector projector;
        map = lanelet::load(lanelet2_filename, projector, &errors);
    } else if(projector_type == 1)
    {
        lanelet::projection::LocalFrameProjector local_projector(target_frame.c_str());
        map = lanelet::load(lanelet2_filename, local_projector, &errors);
    }

    for(const auto &error: errors)
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("map_file_ros2"), error);
    }
    if(!errors.empty())
    {
        return EXIT_FAILURE;
    }

    std::string format_version, map_version;
    lanelet::io_handlers::AutowareOsmParser::parseVersions(lanelet2_filename, &format_version, &map_version);

    autoware_lanelet2_msgs::msg::MapBin map_bin_msg;
    //map_bin_msg.header.stamp = rclcpp::Time(0,0);
    map_bin_msg.header.frame_id = "map";
    map_bin_msg.format_version = format_version;
    if (!map_version.empty()) {
        map_bin_msg.map_version = stoi(map_version); // CARMA Uses monotonically increasing map version numbers in carma_wm
    }
    lanelet::utils::conversion::toBinMsg(map, &map_bin_msg);

    node->set_map_bin_msg(map_bin_msg);

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
