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


#include <lanelet2_map_visualization.hpp>

namespace lanelet2_map_visualization{

    Lanelet2MapVisualization::Lanelet2MapVisualization(const rclcpp::NodeOptions &options) : carma_ros2_utils::CarmaLifecycleNode(options)
    {

        
    }

    carma_ros2_utils::CallbackReturn Lanelet2MapVisualization::handle_on_configure(const rclcpp_lifecycle::State &)
    {

        // Set Publisher
        // NOTE: Currently, intra-process comms must be disabled for the following two publishers that are transient_local: https://github.com/ros2/rclcpp/issues/1753
        rclcpp::PublisherOptions intra_proc_disabled; 
        intra_proc_disabled.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable; // Disable intra-process comms for this PublisherOptions object
        // Create a publisher that will send all previously published messages to late-joining subscribers ONLY If the subscriber is transient_local too
        auto pub_qos_transient_local = rclcpp::QoS(rclcpp::KeepLast(1)); // A publisher with this QoS will store the "Last" message that it has sent on the topic
        g_map_pub = create_publisher<visualization_msgs::msg::MarkerArray>("/lanelet2_map_viz", pub_qos_transient_local, intra_proc_disabled);

        //Set Subscriber
        bin_map_sub = create_subscription<autoware_lanelet2_msgs::msg::MapBin>("/lanelet_map_bin", 1, std::bind(&Lanelet2MapVisualization::binMapCallback, this, std::placeholders::_1));

        return CallbackReturn::SUCCESS;
    }
    
    
    void Lanelet2MapVisualization::insertMarkerArray(visualization_msgs::msg::MarkerArray* a1, const visualization_msgs::msg::MarkerArray& a2)
    {
        a1->markers.insert(a1->markers.end(), a2.markers.begin(), a2.markers.end());
    }

    void Lanelet2MapVisualization::setColor(std_msgs::msg::ColorRGBA* cl, double r, double g, double b, double a)
    {
        cl->r = r;
        cl->g = g;
        cl->b = b;
        cl->a = a;
    }

    void Lanelet2MapVisualization::binMapCallback(const autoware_lanelet2_msgs::msg::MapBin::SharedPtr msg)
    {
        lanelet::LaneletMapPtr viz_lanelet_map(new lanelet::LaneletMap);

        lanelet::utils::conversion::fromBinMsg(*msg, viz_lanelet_map);
        RCLCPP_INFO(get_logger(), "Map loaded");

        // get lanelets etc to visualize
        lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(viz_lanelet_map);
        lanelet::ConstLanelets road_lanelets = lanelet::utils::query::roadLanelets(all_lanelets);
        lanelet::ConstLanelets crosswalk_lanelets = lanelet::utils::query::crosswalkLanelets(all_lanelets);

        std::vector<lanelet::ConstLineString3d> tl_stop_lines = lanelet::utils::query::getTrafficLightStopLines(road_lanelets);
        std::vector<lanelet::ConstLineString3d> ss_stop_lines = lanelet::utils::query::getStopSignStopLines(road_lanelets);
        std::vector<lanelet::TrafficLightConstPtr> tl_reg_elems = lanelet::utils::query::trafficLights(all_lanelets);
        std::vector<lanelet::AutowareTrafficLightConstPtr> aw_tl_reg_elems =
            lanelet::utils::query::autowareTrafficLights(all_lanelets);

        std_msgs::msg::ColorRGBA cl_road, cl_cross, cl_ll_borders, cl_tl_stoplines, cl_ss_stoplines, cl_trafficlights;
        setColor(&cl_road, 0.2, 0.7, 0.7, 0.3);
        setColor(&cl_cross, 0.2, 0.7, 0.2, 0.3);
        setColor(&cl_ll_borders, 1.0, 1.0, 1.0, 1.0);
        setColor(&cl_tl_stoplines, 1.0, 0.5, 0.0, 0.5);
        setColor(&cl_ss_stoplines, 1.0, 0.0, 0.0, 0.5);
        setColor(&cl_trafficlights, 0.7, 0.7, 0.7, 0.8);

        visualization_msgs::msg::MarkerArray map_marker_array;

        insertMarkerArray(&map_marker_array, lanelet::visualization::laneletsBoundaryAsMarkerArray(
            road_lanelets, cl_ll_borders, g_viz_lanelets_centerline));
        insertMarkerArray(&map_marker_array, lanelet::visualization::laneletsAsTriangleMarkerArray(
            "road_lanelets", road_lanelets, cl_road));
        insertMarkerArray(&map_marker_array, lanelet::visualization::laneletsAsTriangleMarkerArray(
            "crosswalk_lanelets", crosswalk_lanelets, cl_cross));
        insertMarkerArray(&map_marker_array, lanelet::visualization::laneletDirectionAsMarkerArray(
            road_lanelets));
        insertMarkerArray(&map_marker_array, lanelet::visualization::lineStringsAsMarkerArray(
            tl_stop_lines, "traffic_light_stop_lines", cl_tl_stoplines, 0.5));
        insertMarkerArray(&map_marker_array, lanelet::visualization::lineStringsAsMarkerArray(
            ss_stop_lines, "stop_sign_stop_lines", cl_ss_stoplines, 0.5));
        insertMarkerArray(&map_marker_array, lanelet::visualization::autowareTrafficLightsAsMarkerArray(
            aw_tl_reg_elems, cl_trafficlights));

        RCLCPP_INFO(get_logger(), "Visualizing lanelet2 map with %lu lanelets, %lu stop lines, and %lu traffic lights",
            all_lanelets.size(), tl_stop_lines.size() + ss_stop_lines.size(), aw_tl_reg_elems.size());

        g_map_pub->publish(map_marker_array);
    }

    
} //namespace lanelet2_map_visualization


#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(lanelet2_map_visualization::Lanelet2MapVisualization)