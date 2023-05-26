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

#include <visualization_msgs/msg/marker_array.hpp>
#include <lanelet2_projection/UTM.h>
#include <autoware_lanelet2_msgs/msg/map_bin.hpp>
#include <carma_ros2_utils/carma_lifecycle_node.hpp>
#include <autoware_lanelet2_ros2_interface/utility/message_conversion.hpp>
#include <autoware_lanelet2_ros2_interface/utility/query.hpp>
#include <autoware_lanelet2_ros2_interface/visualization/visualization.hpp>
#include <lanelet2_extension/regulatory_elements/autoware_traffic_light.h>

#include <vector>

static bool g_viz_lanelets_centerline = true;

namespace lanelet2_map_visualization{
    class Lanelet2MapVisualization : public carma_ros2_utils::CarmaLifecycleNode
    {
        private:

            rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr g_map_pub;
            rclcpp::Subscription <autoware_lanelet2_msgs::msg::MapBin>::SharedPtr bin_map_sub;

            void insertMarkerArray(visualization_msgs::msg::MarkerArray* a1, const visualization_msgs::msg::MarkerArray& a2);
            void setColor(std_msgs::msg::ColorRGBA* cl, double r, double g, double b, double a);
            void binMapCallback(const autoware_lanelet2_msgs::msg::MapBin::SharedPtr msg);

        public:
            Lanelet2MapVisualization(const rclcpp::NodeOptions &options);

            carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State &);

    };
}



