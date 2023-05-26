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
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <carma_ros2_utils/carma_lifecycle_node.hpp>
#include <lanelet2_extension/projection/local_frame_projector.h>
#include <lanelet2_extension/io/autoware_osm_parser.h>
#include <std_msgs/msg/string.hpp>


namespace map_param_loader
{
    class MapParamLoader : public carma_ros2_utils::CarmaLifecycleNode
    {
        private:
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr georef_pub_;
        std::shared_ptr<rclcpp::TimerBase> timer_;
        std::string target_frame_;

        void timer_callback();

        // Get transform from map_frame to ecef_frame using respective proj strings.
        tf2::Transform getTransform(const std::string& map_frame);
        // Broadcast the input transform to tf_static.
        void broadcastTransform(const tf2::Transform& transform);

        //parameters
        std::string lanelet2_filename;
        bool broadcast_earth_frame = false;

        public:
        MapParamLoader(const rclcpp::NodeOptions &options);

        carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State &);
        carma_ros2_utils::CallbackReturn handle_on_activate(const rclcpp_lifecycle::State &);

        
    };

}