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

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <string>
#include <carma_ros2_utils/carma_lifecycle_node.hpp>
#include "voxel_grid_filter_config.hpp"

namespace voxel_grid_filter
{

  /**
   * \class VoxelGridFilterNode
   * \brief The class responsible for processing incoming voxel grids from lidar
   * 
   */
  class VoxelGridFilterNode : public carma_ros2_utils::CarmaLifecycleNode
  {

  private:

    Config config_;
    
    // Subscribers
    carma_ros2_utils::SubPtr<autoware_config_msgs::ConfigVoxelGridFilter> config_sub_;
    carma_ros2_utils::SubPtr<sensor_msgs::PointCloud2> scan_sub_;

    // Publishers
    carma_ros2_utils::PubPtr<sensor_msgs::msg::PointCloud2> filtered_points_pub_;
    carma_ros2_utils::PubPtr<points_downsampler::msg::PointsDownsamplerInfo> points_downsampler_info_pub_;

  public:

    /**
     * \brief VoxelGridFilterNode constructor 
     */
    explicit VoxelGridFilterNode(const rclcpp::NodeOptions &);

    static void config_callback(const autoware_config_msgs::ConfigVoxelGridFilter::ConstPtr& input);
    static void scan_callback(const sensor_msgs::msg::PointCloud2::ConstPtr& input);

    ////
    // Overrides
    ////
    carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State &);

    /**
     * \brief Publish traffic control message 
     */
    void publishTrafficControlMessage(const carma_v2x_msgs::msg::TrafficControlMessage& traffic_control_msg) const;
  };

} // voxel_grid_filter
