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

#ifndef VOXEL_GRID_FILTER_H
#define VOXEL_GRID_FILTER_H

#include <rclcpp/rclcpp.hpp>
#include <carma_ros2_utils/carma_lifecycle_node.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include "autoware_config_msgs/msg/config_voxel_grid_filter.hpp"

#include "points_downsampler/msg/points_downsampler_info.hpp"

#include <chrono>

#include "points_downsampler.hpp"

#define MAX_MEASUREMENT_RANGE 200.0

namespace voxel_grid_filter
{

class VoxelGridFilter : public carma_ros2_utils::CarmaLifecycleNode
{
public:
  explicit VoxelGridFilter(const rclcpp::NodeOptions& options);

  ////
  // Overrides
  ////
  carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State &);
  carma_ros2_utils::CallbackReturn handle_on_activate(const rclcpp_lifecycle::State &);


  void config_callback(autoware_config_msgs::msg::ConfigVoxelGridFilter::UniquePtr input);
  void scan_callback(sensor_msgs::msg::PointCloud2::UniquePtr input);


private:
  carma_ros2_utils::PubPtr<sensor_msgs::msg::PointCloud2> filtered_points_pub_;
  carma_ros2_utils::PubPtr<points_downsampler::msg::PointsDownsamplerInfo> points_downsampler_info_pub_;
  
  carma_ros2_utils::SubPtr<autoware_config_msgs::msg::ConfigVoxelGridFilter> config_sub_;
  carma_ros2_utils::SubPtr<sensor_msgs::msg::PointCloud2> scan_sub_;

  int sample_num = 1000;

  points_downsampler::msg::PointsDownsamplerInfo points_downsampler_info_msg;
  double voxel_leaf_size = 2.0;

  std::chrono::time_point<std::chrono::system_clock> filter_start, filter_end;

  bool _output_log = false;
  std::ofstream ofs;
  std::string filename;

  std::string POINTS_TOPIC = "points_topic";
  double measurement_range = MAX_MEASUREMENT_RANGE;
};

} // namespace voxel_grid_filter

#endif  // VOXEL_GRID_FILTER_H
