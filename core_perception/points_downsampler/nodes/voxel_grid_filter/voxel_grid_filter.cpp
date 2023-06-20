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

#include "voxel_grid_filter.hpp"

namespace voxel_grid_filter
{

namespace std_ph = std::placeholders;

VoxelGridFilter::VoxelGridFilter(const rclcpp::NodeOptions& options) : carma_ros2_utils::CarmaLifecycleNode(options)
{
  declare_parameter<std::string>("points_topic", POINTS_TOPIC);
  declare_parameter<bool>("output_log", _output_log);
  declare_parameter<double>("measurement_range", measurement_range);
}

carma_ros2_utils::CallbackReturn VoxelGridFilter::handle_on_configure(const rclcpp_lifecycle::State &)
{
  get_parameter<std::string>("points_topic", POINTS_TOPIC);
  get_parameter<bool>("output_log", _output_log);

  if(_output_log == true){
	  char buffer[80];
	  std::time_t now = std::time(NULL);
	  std::tm *pnow = std::localtime(&now);
	  std::strftime(buffer,80,"%Y%m%d_%H%M%S",pnow);
	  filename = "voxel_grid_filter_" + std::string(buffer) + ".csv";
	  ofs.open(filename.c_str(), std::ios::app);
  }
  measurement_range = MAX_MEASUREMENT_RANGE;
  get_parameter<double>("measurement_range", measurement_range);

  // Publishers
  filtered_points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_points", 10);
  points_downsampler_info_pub_ = create_publisher<points_downsampler::msg::PointsDownsamplerInfo>("/points_downsampler_info", 1000);

  // Subscribers
  config_sub_ = create_subscription<autoware_config_msgs::msg::ConfigVoxelGridFilter>("config/voxel_grid_filter", 10, std::bind(&VoxelGridFilter::config_callback, this, std_ph::_1));
  scan_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(POINTS_TOPIC, 10, std::bind(&VoxelGridFilter::scan_callback, this, std_ph::_1));

  return CallbackReturn::SUCCESS;
}

carma_ros2_utils::CallbackReturn VoxelGridFilter::handle_on_activate(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

void VoxelGridFilter::config_callback(autoware_config_msgs::msg::ConfigVoxelGridFilter::UniquePtr input)
{
  voxel_leaf_size = input->voxel_leaf_size;
  measurement_range = input->measurement_range;
}

void VoxelGridFilter::scan_callback(sensor_msgs::msg::PointCloud2::UniquePtr input)
{
  pcl::PointCloud<pcl::PointXYZI> scan;
  pcl::fromROSMsg(*input, scan);

  if(measurement_range != MAX_MEASUREMENT_RANGE){
    scan = removePointsByRange(scan, 0, measurement_range);
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(scan));
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());

  sensor_msgs::msg::PointCloud2 filtered_msg;

  filter_start = std::chrono::system_clock::now();

  // if voxel_leaf_size < 0.1 voxel_grid_filter cannot down sample (It is specification in PCL)
  if (voxel_leaf_size >= 0.1)
  {
    // Downsampling the velodyne scan using VoxelGrid filter
    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
    voxel_grid_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
    voxel_grid_filter.setInputCloud(scan_ptr);
    voxel_grid_filter.filter(*filtered_scan_ptr);
    pcl::toROSMsg(*filtered_scan_ptr, filtered_msg);
  }
  else
  {
    pcl::toROSMsg(*scan_ptr, filtered_msg);
  }

  filter_end = std::chrono::system_clock::now();

  filtered_msg.header = input->header;
  filtered_points_pub_->publish(filtered_msg);

  points_downsampler_info_msg.header = input->header;
  points_downsampler_info_msg.filter_name = "voxel_grid_filter";
  points_downsampler_info_msg.measurement_range = measurement_range;
  points_downsampler_info_msg.original_points_size = scan.size();
  if (voxel_leaf_size >= 0.1)
  {
    points_downsampler_info_msg.filtered_points_size = filtered_scan_ptr->size();
  }
  else
  {
    points_downsampler_info_msg.filtered_points_size = scan_ptr->size();
  }
  points_downsampler_info_msg.original_ring_size = 0;
  points_downsampler_info_msg.filtered_ring_size = 0;
  points_downsampler_info_msg.exe_time = std::chrono::duration_cast<std::chrono::microseconds>(filter_end - filter_start).count() / 1000.0;
  points_downsampler_info_pub_->publish(points_downsampler_info_msg);

  if(_output_log == true){
	  if(!ofs){
		  std::cerr << "Could not open " << filename << "." << std::endl;
		  exit(1);
	  }
	  ofs << std::to_string(rclcpp::Time(points_downsampler_info_msg.header.stamp).seconds()) << ","
		  << points_downsampler_info_msg.header.frame_id << ","
		  << points_downsampler_info_msg.filter_name << ","
		  << points_downsampler_info_msg.original_points_size << ","
		  << points_downsampler_info_msg.filtered_points_size << ","
		  << points_downsampler_info_msg.original_ring_size << ","
		  << points_downsampler_info_msg.filtered_ring_size << ","
		  << points_downsampler_info_msg.exe_time << ","
		  << std::endl;
  }
}

} // namespace voxel_grid_filter


#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(voxel_grid_filter::VoxelGridFilter)