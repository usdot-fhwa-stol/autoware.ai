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
#include "voxel_grid_filter_node.hpp"
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

#include "autoware_config_msgs/ConfigVoxelGridFilter.h"

#include <points_downsampler/PointsDownsamplerInfo.h>

#include <chrono>

#include "points_downsampler.h"

#define MAX_MEASUREMENT_RANGE 200.0

// Leaf size of VoxelGrid filter.
static double voxel_leaf_size = 2.0;

static points_downsampler::PointsDownsamplerInfo points_downsampler_info_msg;

static std::chrono::time_point<std::chrono::system_clock> filter_start, filter_end;

static std::ofstream ofs;
static std::string filename;

static double measurement_range = MAX_MEASUREMENT_RANGE;

namespace voxel_grid_filter
{
  namespace std_ph = std::placeholders;

  VoxelGridFilterNode::VoxelGridFilterNode(const rclcpp::NodeOptions &options)
      : carma_ros2_utils::CarmaLifecycleNode(options)
  {
    // Create initial config
    config_ = Config();

    // Declare parameters
    config_.points_topic = declare_parameter<std::string>("points_topic", config_.points_topic);
    config_.output_log = declare_parameter<std::string>("output_log", config_.output_log);
    config_.measurement_range = declare_parameter<double>("measurement_range", config_.measurement_range);  

  }

  carma_ros2_utils::CallbackReturn VoxelGridFilterNode::handle_on_configure(const rclcpp_lifecycle::State &)
  {

    //Reset Config
    config_ = Config();
        
    //Load parameters
    get_parameter<std::string>("points_topic" config_.points_topic);
    get_parameter<std::string>("output_log", config_._output_log);
    get_parameter<double>("measurement_range", config_.measurement_range);

    // Setup subscribers
    config_sub_ = create_subscription<autoware_config_msgs::ConfigVoxelGridFilter>("config/voxel_grid_filter", 10,std::bind(&VoxelGridFilterNode::config_callback, this, std_ph::_1));
    scan_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(config_.points_topic, 10,std::bind(&VoxelGridFilterNode::scan_callback, this, std_ph::_1));
                                                          
    // Setup publishers
    filtered_points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_points", 10);
    points_downsampler_info_pub_ = create_publisher<points_downsampler::PointsDownsamplerInfo>("/points_downsampler_info", 1000);

    // Return success if everthing initialized successfully
    return CallbackReturn::SUCCESS;
  }

  static void VoxelGridFilterNode::config_callback(const autoware_config_msgs::ConfigVoxelGridFilter::ConstPtr& input)
{
  voxel_leaf_size = input->voxel_leaf_size;
  config_.measurement_range = input->config_.measurement_range;
}

static void VoxelGridFilterNode::scan_callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
  pcl::PointCloud<pcl::PointXYZI> scan;
  pcl::fromROSMsg(*input, scan);

  if(config_.measurement_range != MAX_MEASUREMENT_RANGE){
    scan = removePointsByRange(scan, 0, config_.measurement_range);
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(scan));
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());

  sensor_msgs::PointCloud2 filtered_msg;

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
  points_downsampler_info_msg.config_.measurement_range= config_.measurement_range;
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

  if(config_._output_log== true){
    if(!ofs){
      std::cerr << "Could not open " << filename << "." << std::endl;
      exit(1);
    }
    ofs << points_downsampler_info_msg.header.seq << ","
      << points_downsampler_info_msg.header.stamp << ","
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

} // voxel_grid_filter

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(voxel_grid_filter::VoxelGridFilterNode)