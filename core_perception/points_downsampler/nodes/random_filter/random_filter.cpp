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

#include "random_filter.hpp"

namespace random_filter
{

namespace std_ph = std::placeholders;

RandomFilter::RandomFilter(const rclcpp::NodeOptions& options) : carma_ros2_utils::CarmaLifecycleNode(options)
{
  declare_parameter<std::string>("points_topic", POINTS_TOPIC);
  declare_parameter<bool>("output_log", _output_log);
  declare_parameter<double>("measurement_range", measurement_range);
  declare_parameter<int>("sample_num", sample_num);

}

carma_ros2_utils::CallbackReturn RandomFilter::handle_on_configure(const rclcpp_lifecycle::State &)
{
  get_parameter<std::string>("points_topic", POINTS_TOPIC);
  get_parameter<bool>("output_log", _output_log);
  get_parameter<int>("sample_num", sample_num);

  if(_output_log == true){
	  char buffer[80];
	  std::time_t now = std::time(NULL);
	  std::tm *pnow = std::localtime(&now);
	  std::strftime(buffer,80,"%Y%m%d_%H%M%S",pnow);
	  filename = "random_filter_" + std::string(buffer) + ".csv";
	  ofs.open(filename.c_str(), std::ios::app);
  }
  measurement_range = MAX_MEASUREMENT_RANGE;
  get_parameter<double>("measurement_range", measurement_range);

  // Publishers
  filtered_points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("random_points", 10);
  points_downsampler_info_pub_ = create_publisher<points_downsampler::msg::PointsDownsamplerInfo>("points_downsampler_info", 1000);

  // Subscribers
  config_sub_ = create_subscription<autoware_config_msgs::msg::ConfigRandomFilter>("config/random_filter", 10, std::bind(&RandomFilter::config_callback, this, std_ph::_1));
  scan_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(POINTS_TOPIC, 10, std::bind(&RandomFilter::scan_callback, this, std_ph::_1));

  return CallbackReturn::SUCCESS;
}

carma_ros2_utils::CallbackReturn RandomFilter::handle_on_activate(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

rcl_interfaces::msg::SetParametersResult RandomFilter::parameter_update_callback(const std::vector<rclcpp::Parameter> &parameters)
{
  auto error_double = update_params<double>({
    {"measurement_range", measurement_range},
  }, parameters);

  auto error_string = update_params<std::string>({
    {"points_topic", POINTS_TOPIC}
  }, parameters);

  auto error_bool = update_params<bool>({
    {"output_log", _output_log},
  }, parameters);

  auto error_int = update_params<int>({
    {"sample_num", sample_num},
  }, parameters);

  rcl_interfaces::msg::SetParametersResult result;

  result.successful = !error_double && !error_string && !error_bool && !error_int;

  return result;
}

void RandomFilter::config_callback(autoware_config_msgs::msg::ConfigRandomFilter::UniquePtr input)
{
  sample_num = input->sample_num;
  measurement_range = input->measurement_range;
}

void RandomFilter::scan_callback(sensor_msgs::msg::PointCloud2::UniquePtr input)
{
  pcl::PointXYZI sampled_p;
  pcl::PointCloud<pcl::PointXYZI> scan;

  pcl::fromROSMsg(*input, scan);

  if(measurement_range != MAX_MEASUREMENT_RANGE){
    scan = removePointsByRange(scan, 0, measurement_range);
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  filtered_scan_ptr->header = scan.header;

  filter_start = std::chrono::system_clock::now();

  int points_num = scan.size();
  int step = points_num / sample_num;

  if(scan.points.size() >= sample_num)
  {
    for (int i = 0; i < points_num; i++)
    {
      if ((int)filtered_scan_ptr->size() < sample_num && i % step == 0)
      {
        filtered_scan_ptr->points.push_back(scan.at(i));
      }
    }
  }else{
    filtered_scan_ptr = scan.makeShared();
  }

  sensor_msgs::msg::PointCloud2 filtered_msg;
  pcl::toROSMsg(*filtered_scan_ptr, filtered_msg);

  filter_end = std::chrono::system_clock::now();

  filtered_msg.header = input->header;
  filtered_points_pub_->publish(filtered_msg);

  points_downsampler_info_msg.header = input->header;
  points_downsampler_info_msg.filter_name = "random_filter";
  points_downsampler_info_msg.measurement_range = measurement_range;
  points_downsampler_info_msg.original_points_size = points_num;
  points_downsampler_info_msg.filtered_points_size = filtered_scan_ptr->size();
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


} // namespace random_filter


#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(random_filter::RandomFilter)