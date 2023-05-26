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

#include "map_param_loader.hpp"


namespace map_param_loader
{
  namespace std_ph = std::placeholders;

  MapParamLoader::MapParamLoader(const rclcpp::NodeOptions &options) : carma_ros2_utils::CarmaLifecycleNode(options)
  {
    
  }

  carma_ros2_utils::CallbackReturn MapParamLoader::handle_on_configure(const rclcpp_lifecycle::State &)
  {
    lanelet2_filename = declare_parameter<std::string>("file_name", lanelet2_filename);
    broadcast_earth_frame = declare_parameter<bool>("broadcast_earth_frame", broadcast_earth_frame);

    // NOTE: Currently, intra-process comms must be disabled for the following two publishers that are transient_local: https://github.com/ros2/rclcpp/issues/1753
    rclcpp::PublisherOptions intra_proc_disabled; 
    intra_proc_disabled.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable; // Disable intra-process comms for this PublisherOptions object

    // Create a publisher that will send all previously published messages to late-joining subscribers ONLY If the subscriber is transient_local too
    auto pub_qos_transient_local = rclcpp::QoS(rclcpp::KeepAll()); // A publisher with this QoS will store all messages that it has sent on the topic
    pub_qos_transient_local.transient_local();  // A publisher with this QoS will re-send all (when KeepAll is used) messages to all late-joining subscribers 
                                         // NOTE: The subscriber's QoS must be set to transient_local() as well for earlier messages to be resent to the later-joiner.

    georef_pub_ = this->create_publisher<std_msgs::msg::String>("georeference",  pub_qos_transient_local, intra_proc_disabled);

    return CallbackReturn::SUCCESS;
  }

  carma_ros2_utils::CallbackReturn MapParamLoader::handle_on_activate(const rclcpp_lifecycle::State &)
  {
    int projector_type = 1; // default value

    // Parse geo reference info from the lanelet map (.osm)
    lanelet::io_handlers::AutowareOsmParser::parseMapParams(lanelet2_filename, &projector_type, &target_frame_);

    if (broadcast_earth_frame) {
      // Get the transform to ecef (when parsed target_frame is map_frame)
      tf2::Transform tf = getTransform(target_frame_);

      // Broadcast the transform
      broadcastTransform(tf);  
    }

    timer_ = this->create_wall_timer(std::chrono::milliseconds(2000), std::bind(&MapParamLoader::timer_callback, this));
    
    return CallbackReturn::SUCCESS;
  }

  void MapParamLoader::timer_callback()
  {
    std_msgs::msg::String georef_msg;
    georef_msg.data = target_frame_;
    georef_pub_->publish(georef_msg);
  }


  // Get transform from map_frame coord to ecef_frame coord using respective proj strings
  tf2::Transform MapParamLoader::getTransform(const std::string& map_frame)
  {

    lanelet::projection::LocalFrameProjector local_projector(map_frame.c_str());
    
    tf2::Matrix3x3 rot_mat, id = tf2::Matrix3x3::getIdentity();
    lanelet::BasicPoint3d origin_in_map{0,0,0}, origin_in_ecef;

    // Solve map_to_ecef transformation
    // Get translation of map with respect to ecef
    origin_in_ecef = local_projector.projectECEF(origin_in_map, 1);

    // Solve rotation matrix using (1,0,0), (0,1,0), (0,0,1) vectors in map
    for (auto i = 0; i < 3; i ++)
    {
        lanelet::BasicPoint3d rot_mat_row = local_projector.projectECEF(lanelet::BasicPoint3d{id[i][0],id[i][1],id[i][2]}, 1) - origin_in_ecef;
        rot_mat[i][0] = rot_mat_row[0];
        rot_mat[i][1] = rot_mat_row[1];
        rot_mat[i][2] = rot_mat_row[2];
    }
    // Transpose due to the way tf2::Matrix3x3 supposed to be stored.
    tf2::Vector3 v_origin_in_ecef{origin_in_ecef[0],origin_in_ecef[1],origin_in_ecef[2]};
    tf2::Transform tf(rot_mat.transpose(), v_origin_in_ecef);
    
    // map_to_ecef tf
    return tf;
  }

  // broadcast the transform to tf_static topic
  void MapParamLoader::broadcastTransform(const tf2::Transform& transform)
  {
      static tf2_ros::StaticTransformBroadcaster br(this);
      geometry_msgs::msg::TransformStamped transformStamped;

      tf2::Vector3 translation = transform.getOrigin();
      tf2::Quaternion rotation = transform.getRotation();

      transformStamped.header.stamp = rclcpp::Time(0,0); //TODO:: Check if node->now() needed
      transformStamped.header.frame_id = "earth";
      transformStamped.child_frame_id = "map";
      transformStamped.transform.translation.x = translation[0];
      transformStamped.transform.translation.y = translation[1];
      transformStamped.transform.translation.z = translation[2];
      transformStamped.transform.rotation.x = rotation[0];
      transformStamped.transform.rotation.y = rotation[1];
      transformStamped.transform.rotation.z = rotation[2];
      transformStamped.transform.rotation.w = rotation[3];
      br.sendTransform(transformStamped);
  }

} // namespace map_param_loader


#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(map_param_loader::MapParamLoader)