/*
 * Copyright (C) 2019-2020 LEIDOS.
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

#include "map_file/map_param_loader.h"

namespace map_param_loader
{
// Get transform from map_frame coord to ecef_frame coord using respective proj strings
tf2::Transform getTransform(const std::string& map_frame, const std::string& ecef_frame)
{

  lanelet::projection::LocalFrameProjector local_projector(map_frame.c_str(), ecef_frame.c_str());
  
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
void broadcastTransform(const tf2::Transform& transform)
{
    static tf2_ros::StaticTransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    tf2::Vector3 translation = transform.getOrigin();
    tf2::Quaternion rotation = transform.getRotation();

    transformStamped.header.stamp = ros::Time::now();
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


int main(int argc, char **argv)
{
	ros::init(argc, argv, "map_param_loader");
  ros::NodeHandle private_nh("~");

  int projector_type = 1; // default value
  std::string base_frame , target_frame, lanelet2_filename;
  private_nh.param<std::string>("file_name", lanelet2_filename, "");
  
  // Parse geo reference info from the lanelet map (.osm)
  lanelet::io_handlers::AutowareOsmParser::parseMapParams(lanelet2_filename, &projector_type, &base_frame, &target_frame);

  // Get the transform (when parsed target_frame is map_frame, and base_frame is ECEF)
  tf2::Transform tf = map_param_loaer::getTransform(target_frame, base_frame);

  // Broadcast the transform
  map_param_loader::broadcastTransform(tf);

  ros::spin();

}