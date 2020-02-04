
#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

#include <lanelet2_extension/projection/local_frame_projector.h>
#include <lanelet2_extension/io/autoware_osm_parser.h>

void printUsage()
{
  ROS_ERROR_STREAM("Usage:");
  ROS_ERROR_STREAM("rosrun map_file map_param_loader [.OSM]");
}

// Get transform from base(ECEF) to target(map) using local_projector
tf2::Transform getTransform(const std::string& base_frame, const std::string& target_frame)
{
  lanelet::projection::LocalFrameProjector local_projector(base_frame.c_str(), target_frame.c_str());
  tf2::Matrix3x3 rot_mat, id = tf2::Matrix3x3::getIdentity();
  lanelet::BasicPoint3d origin_in_map{0,0,0}, origin_in_ecef;

  // Solve map_to_ecef (target_to_base) transformation
  // Get translation from target to base
  origin_in_ecef = local_projector.project(origin_in_map, -1);

  // Solve rotation matrix using (1,0,0), (0,1,0), (0,0,1) vectors in map
  for (auto i = 0; i < 3; i ++)
  {
    lanelet::BasicPoint3d rot_mat_row = local_projector.project(lanelet::BasicPoint3d{id[i][0],id[i][1],id[i][2]}, -1) - origin_in_ecef;
    rot_mat[i][0] = rot_mat_row[0];
    rot_mat[i][1] = rot_mat_row[1];
    rot_mat[i][2] = rot_mat_row[2];
  }
  // Transpose due to the way tf2::Matrix3x3 supposed to be stored.
  tf2::Vector3 v_origin_in_ecef{origin_in_ecef[0],origin_in_ecef[1],origin_in_ecef[2]};
  tf2::Transform tf(rot_mat.transpose(), v_origin_in_ecef);
  
  // base_to_target tf
  return tf;
}

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

int main(int argc, char **argv)
{
	ros::init(argc, argv, "map_param_loader");

  if (argc < 2)
  {
      printUsage();
      return EXIT_FAILURE;
  }

  int projector_type = 1; // default value
  std::string base_frame , target_frame;
  std::string lanelet2_filename(argv[1]);
  
  // Parse geo reference info from the lanelet map (.osm)
  lanelet::io_handlers::AutowareOsmParser::parseMapParams(lanelet2_filename, &projector_type, &base_frame, &target_frame);

  // Get the transform
  tf2::Transform tf = getTransform(base_frame, target_frame);

  // Broadcast the transform
  broadcastTransform(tf);

}