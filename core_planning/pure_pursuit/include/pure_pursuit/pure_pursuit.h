/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef PURE_PURSUIT_PURE_PURSUIT_H
#define PURE_PURSUIT_PURE_PURSUIT_H

// ROS includes
#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

// C++ includes
#include <vector>

// User defined includes
#include <autoware_msgs/Lane.h>
#include <libwaypoint_follower/libwaypoint_follower.h>

namespace waypoint_follower
{
class PurePursuit
{
public:
  PurePursuit();
  ~PurePursuit();

  // for setting data
  void setLookaheadDistance(const double& ld)
  {
    lookahead_distance_ = ld;
  }
  void setMinimumLookaheadDistance(const double& minld)
  {
    minimum_lookahead_distance_ = minld;
  }
  void setCurrentVelocity(const double& cur_vel)
  {
    current_linear_velocity_ = cur_vel;
  }
  void setCurrentWaypoints(const std::vector<autoware_msgs::Waypoint>& wps)
  {
    current_waypoints_ = wps;
    // check if the point is in front or back
    // we skip 0 because it is our current position
    tf::Vector3 curr_vector(current_waypoints_.at(1).pose.pose.position.x - current_pose_.position.x, 
                      current_waypoints_.at(1).pose.pose.position.y - current_pose_.position.y, 
                      current_waypoints_.at(1).pose.pose.position.z - current_pose_.position.z);
    previous_pose_ = current_pose_;
    curr_vector.setZ(0);
    prev_travelled_vector_ = curr_vector;
  }
  void setCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg)
  {
    current_pose_ = msg->pose;
  }
  void setLinearInterpolationParameter(const bool& param)
  {
    is_linear_interpolation_ = param;
  }

  // for debug on ROS
  geometry_msgs::Point getPoseOfNextWaypoint() const
  {
    std::cerr << ">next_waypoint_number_ used is " << next_waypoint_number_ << std::endl;
    std::cerr << ">and that is " << current_waypoints_.at(next_waypoint_number_).pose.pose.position.x << std::endl;
    return current_waypoints_.at(next_waypoint_number_).pose.pose.position;
  }
  geometry_msgs::Point getPoseOfNextTarget() const
  {
    return next_target_position_;
  }
  geometry_msgs::Pose getCurrentPose() const
  {
    return current_pose_;
  }
  std::vector<autoware_msgs::Waypoint> getCurrentWaypoints() const
  {
    return current_waypoints_;
  }
  double getLookaheadDistance() const
  {
    return lookahead_distance_;
  }
  double getMinimumLookaheadDistance() const
  {
    return minimum_lookahead_distance_;
  }
  void getNextWaypoint();

  // processing
  bool canGetCurvature(double* output_kappa);

private:
  // constant
  const double RADIUS_MAX_;
  const double KAPPA_MIN_;

  // variables
  bool is_linear_interpolation_;
  int next_waypoint_number_;
  geometry_msgs::Point next_target_position_;
  double lookahead_distance_;
  double minimum_lookahead_distance_;
  geometry_msgs::Pose current_pose_, previous_pose_;
  double current_linear_velocity_;
  tf::Vector3 prev_travelled_vector_;
  std::vector<autoware_msgs::Waypoint> current_waypoints_;

  // functions
  double calcCurvature(geometry_msgs::Point target) const;
  bool interpolateNextTarget(
    int next_waypoint, geometry_msgs::Point* next_target) const;
  
};
}  // namespace waypoint_follower

#endif  // PURE_PURSUIT_PURE_PURSUIT_H
