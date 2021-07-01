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

#include <pure_pursuit/pure_pursuit.h>
#include <cmath>
#include <memory>

namespace waypoint_follower
{
// Simple estimation of curvature given two points.
// 1. Convert the target point from map frame into the current pose frame,
//    so it has a local coorinates of (pt.x, pt.y, pt.z).
// 2. If we think it is a cirle with a curvature kappa passing the two points,
//    kappa = 2 * pt.y / (pt.x * pt.x + pt.y * pt.y). For detailed derivation, please
//    refer to "Integrated Mobile Robot Control" by Omead Amidi
//    (CMU-RI-TR-90-17, Equation 3.10 on Page 21)
double PurePursuit::calcCurvature(const geometry_msgs::Point& target) const
{
  double kappa;
  const geometry_msgs::Point pt = calcRelativeCoordinate(target, current_pose_);
  const double denominator = pt.x * pt.x + pt.y * pt.y;
  const double numerator = 2.0 * pt.y;

  if (denominator != 0.0)
  {
    kappa = numerator / denominator;
  }
  else
  {
    kappa = numerator > 0.0 ? KAPPA_MIN_ : -KAPPA_MIN_;
  }

  return kappa;
}

// linear interpolation of next target
bool PurePursuit::interpolateNextTarget(int next_waypoint, geometry_msgs::Point* next_target) const
{
  const int path_size = static_cast<int>(current_waypoints_.size());
  if (next_waypoint == path_size - 1)
  {
    *next_target = current_waypoints_.back().pose.pose.position;
    return true;
  }
  const double search_radius = lookahead_distance_;
  const geometry_msgs::Point end = current_waypoints_.at(next_waypoint).pose.pose.position;
  const geometry_msgs::Point start = current_waypoints_.at(next_waypoint - 1).pose.pose.position;

  // project ego vehicle's current position at C onto the line at D in between two waypoints A and B.
  const tf::Vector3 p_A(start.x, start.y, 0.0);
  const tf::Vector3 p_B(end.x, end.y, 0.0);
  const tf::Vector3 p_C(current_pose_.position.x, current_pose_.position.y, 0.0);
  const tf::Vector3 AB = p_B - p_A;
  const tf::Vector3 AC = p_C - p_A;
  const tf::Vector3 p_D = p_A + AC.dot(AB) / AB.dot(AB) * AB;
  const double dist_CD = (p_D - p_C).length();

  bool found = false;
  tf::Vector3 final_goal;
  // Draw a circle centered at p_C with a radius of search_radius
  if (dist_CD > search_radius)
  {
    // no intersection in between the circle and AB
    found = false;
  }
  else if (dist_CD == search_radius)
  {
    // one intersection
    final_goal = p_D;
    found = true;
  }
  else
  {
    // two intersections
    // get intersection in front of vehicle
    double s = sqrt(pow(search_radius, 2) - pow(dist_CD, 2));
    tf::Vector3 p_E = p_D + s * AB.normalized();
    tf::Vector3 p_F = p_D - s * AB.normalized();

    // verify whether these two points lie on line segment AB
    if ((p_B - p_E).length2() < AB.length2())
    {
      final_goal = p_E;
      found = true;
    }
    else if ((p_B - p_F).length2() < AB.length2())
    {
      final_goal = p_F;
      found = true;
    }
    else
    {
      found = false;
    }
  }

  if (found)
  {
    next_target->x = final_goal.x();
    next_target->y = final_goal.y();
    next_target->z = current_pose_.position.z;
  }

  return found;
}

int PurePursuit::getNextWaypointNumber(bool use_lookahead_distance)
{
  int next_waypoint_number = -1;
  
  int path_size = static_cast<int>(current_waypoints_.size());

  // if waypoints are not given, do nothing.
  if (path_size == 0)
  {
    next_waypoint_number = -1;
    return next_waypoint_number;
  }
  double closest_distance = getPlaneDistance(current_waypoints_.at(0).pose.pose.position, current_pose_.position);
  // look for the next waypoint.
  for (int i = 1; i < path_size; i++)
  {
    // if search waypoint is the last
    if (i == (path_size - 1))
    {
      ROS_INFO_STREAM(">> Search waypoint reached the last: x: " << current_waypoints_.at(i).pose.pose.position.x 
                                              << ", y: " << current_waypoints_.at(i).pose.pose.position.y << ", speed: " << current_waypoints_.at(i).twist.twist.linear.x * 2.23694 << "mph");
      next_waypoint_number = i;
      return next_waypoint_number;
    }

    double current_distance = getPlaneDistance(current_waypoints_.at(i).pose.pose.position, current_pose_.position);
    
    // due to the fact that waypoints represent small portion of the road, there is no suddent change in direction
    // also since waypoints are in increasing distance from old curr_pos (which may have passed),
    // waypoint distances first decrease and increase back again, which helps find the closest point   
    if (use_lookahead_distance)
    {
      // loop through until we hit the closest point
      if (current_distance <= closest_distance) 
      {
        closest_distance = current_distance;
        continue;
      }
      // loop through until we hit the closest and effective point
      if (current_distance < lookahead_distance_)
      {
        continue;
      }
    }
    else
    {
      // loop through until we hit the closest point
      if (current_distance < closest_distance )
      {
        closest_distance = current_distance;
        continue;
      }
    }
    // Else, by this point, we found that prev point is the closest point and is bigger than lookahead_distance if applicable

    // then check if the prev point is in front or back
    tf::Vector3 curr_vector(current_waypoints_.at(i - 1).pose.pose.position.x - current_pose_.position.x, 
                      current_waypoints_.at(i - 1).pose.pose.position.y - current_pose_.position.y, 
                      current_waypoints_.at(i - 1).pose.pose.position.z - current_pose_.position.z);
    curr_vector.setZ(0);
    tf::Vector3 traj_vector(current_waypoints_.at(i).pose.pose.position.x - current_waypoints_.at(i - 1).pose.pose.position.x, 
                      current_waypoints_.at(i).pose.pose.position.y - current_waypoints_.at(i - 1).pose.pose.position.y, 
                      current_waypoints_.at(i).pose.pose.position.z - current_waypoints_.at(i - 1).pose.pose.position.z);
    traj_vector.setZ(0);
    double angle_in_rad = std::fabs(tf::tfAngle(curr_vector, traj_vector));
    // if degree between curr_vector and the direction of the trajectory is more than 90 degrees, we know last point is behind us and unsatisfactory
    if (std::isnan(angle_in_rad) || angle_in_rad > M_PI / 2)
    {
      continue;
    }
    next_waypoint_number = i - 1;
    return next_waypoint_number;
  }
  
  // if this program reaches here , it means we lost the waypoint!
  next_waypoint_number = -1;
  return next_waypoint_number;
}

void PurePursuit::setNextWaypoint(int next_waypoint_number)
{
  next_waypoint_number_ = next_waypoint_number;
}

bool PurePursuit::canGetCurvature(double* output_kappa)
{
  // search next waypoint
  next_waypoint_number_ = getNextWaypointNumber();
  if (next_waypoint_number_ == -1)
  {
    ROS_INFO("lost next waypoint");
    return false;
  }
  // check whether curvature is valid or not
  bool is_valid_curve = false;
  for (const auto& el : current_waypoints_)
  {
    if (getPlaneDistance(el.pose.pose.position, current_pose_.position)
      > minimum_lookahead_distance_)
    {
      is_valid_curve = true;
      break;
    }
  }
  if (!is_valid_curve)
  {
    return false;
  }
  // if is_linear_interpolation_ is false or next waypoint is first or last
  if (!is_linear_interpolation_ || next_waypoint_number_ == 0 ||
      next_waypoint_number_ == (static_cast<int>(current_waypoints_.size() - 1)))
  {
    next_target_position_ = current_waypoints_.at(next_waypoint_number_).pose.pose.position;
    *output_kappa = calcCurvature(next_target_position_);
    return true;
  }

  // linear interpolation and calculate angular velocity
  const bool interpolation = interpolateNextTarget(next_waypoint_number_, &next_target_position_);

  if (!interpolation)
  {
    ROS_INFO("lost target!");
    return false;
  }

  *output_kappa = calcCurvature(next_target_position_);
  return true;
}

}  // namespace waypoint_follower
