/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
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

#include <ros/ros.h>
#include <gtest/gtest.h>
#include <pure_pursuit/pure_pursuit_core.h>

namespace waypoint_follower
{
  geometry_msgs::PoseStamped generateCurrentPose(double x, double y, double yaw)
  {
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    tf::Quaternion quaternion = tf::createQuaternionFromRPY(0, 0, yaw);
    quaternionTFToMsg(quaternion, pose.pose.orientation);
    return std::move(pose);
  }
class PurePursuitNodeTestSuite : public ::testing::Test
{
protected:
  std::unique_ptr<PurePursuitNode> obj_;
  virtual void SetUp()
  {
    obj_ = std::unique_ptr<PurePursuitNode>(new PurePursuitNode());
    obj_->add_virtual_end_waypoints_ = true;
  }
  virtual void TearDown()
  {
    obj_.reset();
  }

public:
  PurePursuitNodeTestSuite() {}
  ~PurePursuitNodeTestSuite() {}
  LaneDirection getDirection()
  {
    return obj_->direction_;
  }
  void ppCallbackFromWayPoints(const autoware_msgs::LaneConstPtr& msg)
  {
    obj_->callbackFromWayPoints(msg);
  }
  void ppConnectVirtualLastWaypoints(
    autoware_msgs::Lane* expand_lane, LaneDirection direction)
  {
    obj_->connectVirtualLastWaypoints(expand_lane, direction);
  }
  void ppcallbackFromCurrentPose(
    const geometry_msgs::PoseStampedConstPtr& msg)
  {
    obj_->callbackFromCurrentPose(msg);
  }
  void ppgetNextWaypoint()
  {
    obj_->getNextWaypoint();
  }
  geometry_msgs::Point ppGetPoseOfNextWaypoint() const
  {
    return obj_->getPoseOfNextWaypoint();
  }
  void ASSERT_NEAR_NEXT_WP_POSE_USING_CURR_POSE(double curr_pose_x, double curr_pose_y, double next_wp_pose_x, double next_wp_pose_y)
  {
    geometry_msgs::PoseStamped pose = waypoint_follower::generateCurrentPose(curr_pose_x, curr_pose_y, 0);
    geometry_msgs::PoseStampedConstPtr pose_ptr = boost::make_shared<const geometry_msgs::PoseStamped>(pose);
    ppcallbackFromCurrentPose(pose_ptr);
    ppgetNextWaypoint();
    auto next_wp_pose_calculated = ppGetPoseOfNextWaypoint();
    ASSERT_NEAR(next_wp_pose_calculated.x, next_wp_pose_x, 0.0001);
    ASSERT_NEAR(next_wp_pose_calculated.y, next_wp_pose_y, 0.0001);
  }

  void ASSERT_NOT_NEAR_NEXT_WP_POSE_USING_CURR_POSE(double curr_pose_x, double curr_pose_y, double next_wp_pose_x, double next_wp_pose_y)
  {
    geometry_msgs::PoseStamped pose = waypoint_follower::generateCurrentPose(curr_pose_x, curr_pose_y, 0);
    geometry_msgs::PoseStampedConstPtr pose_ptr = boost::make_shared<const geometry_msgs::PoseStamped>(pose);
    ppcallbackFromCurrentPose(pose_ptr);
    ppgetNextWaypoint();
    auto next_wp_pose_calculated = ppGetPoseOfNextWaypoint();
    ASSERT_FALSE(std::abs(next_wp_pose_calculated.x - next_wp_pose_x) < 0.001);
  }
};



TEST_F(PurePursuitNodeTestSuite, inputPositivePath)
{
  autoware_msgs::Lane original_lane;
  original_lane.waypoints.resize(3, autoware_msgs::Waypoint());
  for (int i = 0; i < 3; i++)
  {
    original_lane.waypoints[i].pose.pose.position.x = i;
    original_lane.waypoints[i].pose.pose.orientation =
      tf::createQuaternionMsgFromYaw(0.0);
  }
  const autoware_msgs::LaneConstPtr
    lp(boost::make_shared<autoware_msgs::Lane>(original_lane));
  ppCallbackFromWayPoints(lp);
  ASSERT_EQ(getDirection(), LaneDirection::Forward)
    << "direction is not matching to positive lane.";
}

TEST_F(PurePursuitNodeTestSuite, inputNegativePath)
{
  autoware_msgs::Lane original_lane;
  original_lane.waypoints.resize(3, autoware_msgs::Waypoint());
  for (int i = 0; i < 3; i++)
  {
    original_lane.waypoints[i].pose.pose.position.x = -i;
    original_lane.waypoints[i].pose.pose.orientation =
      tf::createQuaternionMsgFromYaw(0.0);
  }
  const autoware_msgs::LaneConstPtr
    lp(boost::make_shared<autoware_msgs::Lane>(original_lane));
  ppCallbackFromWayPoints(lp);
  ASSERT_EQ(getDirection(), LaneDirection::Backward)
    << "direction is not matching to negative lane.";
}
// If original lane is empty, new lane is also empty.
TEST_F(PurePursuitNodeTestSuite, inputEmptyLane)
{
  autoware_msgs::Lane original_lane, new_lane;
  ppConnectVirtualLastWaypoints(&new_lane, LaneDirection::Forward);
  ASSERT_EQ(original_lane.waypoints.size(), new_lane.waypoints.size())
    << "Input empty lane, and output is not empty";
}

// If the original lane exceeds 2 points,
// the additional part will be updated at
// the interval of the first 2 points.
TEST_F(PurePursuitNodeTestSuite, inputNormalLane)
{
  autoware_msgs::Lane original_lane;
  original_lane.waypoints.resize(2, autoware_msgs::Waypoint());
  for (int i = 0; i < 2; i++)
  {
    original_lane.waypoints[i].pose.pose.position.x = i;
  }
  autoware_msgs::Lane new_lane(original_lane);
  ppConnectVirtualLastWaypoints(&new_lane, LaneDirection::Forward);

  ASSERT_LT(original_lane.waypoints.size(), new_lane.waypoints.size())
    << "Fail to expand waypoints";
}


TEST_F(PurePursuitNodeTestSuite, checkWaypointIsAheadOrBehind)
{  
  /*
  *           2.             |3  x. > x-th waypoint coordinate 
  *                          |2  
  *                          |1
  *  0. 1.          3.    4. |0
  * ------------------------ y 
  *x 0  1  2  3  4  5  6  7   
  */
  
  autoware_msgs::Lane original_lane;
  original_lane.waypoints.resize(4, autoware_msgs::Waypoint());
  original_lane.waypoints[0].pose.pose.position.x = 0;
  original_lane.waypoints[1].pose.pose.position.x = 1;
  original_lane.waypoints[2].pose.pose.position.x = 3; 
  original_lane.waypoints[2].pose.pose.position.y = 3; 
  original_lane.waypoints[3].pose.pose.position.x = 5;
  original_lane.waypoints[4].pose.pose.position.x = 7;  
  original_lane.waypoints[0].pose.pose.orientation =
      tf::createQuaternionMsgFromYaw(0.0);
  original_lane.waypoints[1].pose.pose.orientation =
      tf::createQuaternionMsgFromYaw(0.0);
  original_lane.waypoints[2].pose.pose.orientation =
      tf::createQuaternionMsgFromYaw(0.0);
  original_lane.waypoints[3].pose.pose.orientation =
      tf::createQuaternionMsgFromYaw(0.0);
  original_lane.waypoints[4].pose.pose.orientation =
      tf::createQuaternionMsgFromYaw(0.0);

  geometry_msgs::PoseStamped pose = generateCurrentPose(-0.5, 0, 0);
  geometry_msgs::PoseStampedConstPtr pose_ptr = boost::make_shared<const geometry_msgs::PoseStamped>(pose);
  ppcallbackFromCurrentPose(pose_ptr);

  const autoware_msgs::LaneConstPtr
    lp(boost::make_shared<autoware_msgs::Lane>(original_lane));
  ppCallbackFromWayPoints(lp);
  ppgetNextWaypoint();

  // following simulates the car going straight on x coord, 1D
  // despite the waypoint being in 2D to test the angles
  ASSERT_NEAR(ppGetPoseOfNextWaypoint().x, 0, 0.001);

  ASSERT_NEAR_NEXT_WP_POSE_USING_CURR_POSE(0.95, 0, 1, 0); // closest next waypoint is 1. (<1,0> 0 degrees ahead): qualified

  ASSERT_NEAR_NEXT_WP_POSE_USING_CURR_POSE(2.95, 0, 5, 0); // closest next waypoint is 2. (<3,3> angle is 123.69): disqualified
                                                                                    // so picked 3. (<5,0> 0 degrees ahead): qualified
  ASSERT_NEAR_NEXT_WP_POSE_USING_CURR_POSE(3.15, 0, 5, 0); // closest next waypoint is 3. qualified

  ASSERT_NEAR_NEXT_WP_POSE_USING_CURR_POSE(6.95, 0, 7, 0); // closest next waypoint is 4. (<7,0> 0 degrees ahead): qualified

  ASSERT_NOT_NEAR_NEXT_WP_POSE_USING_CURR_POSE(7.95, 0, 7, 0); //  closest next waypoint is 5 (auto generated), not 4
}

}  // namespace waypoint_follower

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "PurePursuitTest");
  return RUN_ALL_TESTS();
}
