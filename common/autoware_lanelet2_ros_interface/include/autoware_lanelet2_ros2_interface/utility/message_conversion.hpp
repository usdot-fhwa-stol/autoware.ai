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

#ifndef AUTOWARE_LANELET2_ROS_INTERFACE_UTILITY_MESSAGE_CONVERSION_H
#define AUTOWARE_LANELET2_ROS_INTERFACE_UTILITY_MESSAGE_CONVERSION_H

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <lanelet2_core/LaneletMap.h>
#include <autoware_lanelet2_msgs/msg/map_bin.hpp>

namespace lanelet
{
namespace utils
{
namespace conversion
{
/**
 * [toBinMsg convervets lanelet2 map to ROS message. Similar implementation to
 * lanelet::io_handlers::BinHandler::write()]
 * @param map [lanelet map data]
 * @param msg [converted ROS message. Only "data" field is filled]
 */
void toBinMsg(const lanelet::LaneletMapPtr& map, autoware_lanelet2_msgs::msg::MapBin* msg);

/**
 * [fromBinMsg converts ROS message into lanelet2 data. Similar implementation
 * to lanelet::io_handlers::BinHandler::parse()]
 * @param msg [ROS message for lanelet map]
 * @param map [Converted lanelet2 data]
 */
void fromBinMsg(const autoware_lanelet2_msgs::msg::MapBin& msg, lanelet::LaneletMapPtr map);

/**
 * [toGeomMsgPt converts various point types to geometry_msgs point]
 * @param src [input point(geometry_msgs::msg::Point3,
 * Eigen::VEctor3d=lanelet::BasicPoint3d, lanelet::Point3d, lanelet::Point2d) ]
 * @param dst [converted geometry_msgs point]
 */
void toGeomMsgPt(const geometry_msgs::msg::Point32& src, geometry_msgs::msg::Point* dst);
void toGeomMsgPt(const Eigen::Vector3d& src, geometry_msgs::msg::Point* dst);
void toGeomMsgPt(const lanelet::ConstPoint3d& src, geometry_msgs::msg::Point* dst);
void toGeomMsgPt(const lanelet::ConstPoint2d& src, geometry_msgs::msg::Point* dst);

/**
 * [toGeomMsgPt converts various point types to geometry_msgs point]
 * @param src [input point(geometry_msgs::msg::Point3,
 * Eigen::VEctor3d=lanelet::BasicPoint3d, lanelet::Point3d, lanelet::Point2d) ]
 * @return    [converted geometry_msgs point]
 */
geometry_msgs::msg::Point toGeomMsgPt(const geometry_msgs::msg::Point32& src);
geometry_msgs::msg::Point toGeomMsgPt(const Eigen::Vector3d& src);
geometry_msgs::msg::Point toGeomMsgPt(const lanelet::ConstPoint3d& src);
geometry_msgs::msg::Point toGeomMsgPt(const lanelet::ConstPoint2d& src);

/**
 * [toGeomMsgPt32 converts Eigen::Vector3d(lanelet:BasicPoint3d to
 * geometry_msgs::msg::Point32)]
 * @param src [input point]
 * @param dst [conveted point]
 */
void toGeomMsgPt32(const Eigen::Vector3d& src, geometry_msgs::msg::Point32* dst);

}  // namespace conversion
}  // namespace utils
}  // namespace lanelet

#endif  // AUTOWARE_LANELET2_ROS_INTERFACE_UTILITY_MESSAGE_CONVERSION_H


