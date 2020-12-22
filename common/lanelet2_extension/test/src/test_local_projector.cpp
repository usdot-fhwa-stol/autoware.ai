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

#include <ros/ros.h>

#include <gtest/gtest.h>
#include <math.h>

#include <lanelet2_extension/projection/local_frame_projector.h>

TEST(LocalProjector, ForwardProjection)
{
  std::string map_frame = "+proj=tmerc +lat_0=38.95197911150576 +lon_0=-77.14835128349988 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m +no_defs";
  lanelet::projection::LocalFrameProjector projector(map_frame.c_str());

  lanelet::GPSPoint gps_point;
  gps_point.lat = 38.95197911150576;
  gps_point.lon = -77.14835128349988;
  gps_point.ele = 51.6;
  lanelet::BasicPoint3d local_point = projector.forward(gps_point);

  ASSERT_DOUBLE_EQ(local_point.x(), 0.0);
  ASSERT_DOUBLE_EQ(local_point.y(), 0.0);
  ASSERT_DOUBLE_EQ(local_point.z(), gps_point.ele);
}

TEST(LocalProjector, ReverseProjection)
{
  std::string map_frame = "+proj=tmerc +lat_0=38.95197911150576 +lon_0=-77.14835128349988 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m +no_defs";
  lanelet::projection::LocalFrameProjector projector(map_frame.c_str());

  lanelet::BasicPoint3d local_point;
  local_point.x() = 0.0;
  local_point.y() = 0.0;
  local_point.z() = 51.6;

  lanelet::GPSPoint gps_point = projector.reverse(local_point);

  ASSERT_DOUBLE_EQ(gps_point.lat, 38.95197911150576);
  ASSERT_DOUBLE_EQ(gps_point.lon, -77.14835128349988);
  ASSERT_DOUBLE_EQ(gps_point.ele, local_point.z());
}
