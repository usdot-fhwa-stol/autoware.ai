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
 *
 * Authors: Shuwei Qiang
 */

#ifndef LANELET2_EXTENSION_PROJECTION_LOCAL_FRAME_PROJECTOR_H
#define LANELET2_EXTENSION_PROJECTION_LOCAL_FRAME_PROJECTOR_H

#include <proj.h>
#include <lanelet2_io/Projection.h>

#include <string>

namespace lanelet
{
namespace projection
{

class LocalFrameProjector : public Projector
{

public:

  explicit LocalFrameProjector(const char* base_frame, const char* target_frame, Origin origin = Origin({ 0.0, 0.0 }));

  /**
   * [LocalFrameProjector::forward projects gps lat/lon to local map frame]
   * @param  gps [point with latitude longitude information]
   * @return     [projected point in local map coordinate]
   */
  BasicPoint3d forward(const GPSPoint& p) const override;

  /**
   * [LocalFrameProjector::forward projects between ECEF and local map/ from base to target]
   * @param  ecef_point [point with x,y,z in ecef information]
   * @param  proj_dir   [1 for forward -1 for reverse]
   * @return            [projected point in local map coordinate]
   */
  BasicPoint3d project(const BasicPoint3d& p, const int& proj_dir) const;

  /**
   * [LocalFrameProjector::reverse projects point within local map frame into gps lat/lon (WGS84)]
   * @param  local_point [3d point in local map frame]
   * @return             [projected point in WGS84]
   */
  GPSPoint reverse(const BasicPoint3d& p) const override;

private:
  
  PJ *P;
};

}  // namespace projection
}  // namespace lanelet

#endif  // LANELET2_EXTENSION_PROJECTION_LOCAL_FRAME_PROJECTOR_H
