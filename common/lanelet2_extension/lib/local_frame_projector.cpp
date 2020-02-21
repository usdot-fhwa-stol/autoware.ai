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

#include <lanelet2_extension/projection/local_frame_projector.h>
#include <iostream>

namespace lanelet
{
namespace projection
{

// C++ 14 vs 17 constant defintion
#if __cplusplus < 201703L
// Forward declare static constexpr
constexpr char LocalFrameProjector::ECEF_PROJ_STR[];  // instantiate string in cpp file
#endif

LocalFrameProjector::LocalFrameProjector(const char* projection_string, Origin origin) : Projector(origin), map_proj_string_(projection_string)
{
  P_ = proj_create(PJ_DEFAULT_CTX, map_proj_string_.c_str());
}

BasicPoint3d LocalFrameProjector::forward(const GPSPoint& p) const
{
  PJ_COORD c{{p.lat, p.lon, p.ele, 0}};
  PJ_COORD c_out = proj_trans(P_, PJ_FWD, c);
  return BasicPoint3d{c_out.xy.x, c_out.xy.y, 0};
}

BasicPoint3d LocalFrameProjector::projectECEF(const BasicPoint3d& p, const int& proj_dir) const
{
  static PJ* ecef_in_map_proj = proj_create_crs_to_crs(PJ_DEFAULT_CTX, map_proj_string_.c_str(), ECEF_PROJ_STR, NULL);
  
  PJ_COORD c{{p[0], p[1], p[2], 0}};
  PJ_COORD c_out;

  if (proj_dir == 1)
    c_out = proj_trans(ecef_in_map_proj, PJ_FWD, c);
  else if (proj_dir == -1)
    c_out = proj_trans(ecef_in_map_proj, PJ_INV, c);
  else
  {
    throw std::invalid_argument(std::string("In function ") + __FUNCTION__ + std::string(": Error:  invalid projection direction: ") + 
    std::to_string(proj_dir) + std::string("; 1 for forward, -1 for reverse."));
  }
  
  return BasicPoint3d{c_out.xyz.x, c_out.xyz.y, c_out.xyz.z};
}

GPSPoint LocalFrameProjector::reverse(const BasicPoint3d& p) const
{
  PJ_COORD c{{p[0], p[1], p[2], 0}};
  PJ_COORD c_out = proj_trans(P_, PJ_INV, c);

  return GPSPoint{c_out.lp.lam, c_out.lp.phi};
}


}  // namespace projection
}  // namespace lanelet

