/*
 * Copyright (C) 2021 LEIDOS.
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

#include <lanelet2_extension/regulatory_elements/CarmaTrafficLight.h>
#include "RegulatoryHelpers.h"

namespace lanelet
{
// C++ 14 vs 17 parameter export
#if __cplusplus < 201703L
constexpr char CarmaTrafficLight::RuleName[];  // instantiate string in cpp file
#endif

ConstLineStrings3d CarmaTrafficLight::stopLine() const
{
  return getParameters<ConstLineString3d>(RoleName::RefLine);
}

LineStrings3d CarmaTrafficLight::stopLine()
{
  return getParameters<LineString3d>(RoleName::RefLine);
}

CarmaTrafficLight::CarmaTrafficLight(const lanelet::RegulatoryElementDataPtr& data) : RegulatoryElement(data)
{}

std::unique_ptr<lanelet::RegulatoryElementData> CarmaTrafficLight::buildData(Id id, LineString3d stop_line, Lanelets lanelets)
{

  if (stop_line.empty()) throw lanelet::InvalidInputError("Empty linestring was passed into CarmaTrafficLight buildData function");
  // Add parameters
  RuleParameterMap rules;
  rules[lanelet::RoleNameString::Refers].insert(rules[lanelet::RoleNameString::Refers].end(), lanelets.begin(),
                                                lanelets.end());
  rules[lanelet::RoleNameString::RefLine].insert(rules[lanelet::RoleNameString::RefLine].end(), stop_line);

  // Add attributes
  AttributeMap attribute_map({
      { AttributeNamesString::Type, AttributeValueString::RegulatoryElement },
      { AttributeNamesString::Subtype, RuleName },
  });

  return std::make_unique<RegulatoryElementData>(id, rules, attribute_map);
}

boost::optional<CarmaTrafficLightState> CarmaTrafficLight::getState()
{
  return predictState(ros::Time::now());
}

boost::optional<CarmaTrafficLightState> CarmaTrafficLight::predictState(ros::Time time_stamp)
{
  if (recorded_time_stamps.empty())
  {
    ROS_WARN_STREAM("CarmaTrafficLight doesn't have any recorded states of traffic lights");
    return boost::none;
  }
  if (recorded_time_stamps.size() == 1) // if only 1 timestamp recorded, this signal doesn't change
  {
    return recorded_time_stamps.front().second;
  }
  // shift starting time to the future or to the past to fit input into a valid cycle
  ros::Duration accumulated_offset_duration;
  double offset_duration_dir = recorded_time_stamps.front().first > time_stamp ? -1.0 : 1.0; // -1 if past, +1 if time_stamp is in future

  if (offset_duration_dir < 0) 
  {
    while (recorded_time_stamps.front().first - accumulated_offset_duration > time_stamp)
    {
      accumulated_offset_duration += fixed_cycle_duration;
    }
  }
  else
  {
    while (recorded_time_stamps.back().first + accumulated_offset_duration < time_stamp)
    {
      accumulated_offset_duration += fixed_cycle_duration;
    }
  }
  
  // iterate through states in the cycle to get the signal
  for (size_t i = 0; i < recorded_time_stamps.size(); i++)
  {
    if (recorded_time_stamps[i].first.toSec() + offset_duration_dir * accumulated_offset_duration.toSec() >= time_stamp.toSec())
    { 
      return recorded_time_stamps[i].second;
    }
  }
}

lanelet::ConstLanelets CarmaTrafficLight::getControlledLanelets() const
{
  return getParameters<lanelet::ConstLanelet>(RoleName::Refers);
} 

void CarmaTrafficLight::setStates(std::vector<std::pair<ros::Time, CarmaTrafficLightState>> input_time_steps, int revision)
{
  if (input_time_steps.empty())
  {
    ROS_ERROR_STREAM("Given states for the CarmaTrafficLight Id: " << id() << " is empty. Returning...");
    return;
  }

  std::sort(input_time_steps.begin(), input_time_steps.end());

  //extract a cycle and trim the rest from the states
  if (input_time_steps.size() > 2)
  {
    int idx = 2;
    while (idx + 1 < input_time_steps.size() && input_time_steps[idx].second != input_time_steps[0].second && input_time_steps[idx + 1].second != input_time_steps[1].second)
    {
      idx ++;
    }
    input_time_steps.resize(idx + 1);
  }
  
  recorded_time_stamps = input_time_steps;
  fixed_cycle_duration = recorded_time_stamps.back().first - recorded_time_stamps.front().first; // it is okay if size is only 1, case is handled in predictState
  revision_ = revision;
}
namespace
{
// this object actually does the registration work for us
lanelet::RegisterRegulatoryElement<lanelet::CarmaTrafficLight> reg;
}  // namespace

}  // namespace lanelet