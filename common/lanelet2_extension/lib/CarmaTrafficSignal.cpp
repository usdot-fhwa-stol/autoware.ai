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

#include <ostream>
#include <lanelet2_extension/regulatory_elements/CarmaTrafficSignal.h>
#include <lanelet2_extension/logging/logger.h>
#include "RegulatoryHelpers.h"

namespace lanelet
{

using namespace lanelet::time;

// C++ 14 vs 17 parameter export
#if __cplusplus < 201703L
constexpr char CarmaTrafficSignal::RuleName[];  // instantiate string in cpp file
#endif

std::ostream& operator<<(std::ostream& os, CarmaTrafficSignalState s)
{
  switch (s)
  {  // clang-format off
    case CarmaTrafficSignalState::UNAVAILABLE   : os << "CarmaTrafficSignalState::UNAVAILABLE"; break;
    case CarmaTrafficSignalState::DARK: os << "CarmaTrafficSignalState::DARK"; break;
    case CarmaTrafficSignalState::STOP_THEN_PROCEED : os << "CarmaTrafficSignalState::STOP_THEN_PROCEED"; break;
    case CarmaTrafficSignalState::STOP_AND_REMAIN  : os << "CarmaTrafficSignalState::STOP_AND_REMAIN"; break;
    case CarmaTrafficSignalState::PRE_MOVEMENT  : os << "CarmaTrafficSignalState::PRE_MOVEMENT"; break;
    case CarmaTrafficSignalState::PERMISSIVE_MOVEMENT_ALLOWED  : os << "CarmaTrafficSignalState::PERMISSIVE_MOVEMENT_ALLOWED"; break;
    case CarmaTrafficSignalState::PROTECTED_MOVEMENT_ALLOWED  : os << "CarmaTrafficSignalState::PROTECTED_MOVEMENT_ALLOWED"; break;
    case CarmaTrafficSignalState::PERMISSIVE_CLEARANCE  : os << "CarmaTrafficSignalState::PERMISSIVE_CLEARANCE"; break;
    case CarmaTrafficSignalState::PROTECTED_CLEARANCE  : os << "CarmaTrafficSignalState::PROTECTED_CLEARANCE"; break;
    case CarmaTrafficSignalState::CAUTION_CONFLICTING_TRAFFIC  : os << "CarmaTrafficSignalState::CAUTION_CONFLICTING_TRAFFIC"; break;
    default: os.setstate(std::ios_base::failbit);
  }  // clang-format on
  return os;
}

ConstLineStrings3d CarmaTrafficSignal::stopLine() const
{
  return getParameters<ConstLineString3d>(RoleName::RefLine);
}

LineStrings3d CarmaTrafficSignal::stopLine()
{
  return getParameters<LineString3d>(RoleName::RefLine);
}

CarmaTrafficSignal::CarmaTrafficSignal(const lanelet::RegulatoryElementDataPtr& data) : RegulatoryElement(data)
{}

std::unique_ptr<lanelet::RegulatoryElementData> CarmaTrafficSignal::buildData(Id id, LineString3d stop_line, Lanelets lanelets)
{

  if (stop_line.empty()) throw lanelet::InvalidInputError("Empty linestring was passed into CarmaTrafficSignal buildData function");
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

boost::optional<CarmaTrafficSignalState> CarmaTrafficSignal::predictState(boost::posix_time::ptime time_stamp)
{
  if (recorded_time_stamps.empty())
  {
    LOG_WARN_STREAM("CarmaTrafficSignal doesn't have any recorded states of traffic lights");
    return boost::none;
  }
  if (recorded_time_stamps.size() == 1) // if only 1 timestamp recorded, this signal doesn't change
  {
    return recorded_time_stamps.front().second;
  }
  // shift starting time to the future or to the past to fit input into a valid cycle
  boost::posix_time::time_duration accumulated_offset_duration;
  double offset_duration_dir = recorded_time_stamps.front().first > time_stamp ? -1.0 : 1.0; // -1 if past, +1 if time_stamp is in future

  int num_of_cycles = std::abs(toSec(recorded_time_stamps.front().first - time_stamp) / toSec(fixed_cycle_duration));
  accumulated_offset_duration = durationFromSec( num_of_cycles * toSec(fixed_cycle_duration));
  
  if (offset_duration_dir < 0) 
  {
    while (recorded_time_stamps.front().first - accumulated_offset_duration > time_stamp)
    {
      accumulated_offset_duration += fixed_cycle_duration;
    }
  }
  // iterate through states in the cycle to get the signal
  for (size_t i = 0; i < recorded_time_stamps.size(); i++)
  {
    if (toSec(recorded_time_stamps[i].first) + offset_duration_dir * toSec(accumulated_offset_duration) >= toSec(time_stamp))
    { 
      return recorded_time_stamps[i].second;
    }
  }

  throw lanelet::InvalidInputError("Reached unreachable code block. Implies duplicate phase is not provided. Unable to determine fixed cycle duration");
}

lanelet::ConstLanelets CarmaTrafficSignal::getControlledLanelets() const
{
  return getParameters<lanelet::ConstLanelet>(RoleName::Refers);
} 

void CarmaTrafficSignal::setStates(std::vector<std::pair<boost::posix_time::ptime, CarmaTrafficSignalState>> input_time_steps, int revision)
{
  if (input_time_steps.empty())
  {
    LOG_ERROR_STREAM("Given states for the CarmaTrafficSignal Id: " << id() << " is empty. Returning...");
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
  // throw where the duplicate phase is not provided
  if (input_time_steps.back().second != input_time_steps.front().second)
  {
    throw lanelet::InvalidInputError("Duplicate phase is not provided. Unable to determine fixed cycle duration");
  }

  recorded_time_stamps = input_time_steps;
  fixed_cycle_duration = recorded_time_stamps.back().first - recorded_time_stamps.front().first; // it is okay if size is only 1, case is handled in predictState
  revision_ = revision;
}


namespace time {

double toSec(const boost::posix_time::time_duration& duration) {
  if (duration.is_special()) {
    throw std::invalid_argument("Cannot convert special duration to seconds");
  }
  return duration.total_microseconds() / 1000000.0;
}

double toSec(const boost::posix_time::ptime& time) {
  // TODO clean up this comment boost::posix_time::time_duration duration = t - boost::posix_time::from_time_t(0);
  return toSec(time - boost::posix_time::from_time_t(0));
}

boost::posix_time::ptime timeFromSec(double sec) {
  return boost::posix_time::from_time_t(0) + boost::posix_time::microseconds(static_cast<long>(sec * 1000000L));
}

boost::posix_time::time_duration durationFromSec(double sec) {
  return boost::posix_time::microseconds(static_cast<long>(sec * 1000000L));
}

} // namespace time



namespace
{
// this object actually does the registration work for us
lanelet::RegisterRegulatoryElement<lanelet::CarmaTrafficSignal> reg;
}  // namespace

}  // namespace lanelet