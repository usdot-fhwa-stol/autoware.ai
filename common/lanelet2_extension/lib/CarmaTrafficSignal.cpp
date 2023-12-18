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
constexpr const char CarmaTrafficSignalRoleNameString::ControlStart[];
constexpr const char CarmaTrafficSignalRoleNameString::ControlEnd[];
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

Optional<LineString3d> CarmaTrafficSignal::getStopLine(const ConstLanelet& llt)
{
  auto sl = stopLine();
  if (sl.empty()) {
    return boost::none;
  }
  lanelet::ConstLanelets llts = getControlStartLanelets();
  if (llts.empty())
  {
    return boost::none;
  }
  auto it = std::find(llts.begin(), llts.end(), llt);
  if (it == llts.end()) {
    return boost::none;
  }
  return sl.at(size_t(std::distance(llts.begin(), it)));
}

Optional<ConstLineString3d> CarmaTrafficSignal::getConstStopLine(const ConstLanelet& llt)
{
  Optional<LineString3d> mutable_stop_line = getStopLine(llt);

  if (!mutable_stop_line)
    return boost::none;

  ConstLineString3d const_stop_line = mutable_stop_line.get();

  return const_stop_line;
}

CarmaTrafficSignal::CarmaTrafficSignal(const lanelet::RegulatoryElementDataPtr& data) : RegulatoryElement(data)
{}

std::unique_ptr<lanelet::RegulatoryElementData> CarmaTrafficSignal::buildData(Id id, LineStrings3d stop_lines, Lanelets entry_lanelets, Lanelets exit_lanelets)
{
  // Add parameters
  RuleParameterMap rules;
  rules[lanelet::CarmaTrafficSignalRoleNameString::ControlStart].insert(rules[lanelet::CarmaTrafficSignalRoleNameString::ControlStart].end(), entry_lanelets.begin(),
                                                entry_lanelets.end());
  rules[lanelet::CarmaTrafficSignalRoleNameString::ControlEnd].insert(rules[lanelet::CarmaTrafficSignalRoleNameString::ControlEnd].end(), exit_lanelets.begin(),
                                                exit_lanelets.end());
  rules[lanelet::RoleNameString::RefLine].insert(rules[lanelet::RoleNameString::RefLine].end(), stop_lines.begin(), stop_lines.end());

  // Add attributes
  AttributeMap attribute_map({
      { AttributeNamesString::Type, AttributeValueString::RegulatoryElement },
      { AttributeNamesString::Subtype, RuleName },
  });

  return std::make_unique<RegulatoryElementData>(id, rules, attribute_map);
}

boost::optional<std::pair<boost::posix_time::ptime, CarmaTrafficSignalState>> CarmaTrafficSignal::predictState(boost::posix_time::ptime time_stamp)
{
  if (recorded_time_stamps.empty())
  {
    LOG_WARN_STREAM("CarmaTrafficSignal doesn't have any recorded states of traffic lights");
    return boost::none;
  }

  if (lanelet::time::toSec(fixed_cycle_duration) < 1.0) // there are recorded states, but no fixed_cycle_duration means it is dynamic
  {
    if (recorded_time_stamps.size() != recorded_start_time_stamps.size())
    {
      throw std::invalid_argument("recorded_start_time_stamps size is not equal to recorded_time_stamps size");
    }

    // if requested time is BEFORE recorded states' time interval, return STOP_AND_REMAIN
    if (recorded_start_time_stamps.front() >= time_stamp)
    {
      return std::pair<boost::posix_time::ptime, CarmaTrafficSignalState>(recorded_start_time_stamps.front(), CarmaTrafficSignalState::STOP_AND_REMAIN);
    }

    // if requested time is AFTER recorded states' time interval, return STOP_AND_REMAIN
    if (recorded_time_stamps.back().first <= time_stamp)
    {
      auto end_infinity_time = timeFromSec(INFINITY_END_TIME_FOR_NOT_ENOUGH_STATES); //corresponds to 03:14:07 on Tuesday, 19 January 2038.
      LOG_DEBUG_STREAM("CarmaTrafficSignal doesn't have enough state saved, so returning STOP_AND_REMAIN state! End_time: " << end_infinity_time);
      return std::pair<boost::posix_time::ptime, CarmaTrafficSignalState>(end_infinity_time, CarmaTrafficSignalState::STOP_AND_REMAIN);
    }

    // iterate through states in the dynamic states to get the signal.
    for (size_t i = 0; i < recorded_time_stamps.size(); i++)
    {
      if (recorded_time_stamps[i].first >= time_stamp)
      {
        return std::pair<boost::posix_time::ptime, CarmaTrafficSignalState>(recorded_time_stamps[i].first, recorded_time_stamps[i].second);
      }
    }
  }

  // This part of the code is used for predicting state if fixed_cycle_duration is set using setStates function
  // shift starting time to the future or to the past to fit input into a valid cycle
  boost::posix_time::time_duration accumulated_offset_duration;
  double offset_duration_dir = recorded_time_stamps.front().first > time_stamp ? -1.0 : 1.0; // -1 if past, +1 if time_stamp is in future

  int num_of_cycles = std::abs(lanelet::time::toSec(recorded_time_stamps.front().first - time_stamp) / lanelet::time::toSec(fixed_cycle_duration));
  accumulated_offset_duration = durationFromSec( num_of_cycles * lanelet::time::toSec(fixed_cycle_duration));

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
    double end_time = lanelet::time::toSec(recorded_time_stamps[i].first) + offset_duration_dir * lanelet::time::toSec(accumulated_offset_duration);
    if (end_time >= lanelet::time::toSec(time_stamp))
    {
      return std::pair<boost::posix_time::ptime, CarmaTrafficSignalState>(timeFromSec(end_time), recorded_time_stamps[i].second);
    }
  }

  throw lanelet::InvalidInputError("Reached unreachable code block. Implies duplicate phase is not provided. Unable to determine fixed cycle duration");
}

lanelet::ConstLanelets CarmaTrafficSignal::getControlStartLanelets() const
{
  return getParameters<ConstLanelet>(CarmaTrafficSignalRoleNameString::ControlStart);
}

lanelet::ConstLanelets CarmaTrafficSignal::getControlEndLanelets() const
{
  return getParameters<ConstLanelet>(CarmaTrafficSignalRoleNameString::ControlEnd);
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

  for (size_t i = 0; i < input_time_steps.size() - 1; i++)
  {
    signal_durations[input_time_steps[i + 1].second] = input_time_steps[i + 1].first - input_time_steps[i].first;
  }

  recorded_time_stamps = input_time_steps;
  fixed_cycle_duration = recorded_time_stamps.back().first - recorded_time_stamps.front().first; // it is okay if size is only 1, case is handled in predictState
  revision_ = revision;
}

namespace
{
// this object actually does the registration work for us
lanelet::RegisterRegulatoryElement<lanelet::CarmaTrafficSignal> reg;
}  // namespace

}  // namespace lanelet
