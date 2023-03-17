#pragma once
/*
 * Copyright (C) 2023 LEIDOS.
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
#include <lanelet2_core/primitives/RegulatoryElement.h>
#include <boost/algorithm/string.hpp>
#include <unordered_set>
#include "StopRule.h"

namespace lanelet
{
/**
 * @brief Represents a virtual bus stop and wait line horizontally laying on the roadway. It indicates whether a given participant bus 
 * should stop and wait momentarily before passing the line. General usage is as a  bus stop line that is not represented by an actual
 * physical roadway object. By default, it only apply to bus in the participant list.
 *
 * A BusStopRule is created from a list of contiguous LineString3d and participant bus which should stop and wait at bus stop before crossing.
 * The object is agnostic to the line's invertedness.
 *
 * @ingroup RegulatoryElementPrimitives
 * @ingroup Primitives
 */
class BusStopRule : public StopRule
{
public:
  static constexpr char RuleName[] = "stop_rule";
  static constexpr char Participants[] = "participants";
  std::unordered_set<std::string> participants_ = {"bus"};

  /**
   * @brief Constructor defined to support loading from lanelet files
   */
  explicit BusStopRule(const lanelet::RegulatoryElementDataPtr& data);

  /**
   * @brief Static helper function that creates a stop line data object based on the provided inputs
   *
   * @param id The lanelet::Id of this element
   * @param stopAndWaitLine The line strings which represents the virtual stop line before with bus will stop at a bus stop
   * @param participants The set of participants which this rule applies to
   *
   * @return RegulatoryElementData containing all the necessary information to construct a stop rule
   */
  static std::unique_ptr<lanelet::RegulatoryElementData> buildData(Id id, LineStrings3d stopAndWaitLine);

protected:
  // the following lines are required so that lanelet2 can create this object when loading a map with this regulatory
  // element
  friend class RegisterRegulatoryElement<StopRule>;
};

// Convenience Ptr Declarations
using BusStopRulePtr = std::shared_ptr<BusStopRule>;
using BusStopRuleConstPtr = std::shared_ptr<const BusStopRule>;

}  // namespace lanelet