#pragma once
/*
 * Copyright (C) 2020 LEIDOS.
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

namespace lanelet
{
/**
 * @brief TODO: Represents a  restriction in the form of a line laying on the roadway. Restricts whether a given
 * participant can cross the line from the left or right. General usage is as lane boundaries.
 *
 * A PassingControlLine is created from a list of contiguous LineString3d and participants who are allowed to cross from
 * the left or right. If the control line is representing a lane boundary, each LineString3d parameter should exactly
 * match a right or left bound of an adjacent lanelet. In this fashion, a single regulatory element can represent the
 * lane change restrictions of multiple lanelets while still allowing each lanelet to be associated individually.
 *
 * @ingroup RegulatoryElementPrimitives
 * @ingroup Primitives
 */
class StopRule : public RegulatoryElement
{
public:
  static constexpr char RuleName[] = "stop_rule";
  static constexpr char Participants[] = "participants";
  std::unordered_set<std::string> participants_;

  /**
   * @brief get the list of contigious line strings that form this stop and wait line
   * @return the lines as a list of line strings
   */
  ConstLineStrings3d stopAndWaitLine() const;
  /**
   * @brief Same as ConstLineStrings3d stopAndWaitLine() const but without the const modifier
   * However, the implementation of this method is expected to be const
   */
  LineStrings3d stopAndWaitLine();

  /**
   * @brief Returns true if the provided participant is allowed to cross this stop and wait line
   *
   * @param participant The string classification of the participant type
   *
   * @return True if participant can cross
   */
  bool passable(const std::string& participant) const;

  /**
   * @brief Constructor defined to support loading from lanelet files
   */
  explicit StopRule(const lanelet::RegulatoryElementDataPtr& data);

  /**
   * @brief Static helper function that creates a passing control line data object based on the provided inputs
   *
   * @param id The lanelet::Id of this element
   * @param stopAndWaitLine The line strings which represent this regularoty elements geometry
   * @param participants The set of participants which can cross this line forward
   *
   * @return RegulatoryElementData containing all the necessary information to construct a passing control line
   */
  static std::unique_ptr<lanelet::RegulatoryElementData> buildData(Id id, LineStrings3d stopAndWaitLine,
                                                     std::vector<std::string> participants);

protected:
  // the following lines are required so that lanelet2 can create this object when loading a map with this regulatory
  // element
  friend class RegisterRegulatoryElement<StopRule>;
};

// Convenience Ptr Declarations
using StopRulePtr = std::shared_ptr<StopRule>;
using StopRuleConstPtr = std::shared_ptr<const StopRule>;

}  // namespace lanelet
