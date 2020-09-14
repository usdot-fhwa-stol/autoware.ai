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
#include <lanelet2_extension/regulatory_elements/StopRule.h>
#include "RegulatoryHelpers.h"

namespace lanelet
{
// C++ 14 vs 17 parameter export
#if __cplusplus < 201703L
constexpr char StopRule::RuleName[];  // instantiate string in cpp file
constexpr char StopRule::Participants[];
#endif

ConstLineStrings3d StopRule::stopAndWaitLine() const
{
  return getParameters<ConstLineString3d>(RoleName::RefLine);
}

LineStrings3d StopRule::stopAndWaitLine()
{
  return getParameters<LineString3d>(RoleName::RefLine);
}

bool StopRule::passable(const std::string& participant) const
{
  return setContainsParticipant(participants_, participant);
}

bool StopRule::boundPassable(const ConstLineString3d& bound,
                                       const std::vector<std::shared_ptr<const StopRule>>& stopAndWaitLines,
                                       const std::string& participant)
{
  for (auto stop_line : stopAndWaitLines)
  {
    for (auto sub_line : stop_line->stopAndWaitLine())
    {
      if (bound.id() == sub_line.id())
      {
        return stop_line->passable(participant);
      }
    }
  }
  return true;
}

bool StopRule::boundPassable(const ConstLineString3d& bound,
                                       const std::vector<std::shared_ptr<StopRule>>& stopAndWaitLines,
                                       const std::string& participant)
{
  return boundPassable(bound, utils::transformSharedPtr<const StopRule>(stopAndWaitLines), participant);
}


StopRule::StopRule(const lanelet::RegulatoryElementDataPtr& data) : RegulatoryElement(data)
{
  // Read participants
  addParticipantsToSetFromMap(participants_, attributes());
}

std::unique_ptr<lanelet::RegulatoryElementData> StopRule::buildData(Id id, LineStrings3d stopAndWaitLine,
                                                                std::vector<std::string> participants)
{
  for (auto ls : stopAndWaitLine)
  {
    if (ls.empty()) throw lanelet::InvalidInputError("Empty linestring was passed into StopRule buildData function");
  }
  
  // Add parameters
  RuleParameterMap rules;

  rules[lanelet::RoleNameString::RefLine].insert(rules[lanelet::RoleNameString::RefLine].end(), stopAndWaitLine.begin(),
                                                 stopAndWaitLine.end());

  // Add attributes
  AttributeMap attribute_map({
      { AttributeNamesString::Type, AttributeValueString::RegulatoryElement },
      { AttributeNamesString::Subtype, RuleName },
  });

  for (auto participant : participants)
  {
    const std::string key = std::string(AttributeNamesString::Participant) + ":" + participant;
    attribute_map[key] = "yes";
  }

  return std::make_unique<RegulatoryElementData>(id, rules, attribute_map);
}

namespace
{
// this object actually does the registration work for us
lanelet::RegisterRegulatoryElement<lanelet::StopRule> reg;
}  // namespace

}  // namespace lanelet
