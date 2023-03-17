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
#include <lanelet2_extension/regulatory_elements/BusStopRule.h>
#include "RegulatoryHelpers.h"

namespace lanelet
{
// C++ 14 vs 17 parameter export
#if __cplusplus < 201703L
constexpr char BusStopRule::RuleName[];  // instantiate string in cpp file
constexpr char BusStopRule::Participants[];
#endif

BusStopRule::BusStopRule(const lanelet::RegulatoryElementDataPtr& data) : StopRule(data)
{
  // Read participants
  addParticipantsToSetFromMap(participants_, attributes());
}

std::unique_ptr<lanelet::RegulatoryElementData> BusStopRule::buildData(Id id, LineStrings3d stopAndWaitLine)
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

  const std::string key = "participant:vehicle:bus";
  attribute_map[key] = "yes";

  return std::make_unique<RegulatoryElementData>(id, rules, attribute_map);
}

namespace
{
  // this object actually does the registration work for us
  static lanelet::RegisterRegulatoryElement<lanelet::BusStopRule> reg;
}  // namespace

}  // namespace lanelet
