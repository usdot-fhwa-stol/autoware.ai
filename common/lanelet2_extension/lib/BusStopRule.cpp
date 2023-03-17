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
#include <lanelet2_extension/regulatory_elements/StopRule.h>
#include "RegulatoryHelpers.h"

namespace lanelet
{
// C++ 14 vs 17 parameter export
#if __cplusplus < 201703L
constexpr char BusStopRule::RuleName[];  // instantiate string in cpp file
constexpr char BusStopRule::Participants[];
#endif

BusStopRule::StopRule(const lanelet::RegulatoryElementDataPtr& data) : RegulatoryElement(data)
{
  // Read participants
  addParticipantsToSetFromMap(participants_, attributes());
}

namespace
{
  // this object actually does the registration work for us
  static lanelet::RegisterRegulatoryElement<lanelet::BusStopRule> reg;
}  // namespace

}  // namespace lanelet
