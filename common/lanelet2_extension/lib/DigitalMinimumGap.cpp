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
#include <lanelet2_extension/regulatory_elements/DigitalMinimumGap.h>
#include "RegulatoryHelpers.h"

namespace lanelet
{
    // C++ 14 vs 17 constant defintion
    #if __cplusplus < 201703L
    // Forward declare static constexpr
    constexpr char DigitalMinimumGap::RuleName[];  // instantiate string in cpp file
    constexpr char DigitalMinimumGap::MinGap[];
    #endif

    ConstLanelets DigitalMinimumGap::getLanelets() const
    {
    return getParameters<ConstLanelet>(RoleName::Refers);
    }

    ConstAreas DigitalMinimumGap::getAreas() const
    {
        return getParameters<ConstArea>(RoleName::Refers);
    }

    double DigitalMinimumGap::getMinimumGap() const
    {
        return min_gap_;
    }

    bool DigitalMinimumGap::appliesTo(const std::string& participant) const
    {
        return setContainsParticipant(participants_, participant);
    }
       
    std::unique_ptr<lanelet::RegulatoryElementData> DigitalMinimumGap::buildData(Id id, double min_gap, Lanelets lanelets,
                                                                Areas areas, std::vector<std::string> participants)
    {
        // Add parameters
        RuleParameterMap rules;
        rules[lanelet::RoleNameString::Refers].insert(rules[lanelet::RoleNameString::Refers].end(), lanelets.begin(),
                                                        lanelets.end());
        rules[lanelet::RoleNameString::Refers].insert(rules[lanelet::RoleNameString::Refers].end(), areas.begin(),
                                                        areas.end());

        // Add attributes
        AttributeMap attribute_map({ { AttributeNamesString::Type, AttributeValueString::RegulatoryElement },
                                    { AttributeNamesString::Subtype, RuleName },
                                    { MinGap, Attribute(min_gap).value() } });

        for (auto participant : participants)
        {
            const std::string key = std::string(AttributeNamesString::Participant) + ":" + participant;
            attribute_map[key] = "yes";
        }

        return std::make_unique<RegulatoryElementData>(id, rules, attribute_map);
    }


    DigitalMinimumGap::DigitalMinimumGap(const lanelet::RegulatoryElementDataPtr& data) : RegulatoryElement(data)
    {
        // Read participants
        addParticipantsToSetFromMap(participants_, attributes());

        // Read min gap
        auto optional_min_gap = attribute(MinGap).asDouble();

        if (!optional_min_gap)
        {
            throw std::invalid_argument("MinGap attribute of DigitalMinimumGap regulatory element is not set or cannot be "
                                        "read ");
        }

        min_gap_ = *optional_min_gap;
    }


    namespace
    {
    // this object actually does the registration work for us
    lanelet::RegisterRegulatoryElement<lanelet::DigitalMinimumGap> reg;
    }  // namespace

} //namespace lanelet