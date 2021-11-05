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
#include <lanelet2_core/primitives/RegulatoryElement.h>
#include <boost/algorithm/string.hpp>
#include <lanelet2_extension/regulatory_elements/SignalizedIntersection.h>
#include "RegulatoryHelpers.h"

namespace lanelet
{
    // C++ 14 vs 17 constant defintion
    #if __cplusplus < 201703L
    // Forward declare static constexpr
    constexpr char SignalizedIntersection::RuleName[];  // instantiate string in cpp file
    constexpr const char CarmaRoleNameString::IntersectionEntry[];
    constexpr const char CarmaRoleNameString::IntersectionExit[];
    constexpr const char CarmaRoleNameString::IntersectionInterior[];
    #endif

    ConstLanelets SignalizedIntersection::getEntryLanelets() const {return getParameters<ConstLanelet>(RoleName::Refers);}

    ConstLanelets SignalizedIntersection::getExitLanelets() const {return getParameters<ConstLanelet>(CarmaRoleNameString::IntersectionExit);}

    ConstLanelets SignalizedIntersection::getInteriorLanelets() const {return getParameters<ConstLanelet>(CarmaRoleNameString::IntersectionInterior);}

    std::unique_ptr<lanelet::RegulatoryElementData> SignalizedIntersection::buildData(Id id, const Lanelets& entry, const Lanelets& exit,
                                                                const Lanelets& interior)
    {
        // Add parameters
        RuleParameterMap rules;
        rules[lanelet::RoleNameString::Refers].insert(rules[lanelet::RoleNameString::Refers].end(), entry.begin(),
                                                        entry.end());
        rules[lanelet::CarmaRoleNameString::IntersectionExit].insert(rules[lanelet::CarmaRoleNameString::IntersectionExit].end(), exit.begin(),
                                                        exit.end());
        rules[lanelet::CarmaRoleNameString::IntersectionInterior].insert(rules[lanelet::CarmaRoleNameString::IntersectionInterior].end(), interior.begin(),
                                                        interior.end());

        // Add attributes
        AttributeMap attribute_map({ { AttributeNamesString::Type, AttributeValueString::RegulatoryElement },
                                    { AttributeNamesString::Subtype, RuleName }});


        return std::make_unique<RegulatoryElementData>(id, rules, attribute_map);
    }

    std::vector<CarmaTrafficLightConstPtr> SignalizedIntersection::getTrafficSignals(const ConstLanelet& llt) const
    {
        return llt.regulatoryElementsAs<CarmaTrafficLight>();
    }

    void SignalizedIntersection::addLanelet(const Lanelet& lanelet, IntersectionSection section)
    {
        switch (section)
        {
            case IntersectionSection::ENTRY:
                parameters()[RoleNameString::Refers].emplace_back(RuleParameter(lanelet));
                break;
            case IntersectionSection::EXIT:
                parameters()[CarmaRoleNameString::IntersectionExit].emplace_back(RuleParameter(lanelet));
                break;
            case IntersectionSection::INTERIOR:
                parameters()[CarmaRoleNameString::IntersectionInterior].emplace_back(RuleParameter(lanelet));
                break;
            default:
                throw std::invalid_argument("Invalid section is passed as IntersectionSection");
        }
    }
    
    namespace {

        template <typename T>
        bool findAndErase(const T& primitive, RuleParameters* member)
        {
        if (member == nullptr)
        {
            std::cerr << __FUNCTION__ << ": member is null pointer";
            return false;
        }
        auto it = std::find(member->begin(), member->end(), RuleParameter(primitive));
        if (it == member->end())
        {
            return false;
        }
        member->erase(it);
        return true;
        }
    }

    bool SignalizedIntersection::removeLanelet(const Lanelet& llt)
    {
        // if successfully found and erased, then return true, else false
        if (findAndErase(RuleParameter(llt), &parameters().find(RoleNameString::Refers)->second))   
            return true;
        if (findAndErase(RuleParameter(llt), &parameters().find(CarmaRoleNameString::IntersectionExit)->second))
            return true;
        if (findAndErase(RuleParameter(llt), &parameters().find(CarmaRoleNameString::IntersectionInterior)->second))
            return true;
        
        return false;
    }

    Optional<lanelet::ConstLineString3d> SignalizedIntersection::getStopLine(const ConstLanelet& llt) const
    {
        Optional<lanelet::ConstLineString3d> stop_line = boost::none;
        // stop line geometry should be same for any traffic signals
        auto traffic_signals = llt.regulatoryElementsAs<CarmaTrafficLight>();
        if (!traffic_signals.empty())
            stop_line = traffic_signals.front()->stopLine().front();

        return stop_line;
    }   

    SignalizedIntersection::SignalizedIntersection(const lanelet::RegulatoryElementDataPtr& data) : RegulatoryElement(data) {
        // use only RoleName::Refers to avoid data inconsistency 
        data->parameters[lanelet::RoleNameString::Refers].insert(data->parameters[lanelet::RoleNameString::Refers].end(), 
                                                        data->parameters[CarmaRoleNameString::IntersectionEntry].begin(),
                                                        data->parameters[CarmaRoleNameString::IntersectionEntry].end());
        data->parameters[CarmaRoleNameString::IntersectionEntry].clear();
    }

    namespace
    {
    // this object actually does the registration work for us
    lanelet::RegisterRegulatoryElement<lanelet::SignalizedIntersection> reg;
    }  // namespace

} //namespace lanelet