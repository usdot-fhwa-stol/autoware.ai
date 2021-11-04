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
    constexpr const char CarmaRoleNameString::IntersectionExit[];
    constexpr const char CarmaRoleNameString::IntersectionInterior[];
    #endif

    namespace signalized_intersection
    {
        template <typename T>
        RuleParameters toRuleParameters(const std::vector<T>& primitives)
        {
        auto cast_func = [](const auto& elem) { return static_cast<RuleParameter>(elem); };
        return utils::transform(primitives, cast_func);
        }

        //template <>
        //RuleParameters toRuleParameters(const std::vector<LineStringOrPolygon3d>& primitives)
        //{
        //auto cast_func = [](const auto& elem) { return elem.asRuleParameter(); };
        //return utils::transform(primitives, cast_func);
        //}
    }

    ConstLanelets SignalizedIntersection::getEntryLanelets() const {return getParameters<ConstLanelet>(RoleName::Refers);}

    //Lanelets SignalizedIntersection::getEntryLanelets() {return getParameters<Lanelet>(RoleName::Refers);} TODO: investigate if it is really not possible 

    ConstLanelets SignalizedIntersection::getExitLanelets() const 
    {
        // TODO: is there a faster way?
        ConstLanelets const_lanelets;
        const_lanelets.insert(const_lanelets.end(), exit_lanelets.begin(), exit_lanelets.end());
        return const_lanelets;
    }
    Lanelets SignalizedIntersection::getExitLanelets() 
    {
        return exit_lanelets;
    }

    ConstLanelets SignalizedIntersection::getInteriorLanelets() const 
    {
        // TODO: is there a faster way?
        ConstLanelets const_lanelets;
        const_lanelets.insert(const_lanelets.end(), interior_lanelets.begin(), interior_lanelets.end());
        return const_lanelets;
    }
    Lanelets SignalizedIntersection::getInteriorLanelets() 
    {
        return interior_lanelets;
    }

    std::unique_ptr<lanelet::RegulatoryElementData> SignalizedIntersection::buildData(Id id, Lanelets entry, Lanelets exit,
                                                                Lanelets interior)
    {
        // Add parameters
        std::cerr << "Intersection Called~" << std::endl;
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
            {
                auto entry_lanelets = getEntryLanelets();
                if (std::find(entry_lanelets.begin(), entry_lanelets.end(), lanelet) == entry_lanelets.end())
                    parameters()[RoleName::Refers].emplace_back(lanelet);
                break;
            }
            case IntersectionSection::EXIT:
                if (std::find(exit_lanelets.begin(), exit_lanelets.end(), lanelet) == exit_lanelets.end())
                    exit_lanelets.push_back(lanelet);
                break;
            case IntersectionSection::INTERIOR:
                if (std::find(interior_lanelets.begin(), interior_lanelets.end(), lanelet) == interior_lanelets.end())
                    interior_lanelets.push_back(lanelet);
                break;
            default:
                throw std::invalid_argument("Invalid section is passed as IntersectionSection");
        }
        
    }
    
    bool SignalizedIntersection::removeLanelet(const Lanelet& llt)
    {
        auto exit_it = std::find(exit_lanelets.begin(), exit_lanelets.end(), llt);
        if (exit_it != exit_lanelets.end())
        {
            exit_lanelets.erase(exit_it);
            return true;
        }     
        ConstLanelet const_llt = llt;
        auto entry_lanelets = getEntryLanelets();
        auto lltIt = std::find(entry_lanelets.begin(), entry_lanelets.end(), const_llt); //this works because == operator compares constData()
        if (lltIt != entry_lanelets.end()) {
            // TODO remove it from parameter
            return true;
        }
        
        auto interior_it = std::find(interior_lanelets.begin(), interior_lanelets.end(), llt);
        if (interior_it != interior_lanelets.end())
        {
            interior_lanelets.erase(interior_it);
            return true;
        }  
        
        return false;
    }

    RegulatoryElementDataPtr constructSignalizedIntersectionData(Id id, const AttributeMap& attributes, Lanelets entry, Lanelets exit, Lanelets interior)
    {
        std::cerr << "Intersection 3 Called~ Data creation" << std::endl;
        RuleParameterMap rpm;
        rpm[lanelet::RoleNameString::Refers].insert(rpm[lanelet::RoleNameString::Refers].end(), entry.begin(),
                                                        entry.end());
        rpm[lanelet::CarmaRoleNameString::IntersectionExit].insert(rpm[lanelet::CarmaRoleNameString::IntersectionExit].end(), exit.begin(),
                                                        exit.end());
        rpm[lanelet::CarmaRoleNameString::IntersectionInterior].insert(rpm[lanelet::CarmaRoleNameString::IntersectionInterior].end(), interior.begin(),
                                                        interior.end());

        auto data = std::make_shared<RegulatoryElementData>(id, std::move(rpm), attributes);
        data->attributes[AttributeName::Type] = AttributeValueString::RegulatoryElement;
        data->attributes[AttributeName::Subtype] = SignalizedIntersection::RuleName;
        return data;
    }
    
    SignalizedIntersection::SignalizedIntersection(Id id, const AttributeMap& attributes, Lanelets entry, Lanelets exit, Lanelets interior):
        SignalizedIntersection(constructSignalizedIntersectionData(id, attributes, entry, exit, interior)) {
        std::cerr << "Intersection 2 Called~" << std::endl;
    }

    SignalizedIntersection::SignalizedIntersection(const lanelet::RegulatoryElementDataPtr& data) : RegulatoryElement(data) {
        std::cerr << "Intersection 1 Called~" << std::endl;
        std::cerr << "parameters size: " << data->parameters.size() << std::endl;
    }

    namespace
    {
    // this object actually does the registration work for us
    lanelet::RegisterRegulatoryElement<lanelet::SignalizedIntersection> reg;
    }  // namespace

} //namespace lanelet