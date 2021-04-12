#pragma once
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
#include <lanelet2_core/utility/Units.h>
#include <unordered_set>

namespace lanelet
{
/**
 * @brief Represents a  minimum gap which can be set dynamically either through a V2X communications service or other mechanism..
 *        In a standard use case a digital  minimum gap would be expected to have precedence over a  minimum gap from a
 * sign
 *
 * A digital minimum gap is dynamic and is normally provided through a communications service. This means the minimum gap
 * is stored directly in the regulatory element rather than a TrafficSign element. A  minimum gap is applied
 * uniformly accross all affected lanelets.
 *
 * @ingroup RegulatoryElementPrimitives
 * @ingroup Primitives
 */
class DigitalMinimumGap : public RegulatoryElement
{
public:
  static constexpr char RuleName[] = "digital_minimun_gap";
  static constexpr char MinGap[] = "mingap";
  double min_gap_ = 0;
  std::unordered_set<std::string> participants_;

  /**
   * @brief Returns the lanelets this rule applies to
   *
   * @return Lanelets affected by this access rule
   */
  ConstLanelets getLanelets() const;

  /**
   * @brief Returns the areas this rule applies to
   *
   * @return The areas affected by this rule
   */
  ConstAreas getAreas() const;

  /**
   * @brief Returns the minimum gap  defined by this regulation
   *
   * @return The minimum gap  as a double object
   */
  double getMinimumGap() const;

  /**
   * @brief Returns true if the given participant must follow this minimum gap 
   *
   * @return True if this minimum gap  should apply to the given participant
   */
  bool appliesTo(const std::string& participant) const;

  /**
   * @brief Static helper function that creates a minimum gap  based on the provided double, start, end lines, and the
   * affected participants
   *
   * @param id The lanelet::Id of this object
   * @param min_gap The double which will be treated as the minimum gap (unit of meters) in this region
   * @param lanelets The lanelets this minimum gap  applies to
   * @param areas The areas this minimum gap  applies to
   * @param participants The set of participants which this minimum gap  will apply to
   *
   * @return RegulatoryElementData containing all the necessary information to construct a minimum gap  element
   */
  static std::unique_ptr<lanelet::RegulatoryElementData> buildData(Id id, double min_gap, Lanelets lanelets, Areas areas,
                                                     std::vector<std::string> participants);

  /**
   * @brief Constructor required for compatability with lanlet2 loading
   *
   * @param data The data to initialize this regulation with
   */
  explicit DigitalMinimumGap(const lanelet::RegulatoryElementDataPtr& data);

protected:
  // the following lines are required so that lanelet2 can create this object when loading a map with this regulatory
  // element
  friend class RegisterRegulatoryElement<DigitalMinimumGap>;
};

// Convenience Ptr Declarations
using DigitalMinimumGapPtr = std::shared_ptr<DigitalMinimumGap>;
using DigitalMinimumGapConstPtr = std::shared_ptr<const DigitalMinimumGap>;

}  // namespace lanelet