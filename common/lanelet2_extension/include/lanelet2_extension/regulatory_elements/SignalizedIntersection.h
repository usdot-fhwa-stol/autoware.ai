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
#include <lanelet2_extension/regulatory_elements/CarmaTrafficLight.h>
#include <unordered_set>

namespace lanelet
{
/**
 * @brief Represents signalized intersection on the road. 
 *
 * SignalizedIntersection consists of ENTRY, EXIT, INTERIOR lanelets that are saved as parameters. 
 * Although this class doesn't manage it, entry lanelets are expected to have CarmaTrafficSignal class to represent the traffic light.
 * Therefore, this class can be understood as merely recording of which lanelets are part of the intersection, but traffic signal timer
 * or which entry correlates to which exit information is handled by each CarmaTrafficSignal object itself.
 *
 * @ingroup RegulatoryElementPrimitives
 * @ingroup Primitives
 */
enum class IntersectionSection {ENTRY, EXIT, INTERIOR};

struct CarmaRoleNameString
{
  static constexpr char IntersectionEntry[] = "intersection_entry"; //saved and used internally as RoleName::Refers
  static constexpr char IntersectionExit[] = "intersection_exit";  
  static constexpr char IntersectionInterior[] = "intersection_interior";  
};

class SignalizedIntersection : public RegulatoryElement
{
public:
  static constexpr char RuleName[] = "signalized_intersection";

  /**
   * @brief Returns the entry lanelets into the signalized intersection
   *
   * @return list of const lanelets
   */
  ConstLanelets getEntryLanelets() const;

    /**
   * @brief Returns the interior lanelets of signalized intersection
   *
   * @return list of const lanelets
   */
  ConstLanelets getInteriorLanelets() const;

  /**
   * @brief Returns the exit lanelets into the signalized intersection
   *
   * @return list of const lanelets
   */
  ConstLanelets getExitLanelets() const;

  /**
   * @brief Returns the carma traffic signals in the specified entry lanelet in the intersection
   * @param entry lanelet in the intersection to get the traffic signals for
   *
   * @return vector of const traffic signals. Empty list if the lanelet is not an entry lanelet or not in the intersection
   */
  std::vector<CarmaTrafficLightConstPtr> getTrafficSignals(const ConstLanelet& llt) const;

  /**
   * @brief Returns the stop line of the specified lanelet in the intersection
   *
   * @return gets a const stop line for a lanelet, if there is one
   */
  Optional<lanelet::ConstLineString3d> getStopLine(const ConstLanelet& llt) const;

  /**
   * @brief Adds the lanelet into the specified section of the intersection
   * @param lanelet to add  
   * @param section section of the intersection to add to (entry, exit, interior)
   * 
   * NOTE: If entry lanelet is being added, the user must make sure that the regulatory element refers the lanelet back
   *       For maps, laneletMapPtr->update(lanelet, regem) for full connection
   */
  void addLanelet(const Lanelet& lanelet, IntersectionSection section);

  /**
   * @brief Removes the lanelet from the intersection
   * @param lanelet to remove
   * @return true if successful, false if lanelet was not found  
   * 
   * NOTE: If entry lanelet is being removed, the user must make sure that they remove the regulatory element from the lanelet
   *       For maps, laneletMapPtr->remove(lanelet, regem) for removing all connection
   */
  bool removeLanelet(const Lanelet& lanelet);

  /**
   * @brief Static helper function that creates a minimum gap  based on the provided double, start, end lines, and the
   * affected participants
   *
   * @param id The lanelet::Id of this object
   * @param entry Entry lanelets of the intersection (which this regem also refers)
   * @param exit Exit lanelets of the intersection
   * @param interior Interior lanelets of the intersection
   *
   * @return RegulatoryElementData containing all the necessary information to construct a minimum gap  element
   */
  static std::unique_ptr<lanelet::RegulatoryElementData> buildData(Id id, const Lanelets& entry, const Lanelets& exit, const Lanelets& interior);

  /**
   * @brief Constructor required for compatability with lanelet2 loading
   *
   * @param data The data to initialize this regulation with
   */
  explicit SignalizedIntersection(const lanelet::RegulatoryElementDataPtr& data);

protected:
  // the following lines are required so that lanelet2 can create this object when loading a map with this regulatory
  // element
  friend class RegisterRegulatoryElement<SignalizedIntersection>;
  SignalizedIntersection(Id id, const AttributeMap& attributes, const Lanelets& entry, const Lanelets& exit, const Lanelets& interior);

};

// Convenience Ptr Declarations
using SignalizedIntersectionPtr = std::shared_ptr<SignalizedIntersection>;
using SignalizedIntersectionConstPtr = std::shared_ptr<const SignalizedIntersection>;

}  // namespace lanelet