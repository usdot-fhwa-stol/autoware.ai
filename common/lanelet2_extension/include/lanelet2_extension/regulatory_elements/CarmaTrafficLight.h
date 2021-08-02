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

#ifndef LANELET2_EXTENSION_REGULATORY_ELEMENTS_CARMA_TRAFFIC_LIGHT_H
#define LANELET2_EXTENSION_REGULATORY_ELEMENTS_CARMA_TRAFFIC_LIGHT_H

#include <lanelet2_core/primitives/RegulatoryElement.h>
#include <ros/ros.h>

namespace lanelet
{
/**
 * @brief: Enum representing Traffic Light States. 
 * These states match the SAE J2735 PhaseState definitions used for SPaT messages
 * 
 * UNAVAILABLE : No data available
 * DARK : Light is non-functional
 * STOP_THEN_PROCEED : Flashing Red
 * STOP_AND_REMAIN : Solid Red
 * PRE_MOVEMENT : Yellow to Red transition (Found only in the EU)
 * PERMISSIVE_MOVEMENT_ALLOWED : Solid Green there could be conflict traffic
 * PROTECTED_MOVEMENT_ALLOWED : Solid Green no chance of conflict traffic (normally used with arrows)
 * PERMISSIVE_CLEARANCE : Yellow Solid there is a chance of conflicting traffic
 * PROTECTED_CLEARANCE : Yellow Solid no chance of conflicting traffic (normally used with arrows)
 * CAUTION_CONFLICTING_TRAFFIC : Yellow Flashing
 *
 */
enum class CarmaTrafficLightState {
  UNAVAILABLE=0,
  DARK=1,
  STOP_THEN_PROCEED=2,
  STOP_AND_REMAIN=3,
  PRE_MOVEMENT=4,
  PERMISSIVE_MOVEMENT_ALLOWED=5,
  PROTECTED_MOVEMENT_ALLOWED=6,
  PERMISSIVE_CLEARANCE=7,
  PROTECTED_CLEARANCE=8,
  CAUTION_CONFLICTING_TRAFFIC=9
};

/**
 * \brief Stream operator for CarmaTrafficLightState enum.
 */
std::ostream& operator<<(std::ostream& os, CarmaTrafficLightState s);

/**
 * @brief: Class representing a known timing traffic light.
 *         Normally the traffic light timing information is provided by SAE J2735 SPaT messages although alternative data sources can be supported
 * 
 * @ingroup RegulatoryElementPrimitives
 * @ingroup Primitives
 */

class CarmaTrafficLight : public lanelet::RegulatoryElement
{
public:
  static constexpr char RuleName[] = "carma_traffic_light";
  int revision_ = 0; //indicates when was this last modified
  ros::Duration fixed_cycle_duration;
  std::vector<std::pair<ros::Time, CarmaTrafficLightState>> recorded_time_stamps;
  /**
   * @brief setStates function sorts states automatically
   *
   * @param data The data to initialize this regulation with
   * NOTE: to extract full cycle, first and last state should match in input_time_steps
   */
  void setStates(std::vector<std::pair<ros::Time, CarmaTrafficLightState>> input_time_steps, int revision);

  /**
   * @brief getControlledLanelets function returns lanelets this element controls
   */
  lanelet::ConstLanelets getControlledLanelets() const;
  /**
   * @brief getState get the current state
   *
   * @return return current, ros::Time::now
   */
  boost::optional<CarmaTrafficLightState> getState();
  /**
   * @brief prefictState assumes sorted, fixed time, so guaranteed to give you one final state
   *
   * @param time_stamp ros::Time of the event happening
   */
  boost::optional<CarmaTrafficLightState> predictState(ros::Time time_stamp);
  ConstLineStrings3d stopLine() const;
  LineStrings3d stopLine();

  explicit CarmaTrafficLight(const lanelet::RegulatoryElementDataPtr& data);
  /**
   * @brief: Creating one is not directly usable unless setStates is called Static helper function that creates a stop line data object based on the provided inputs
   *
   * @param id The lanelet::Id of this element
   * @param lanelets List of lanelets this element controls.
   * @param stop_line The line string which represent the stop line of the traffic light
   *
   * @return RegulatoryElementData containing all the necessary information to construct a stop rule
   */
  static std::unique_ptr<lanelet::RegulatoryElementData> buildData(Id id, LineString3d stop_line, Lanelets lanelets);

private:
  // the following lines are required so that lanelet2 can create this object
  // when loading a map with this regulatory element
  friend class lanelet::RegisterRegulatoryElement<CarmaTrafficLight>;
};


// Convenience Ptr Declarations
using CarmaTrafficLightPtr = std::shared_ptr<CarmaTrafficLight>;
using CarmaTrafficLightConstPtr = std::shared_ptr<const CarmaTrafficLight>;

}  // namespace lanelet

#endif  // LANELET2_EXTENSION_REGULATORY_ELEMENTS_CARMA_TRAFFIC_LIGHT_H