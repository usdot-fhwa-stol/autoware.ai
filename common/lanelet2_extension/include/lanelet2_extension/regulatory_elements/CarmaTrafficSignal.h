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
#include <boost/date_time/posix_time/posix_time.hpp>
#include <lanelet2_extension/time/TimeConversion.h>
#include <unordered_map>

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
enum class CarmaTrafficSignalState {
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

namespace time{

//This int is max representable number by 32bit which the posix::time library is using.
//It corresponds to 03:14:07 on Tuesday, 19 January 2038.

constexpr auto INFINITY_END_TIME_FOR_NOT_ENOUGH_STATES{2147483647};

}

struct CarmaTrafficSignalRoleNameString
{
  static constexpr char ControlStart[] = "control_start";
  static constexpr char ControlEnd[] = "control_end";
};

/**
 * \brief Stream operator for CarmaTrafficSignalState enum.
 */
std::ostream& operator<<(std::ostream& os, CarmaTrafficSignalState s);

/**
 * @brief: Class representing a known timing traffic light.
 *         Normally the traffic light timing information is provided by SAE J2735 SPaT messages although alternative data sources can be supported
 *
 * @ingroup RegulatoryElementPrimitives
 * @ingroup Primitives
 */

class CarmaTrafficSignal : public lanelet::RegulatoryElement
{
public:
  static constexpr char RuleName[] = "carma_traffic_signal";

  int revision_ = 0; //indicates when was this last modified
  boost::posix_time::time_duration fixed_cycle_duration;
  std::vector<boost::posix_time::ptime> recorded_start_time_stamps; //user must ensure it's 1 to 1 with recorded_time_stamps ,
                                                                                                        //used in dynamic SPAT processing
  std::vector<std::pair<boost::posix_time::ptime, CarmaTrafficSignalState>> recorded_time_stamps;
  std::unordered_map<CarmaTrafficSignalState, boost::posix_time::time_duration> signal_durations;


  /**
   * @brief setStates function sorts states automatically
   *
   * @param data The data to initialize this regulation with
   * NOTE: to extract full cycle, first and last state should match in input_time_steps
   */
  void setStates(std::vector<std::pair<boost::posix_time::ptime, CarmaTrafficSignalState>> input_time_steps, int revision);

  /**
   * @brief getControlStartLanelets function returns lanelets where this element's control starts
   * NOTE: Order of the lanelets does not correlate to the order of the control_end lanelets
   */
  lanelet::ConstLanelets getControlStartLanelets() const;

  /**
   * @brief getControlEndLanelets function returns lanelets where this element's control ends
   * NOTE: Order of the lanelets does not correlate to the order of the control_start lanelets
   *
   */
  lanelet::ConstLanelets getControlEndLanelets() const;

  /**
   * @brief prefictState assumes sorted, fixed time, so guaranteed to give you one final state
   *
   * @param time_stamp boost::posix_time::ptime of the event happening
   * @return std::pair of signal state and its end time at the input time
   *         optional will empty if no timestamps are recorded yet
   * @throw  InvalidInputError if timestamps recorded somehow did not have full cycle
   */
  boost::optional<std::pair<boost::posix_time::ptime, CarmaTrafficSignalState>> predictState(boost::posix_time::ptime time_stamp);

  /**
   * @brief Return the stop_lines related to the entry lanelets in order if exists.
   */
  ConstLineStrings3d stopLine() const;
  LineStrings3d stopLine();

  /**
   * @brief Return the stop_lines related to the specified entry lanelet
   * @param llt entry_lanelet
   * @return optional stop line linestring.
   *         Empty optional if no stopline, or no entry lanelets, or if specified entry lanelet is not recorded.
   */
  Optional<ConstLineString3d> getConstStopLine(const ConstLanelet& llt);
  Optional<LineString3d> getStopLine(const ConstLanelet& llt);

  explicit CarmaTrafficSignal(const lanelet::RegulatoryElementDataPtr& data);
  /**
   * @brief: Creating one is not directly usable unless setStates is called
   *         Static helper function that creates a stop line data object based on the provided inputs
   *
   * @param id The lanelet::Id of this element
   * @param entry_lanelets List of lanelets where this element's control starts
   * @param exit_lanelets List of lanelets where this element's control ends
   * @param stop_lines The line strings which represent the stop lines of the entry_lanelets (in order) controlled by the traffic light
   *
   * @return RegulatoryElementData containing all the necessary information to construct a stop rule
   */
  static std::unique_ptr<lanelet::RegulatoryElementData> buildData(Id id, LineStrings3d stop_lines, Lanelets entry_lanelets, Lanelets exit_lanelets);


private:
  // the following lines are required so that lanelet2 can create this object
  // when loading a map with this regulatory element
  friend class lanelet::RegisterRegulatoryElement<CarmaTrafficSignal>;
};


// Convenience Ptr Declarations
using CarmaTrafficSignalPtr = std::shared_ptr<CarmaTrafficSignal>;
using CarmaTrafficSignalConstPtr = std::shared_ptr<const CarmaTrafficSignal>;

}  // namespace lanelet

#endif  // LANELET2_EXTENSION_REGULATORY_ELEMENTS_CARMA_TRAFFIC_LIGHT_H