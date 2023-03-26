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

#include <gmock/gmock.h>
#include <iostream>
#include <lanelet2_extension/regulatory_elements/StopRule.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <lanelet2_core/Attribute.h>
#include "TestHelpers.h"
#include <lanelet2_extension/regulatory_elements/CarmaTrafficSignal.h>

using ::testing::_;
using ::testing::A;
using ::testing::DoAll;
using ::testing::InSequence;
using ::testing::Return;
using ::testing::ReturnArg;

namespace lanelet
{

TEST(CarmaTrafficSignalTest, CarmaTrafficSignal)
{
  auto pl1 = carma_wm::getPoint(0, 0, 0);
  auto pl2 = carma_wm::getPoint(0, 1, 0);
  auto pl3 = carma_wm::getPoint(0, 2, 0);
  auto pr1 = carma_wm::getPoint(1, 0, 0);
  auto pr2 = carma_wm::getPoint(1, 1, 0);
  auto pr3 = carma_wm::getPoint(1, 2, 0);
  std::vector<lanelet::Point3d> left_1 = { pl1, pl2, pl3 };
  std::vector<lanelet::Point3d> right_1 = { pr1, pr2, pr3 };
  std::vector<lanelet::Point3d> left_2 = { pl1, pl2, pl3 };
  std::vector<lanelet::Point3d> right_2 = { pr1, pr2, pr3 };
  auto ll_1 = carma_wm::getLanelet(left_1, right_1, lanelet::AttributeValueString::SolidDashed,lanelet::AttributeValueString::Dashed);
  auto ll_2 = carma_wm::getLanelet(left_1, right_1, lanelet::AttributeValueString::SolidDashed,lanelet::AttributeValueString::Dashed);
  
  lanelet::Id stop_line_id = utils::getId();
  LineString3d virtual_stop_line(stop_line_id, {pl2, pr2});
  lanelet::Id stop_line_id1= utils::getId();
  LineString3d virtual_stop_line1(stop_line_id1, {pl2, pr2});
  // Creat passing control line for solid dashed line
  std::shared_ptr<CarmaTrafficSignal> traffic_light(new CarmaTrafficSignal(CarmaTrafficSignal::buildData(lanelet::utils::getId(), { virtual_stop_line,  virtual_stop_line1}, {ll_1, ll_2}, {ll_2})));
  ll_1.addRegulatoryElement(traffic_light);

  auto entry_lanelets = traffic_light->getControlStartLanelets();

  lanelet::RegulatoryElementPtr regem = traffic_light;
  auto factory_pcl = lanelet::RegulatoryElementFactory::create(regem->attribute(lanelet::AttributeName::Subtype).value(),
                                                            std::const_pointer_cast<lanelet::RegulatoryElementData>(regem->constData()));

  lanelet::CarmaTrafficSignalPtr ctl = std::dynamic_pointer_cast<lanelet::CarmaTrafficSignal>(factory_pcl);
  
  ASSERT_EQ(2,entry_lanelets.size());

  auto sl = traffic_light->getStopLine(ll_2);
  ASSERT_EQ(stop_line_id1,sl.get().id());

  std::vector<std::pair<boost::posix_time::ptime, CarmaTrafficSignalState>> input_time_steps;
  
  input_time_steps.push_back(std::make_pair(time::timeFromSec(1001),static_cast<lanelet::CarmaTrafficSignalState>(0)));
  input_time_steps.push_back(std::make_pair(time::timeFromSec(1002),static_cast<lanelet::CarmaTrafficSignalState>(1)));

  /// TEST DYNAMIC SPAT
  traffic_light->recorded_time_stamps = input_time_steps;
  ASSERT_EQ(2,traffic_light->recorded_time_stamps.size());
  ASSERT_THROW(traffic_light->predictState(time::timeFromSec(1002.5)), std::invalid_argument);
  traffic_light->recorded_start_time_stamps.push_back(time::timeFromSec(1000));
  traffic_light->recorded_start_time_stamps.push_back(time::timeFromSec(1001));
  ASSERT_EQ(traffic_light->predictState(time::timeFromSec(1011.5)).get().second, static_cast<lanelet::CarmaTrafficSignalState>(3));
  /// END DYNAMIC SPAT TEST

  input_time_steps.push_back(std::make_pair(time::timeFromSec(1003),static_cast<lanelet::CarmaTrafficSignalState>(2)));
  input_time_steps.push_back(std::make_pair(time::timeFromSec(1004),static_cast<lanelet::CarmaTrafficSignalState>(3)));
  input_time_steps.push_back(std::make_pair(time::timeFromSec(1005),static_cast<lanelet::CarmaTrafficSignalState>(4)));

  EXPECT_THROW(traffic_light->setStates(input_time_steps,0), lanelet::InvalidInputError);

  input_time_steps.push_back(std::make_pair(time::timeFromSec(1006),static_cast<lanelet::CarmaTrafficSignalState>(0)));

  traffic_light->setStates(input_time_steps,0);

  ASSERT_EQ(6,traffic_light->recorded_time_stamps.size());
  ASSERT_EQ(time::durationFromSec(5),traffic_light->fixed_cycle_duration);
  ASSERT_EQ(0,traffic_light->revision_);

  ASSERT_EQ(static_cast<lanelet::CarmaTrafficSignalState>(1),traffic_light->predictState(time::timeFromSec(1.5)).get().second);
  ASSERT_EQ(static_cast<lanelet::CarmaTrafficSignalState>(0),traffic_light->predictState(time::timeFromSec(1)).get().second);
  ASSERT_EQ(traffic_light->getControlStartLanelets().size(), 2);
  ASSERT_EQ(traffic_light->getControlStartLanelets().back().id(), ll_2.id());
  
}

} // namespace lanelet
