/*
 * Copyright (C) 2019-2021 LEIDOS.
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
/*
#include <gmock/gmock.h>
#include <iostream>
#include <lanelet2_extension/regulatory_elements/StopRule.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <lanelet2_core/Attribute.h>
#include "TestHelpers.h"
#include <CarmaTrafficLight.h>

using ::testing::_;
using ::testing::A;
using ::testing::DoAll;
using ::testing::InSequence;
using ::testing::Return;
using ::testing::ReturnArg;

namespace lanelet
{

TEST(CarmaTrafficLightTest, CarmaTrafficLight)
{
  auto pl1 = carma_wm::getPoint(0, 0, 0);
  auto pl2 = carma_wm::getPoint(0, 1, 0);
  auto pl3 = carma_wm::getPoint(0, 2, 0);
  auto pr1 = carma_wm::getPoint(1, 0, 0);
  auto pr2 = carma_wm::getPoint(1, 1, 0);
  auto pr3 = carma_wm::getPoint(1, 2, 0);
  std::vector<lanelet::Point3d> left_1 = { pl1, pl2, pl3 };
  std::vector<lanelet::Point3d> right_1 = { pr1, pr2, pr3 };
  auto ll_1 = carma_wm::getLanelet(left_1, right_1, lanelet::AttributeValueString::SolidDashed,lanelet::AttributeValueString::Dashed);
  lanelet::Id traffic_light_id = utils::getId();
  LineString3d virtual_stop_line(traffic_light_id, {pl2, pr2});
  // Creat passing control line for solid dashed line
  std::shared_ptr<CarmaTrafficLight> traffic_light(new CarmaTrafficLight(CarmaTrafficLight::buildData(lanelet::utils::getId(), { virtual_stop_line }, {ll_1}, {})));
  ll_1.addRegulatoryElement(traffic_light);

  std::vector<std::pair<ros::Time, CarmaTrafficLightState>> input_time_steps;
  input_time_steps.push_back(make_pair(0,1));
  //ll_1.setStates(std::vector<std::pair<ros::Time, CarmaTrafficLightState>> input_time_steps, int revision);
  ll_1.setStates(input_time_steps, 0);

  ASSERT_EQ(1,ll_1.recorded_time_stamps);
  ASSERT_EQ(1,ll_1.fixed_cycle_duration);
  ASSERT_EQ(1,ll_1.revision_);
  ASSERT_EQ(1,ll_1.getState());
  ASSERT_EQ(1,ll_1.predictState(0));
}

} // namespace lanelet
*/