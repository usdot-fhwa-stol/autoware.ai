/*
 * Copyright (C) 2019 LEIDOS.
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
#include <lanelet2_io/Io.h>
#include <lanelet2_io/io_handlers/Factory.h>
#include <lanelet2_io/io_handlers/Writer.h>
#include <lanelet2_extension/io/autoware_osm_parser.h>

#include <lanelet2_extension/regulatory_elements/RegionAccessRule.h>
#include <lanelet2_extension/regulatory_elements/DigitalSpeedLimit.h>
#include <lanelet2_extension/regulatory_elements/DirectionOfTravel.h>
#include <lanelet2_extension/regulatory_elements/PassingControlLine.h>
#include <lanelet2_extension/regulatory_elements/StopRule.h>

#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <lanelet2_core/Attribute.h>
#include <lanelet2_core/utility/Units.h>
#include "TestHelpers.h"

using ::testing::_;
using ::testing::A;
using ::testing::DoAll;
using ::testing::InSequence;
using ::testing::Return;
using ::testing::ReturnArg;

namespace lanelet
{
using namespace lanelet::units::literals;
TEST(MapLoadingTest, parseMapParams)
{
  int projector_type = 1; // default value
  std::string target_frame, lanelet2_filename;
  std::string correct_georeference = "+proj=tmerc +lat_0=39.46636844371259 +lon_0=-76.16919523566943 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m +no_defs ";
  // Test regular way
  lanelet2_filename = "resources/test_map.osm";
  lanelet::io_handlers::AutowareOsmParser::parseMapParams(lanelet2_filename, &projector_type, &target_frame);
  EXPECT_EQ(correct_georeference, target_frame);
  // Test v="string" way
  target_frame = "";
  lanelet2_filename = "resources/test_map_v.osm";
  lanelet::io_handlers::AutowareOsmParser::parseMapParams(lanelet2_filename, &projector_type, &target_frame);
  EXPECT_EQ(correct_georeference, target_frame);
  // Test v="string" way
  target_frame = "";
  lanelet2_filename = "resources/test_map_value.osm";
  lanelet::io_handlers::AutowareOsmParser::parseMapParams(lanelet2_filename, &projector_type, &target_frame);
  EXPECT_EQ(correct_georeference, target_frame);
  // Test unsupported way
  target_frame = "";
  lanelet2_filename = "resources/test_map_nogeoreference.osm";
  EXPECT_THROW(lanelet::io_handlers::AutowareOsmParser::parseMapParams(lanelet2_filename, &projector_type, &target_frame), lanelet::ParseError);
}



TEST(MapLoadingTest, mapLoadingTest)
{
  lanelet::LaneletMapPtr map = lanelet::load("resources/test_map.osm", lanelet::Origin({ 0, 0 }));

  auto ll_1 = map->laneletLayer.find(1349);

  auto sl = (*ll_1).regulatoryElementsAs<DigitalSpeedLimit>();

  ASSERT_EQ(1, sl.size());
  ASSERT_EQ(5_mph, sl[0]->getSpeedLimit());

  auto control_lines = (*ll_1).regulatoryElementsAs<PassingControlLine>();

  ASSERT_EQ(2, control_lines.size());
  // TODO control line check
  ASSERT_TRUE(
      PassingControlLine::boundPassable(ll_1->leftBound(), control_lines, true, lanelet::Participants::VehicleCar));
  ASSERT_TRUE(
      PassingControlLine::boundPassable(ll_1->leftBound(), control_lines, false, lanelet::Participants::VehicleCar));

  ASSERT_FALSE(
      PassingControlLine::boundPassable(ll_1->rightBound(), control_lines, true, lanelet::Participants::VehicleCar));
  ASSERT_FALSE(
      PassingControlLine::boundPassable(ll_1->rightBound(), control_lines, false, lanelet::Participants::VehicleCar));

  auto dot = (*ll_1).regulatoryElementsAs<DirectionOfTravel>();

  ASSERT_EQ(1, dot.size());
  ASSERT_TRUE(dot[0]->isOneWay());

  auto rar = (*ll_1).regulatoryElementsAs<RegionAccessRule>();

  ASSERT_EQ(1, dot.size());
  ASSERT_TRUE(rar[0]->accessable("vehicle:car"));
  ASSERT_FALSE(rar[0]->accessable("vehicle"));

  auto stop_lines = (*ll_1).regulatoryElementsAs<StopRule>();
  ASSERT_EQ(1, stop_lines.size());

  ASSERT_FALSE(
      StopRule::appliesTo(map->lineStringLayer.get(1350), stop_lines, lanelet::Participants::VehicleCar)); //"no": (not passable by default) is implied in the map
}

}  // namespace lanelet
