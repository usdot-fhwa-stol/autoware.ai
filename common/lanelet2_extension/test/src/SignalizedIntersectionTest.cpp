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

#include <gmock/gmock.h>
#include <iostream>
#include <lanelet2_extension/regulatory_elements/SignalizedIntersection.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/utility/Units.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <lanelet2_core/Attribute.h>
#include "TestHelpers.h"
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_extension/traffic_rules/CarmaUSTrafficRules.h>

#include <lanelet2_io/Io.h>
#include <lanelet2_io/io_handlers/Factory.h>
#include <lanelet2_io/io_handlers/Writer.h>
#include <lanelet2_extension/io/autoware_osm_parser.h>

#include <lanelet2_extension/regulatory_elements/DigitalSpeedLimit.h>
#include <lanelet2_extension/projection/local_frame_projector.h>
#include <lanelet2_extension/io/autoware_osm_parser.h>
#include <lanelet2_routing/Route.h>

using ::testing::_;
using ::testing::A;
using ::testing::DoAll;
using ::testing::InSequence;
using ::testing::Return;
using ::testing::ReturnArg;
namespace lanelet
{

TEST(SignalizedIntersectionTest, mapLoadingTest)
{
  // Write new map to file
  std::string file = "resources/test_map_si.osm";
  int projector_type = 0;
  std::string target_frame;
  lanelet::ErrorMessages load_errors;
  // Parse geo reference info from the original lanelet map (.osm)
  lanelet::io_handlers::AutowareOsmParser::parseMapParams(file, &projector_type, &target_frame);

  lanelet::projection::LocalFrameProjector local_projector(target_frame.c_str());

  lanelet::LaneletMapPtr map = lanelet::load(file, local_projector, &load_errors);

  EXPECT_EQ(load_errors.size() , 0);

  EXPECT_EQ(map->laneletLayer.size(), 4);
  
  auto llt = map->laneletLayer.get(1349);

  for (auto regem : llt.regulatoryElementsAs<SignalizedIntersection>())
  {
    EXPECT_EQ(regem->getExitLanelets().size(), 1);
    EXPECT_EQ(regem->getEntryLanelets().size(), 2);
    EXPECT_EQ(regem->getInteriorLanelets().size(), 1);
    auto llts = map->laneletLayer.findUsages(regem);
    EXPECT_EQ(llts.size(), 2);   
  }

  EXPECT_EQ(map->regulatoryElementLayer.size(), 2);
}

TEST(SignalizedIntersectionTest, addRemoveLaneletTest)
{
  // Write new map to file
  std::string file = "resources/test_map_si.osm";
  int projector_type = 0;
  std::string target_frame;
  lanelet::ErrorMessages load_errors;
  // Parse geo reference info from the original lanelet map (.osm)
  lanelet::io_handlers::AutowareOsmParser::parseMapParams(file, &projector_type, &target_frame);

  lanelet::projection::LocalFrameProjector local_projector(target_frame.c_str());

  lanelet::LaneletMapPtr map = lanelet::load(file, local_projector, &load_errors);

  EXPECT_EQ(load_errors.size() , 0);

  EXPECT_EQ(map->laneletLayer.size(), 4);
  
  auto llt = map->laneletLayer.get(1349);
  auto llt3 = map->laneletLayer.get(1352);

  auto intersection = llt.regulatoryElementsAs<SignalizedIntersection>().front();
  EXPECT_EQ(intersection->getEntryLanelets().size(), 2); // check alias names of IntersectionEntry/Refers working

  EXPECT_TRUE(intersection->removeLanelet(llt3));
  EXPECT_TRUE(intersection->removeLanelet(llt));
  EXPECT_FALSE(intersection->removeLanelet(llt));

  EXPECT_EQ(llt.regulatoryElementsAs<SignalizedIntersection>().size(), 1); //NOTE: user must delete it from the lanelet 
                                                                            //itself as above will not remove it
  EXPECT_EQ(map->laneletLayer.findUsages(intersection).size(), 2);        //and also map will still have the connection

  EXPECT_EQ(intersection->getEntryLanelets().size(), 0 );
  EXPECT_EQ(intersection->getExitLanelets().size(), 1 );
  EXPECT_EQ(intersection->getInteriorLanelets().size(), 1 );

  auto llt1 = map->laneletLayer.get(1350);
  auto llt2 = map->laneletLayer.get(1351);
  EXPECT_TRUE(intersection->removeLanelet(llt1));
  EXPECT_FALSE(intersection->removeLanelet(llt1));
  EXPECT_TRUE(intersection->removeLanelet(llt2));
  EXPECT_FALSE(intersection->removeLanelet(llt2));

  EXPECT_EQ(intersection->getEntryLanelets().size(), 0 );
  EXPECT_EQ(intersection->getExitLanelets().size(), 0 );
  EXPECT_EQ(intersection->getInteriorLanelets().size(), 0 );

  intersection->addLanelet(llt, IntersectionSection::ENTRY);

  EXPECT_EQ(intersection->getEntryLanelets().size(), 1 );
  EXPECT_EQ(intersection->getExitLanelets().size(), 0 );
  EXPECT_EQ(intersection->getInteriorLanelets().size(), 0 );

  intersection->addLanelet(llt1,IntersectionSection::EXIT);
  intersection->addLanelet(llt2,IntersectionSection::INTERIOR);

  EXPECT_EQ(intersection->getEntryLanelets().size(), 1 );
  EXPECT_EQ(intersection->getExitLanelets().size(), 1 );
  EXPECT_EQ(intersection->getInteriorLanelets().size(), 1 );
}

TEST(SignalizedIntersectionTest, trafficSignalFunctions)
{
  // Write new map to file
  std::string file = "resources/test_map_si.osm";
  int projector_type = 0;
  std::string target_frame;
  lanelet::ErrorMessages load_errors;
  // Parse geo reference info from the original lanelet map (.osm)
  lanelet::io_handlers::AutowareOsmParser::parseMapParams(file, &projector_type, &target_frame);

  lanelet::projection::LocalFrameProjector local_projector(target_frame.c_str());

  lanelet::LaneletMapPtr map = lanelet::load(file, local_projector, &load_errors);

  EXPECT_EQ(load_errors.size() , 0);

  EXPECT_EQ(map->laneletLayer.size(), 4);

  auto llt = map->laneletLayer.get(1349);
  auto intersection = llt.regulatoryElementsAs<SignalizedIntersection>().front();

  lanelet::Id stop_line_id = utils::getId();
  auto pl = carma_wm::getPoint(0, 1, 0);
  auto pr = carma_wm::getPoint(1, 1, 0); 
  LineString3d virtual_stop_line(stop_line_id, {pl, pr});
  std::shared_ptr<CarmaTrafficLight> traffic_light(new CarmaTrafficLight(CarmaTrafficLight::buildData(lanelet::utils::getId(), { virtual_stop_line }, {llt})));
  llt.addRegulatoryElement(traffic_light);

  EXPECT_EQ(intersection->getTrafficSignals(llt).size(), 1);
  EXPECT_EQ(intersection->getStopLine(llt).get().id(), stop_line_id);
}

}  // namespace lanelet
