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

#include <lanelet2_io/Io.h>
#include <lanelet2_io/io_handlers/Factory.h>
#include <lanelet2_io/io_handlers/Writer.h>
#include <lanelet2_extension/io/autoware_osm_parser.h>

#include <lanelet2_extension/regulatory_elements/DigitalSpeedLimit.h>
#include <lanelet2_extension/projection/local_frame_projector.h>
#include <lanelet2_extension/io/autoware_osm_parser.h>

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

  EXPECT_EQ(map->laneletLayer.size(), 3);
  for (auto llt: map->laneletLayer)
  {
    std::cerr << "llt: " << llt.id() << std::endl;
    std::cerr << "regem size: " << llt.regulatoryElements().size() << std::endl;
    std::cerr << "----------------" << std::endl;
    for (auto regem : llt.regulatoryElementsAs<SignalizedIntersection>())
    {
      std::cerr << "regem: " << regem->id() << ", subtype: " << regem->RuleName << std::endl;
      std::cerr << "getExitLanelets size: " << regem->getExitLanelets().size() << std::endl;
      std::cerr << "a. getEntryLanelets size: " << regem->getEntryLanelets().size() << std::endl;
      std::cerr << "getInteriorLanelets size: " << regem->getInteriorLanelets().size() << std::endl;
      auto llts = map->laneletLayer.findUsages(regem);
      std::cerr << "b. getEntryLanelets size: " << llts.size() << std::endl;

      //std::cerr << "getExitLanelets: " << regem->getExitLanelets().front().id() << std::endl;
      //std::cerr << "getEntryLanelets: " << regem->getEntryLanelets().front().id() << std::endl;
      //std::cerr << "getInteriorLanelets: " << regem->getInteriorLanelets().front().id() << std::endl;
      
    }
    for (auto regem : llt.regulatoryElementsAs<DigitalSpeedLimit>())
    {
      std::cerr << "regem: " << regem->id() << ", subtype: " << regem->RuleName << std::endl;
      std::cerr << "size llt of regem speed: " << regem->getLanelets().size() << std::endl;;
      auto llts = map->laneletLayer.findUsages(regem);
      
      std::cerr << "speed findUsage size: " << llts.size() << std::endl;

    }
    std::cerr << "==================" << std::endl;
  }
  
  std::cerr << "Printing Regems: " << std::endl;
  EXPECT_EQ(map->regulatoryElementLayer.size(), 2);
  for (auto regem: map->regulatoryElementLayer)
  {
    std::cerr << "regem: " << regem->id() << std::endl;
    std::cerr << "subtype: " << regem->attribute("subtype").value() << std::endl;
  }


}


}  // namespace lanelet
