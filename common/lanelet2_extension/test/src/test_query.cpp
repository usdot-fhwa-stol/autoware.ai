/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <gtest/gtest.h>
#include <lanelet2_extension/utility/query.h>
#include <lanelet2_extension/regulatory_elements/PassingControlLine.h>
#include <math.h>
#include <ros/ros.h>

using lanelet::Lanelet;
using lanelet::LineString3d;
using lanelet::LineStringOrPolygon3d;
using lanelet::Point3d;
using lanelet::Points3d;
using lanelet::Area;
using lanelet::utils::getId;

class TestSuite : public ::testing::Test
{
public:
  Point3d p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13, p14, p15, p16, p17, p_unreg1, p_unreg2;
  LineString3d ls_left, ls_right, ls_right_right, traffic_light_base, 
    traffic_light_bulbs, stop_line, ls_area_right, ls_area_top, ls_area_bottom, pcl_unreg_ls, ls_unreg;
  lanelet::ConstLineString3d centerline;
  Lanelet road_lanelet, road_lanelet1, crosswalk_lanelet;
  Area side_area;
  lanelet::autoware::AutowareTrafficLight::Ptr tl;
  std::shared_ptr<lanelet::PassingControlLine> pcl, pcl_unreg;

  TestSuite() : sample_map_ptr(new lanelet::LaneletMap())
  {  // NOLINT

    // create sample lanelets
    p1 = Point3d(getId(), 0., 0., 0.);
    p2 = Point3d(getId(), 0., 1., 0.);
    p_unreg1 = Point3d(getId(), 0., 11., 0.);
    p_unreg2 = Point3d(getId(), 0., 12., 0.);

    p3 = Point3d(getId(), 1., 0., 0.);
    p4 = Point3d(getId(), 1., 1., 0.);

    p5 = Point3d(getId(), 2., 0., 0.);
    p6 = Point3d(getId(), 2., 1., 0.);

    // create a sample area
    p14 = Point3d(getId(), 3., 0., 0.);
    p15 = Point3d(getId(), 3., 1., 0.);

    ls_left  = LineString3d(getId(), { p1, p2 });   // NOLINT
    ls_right = LineString3d(getId(), { p3, p4 });  // NOLINT
    ls_right_right = LineString3d(getId(), { p5, p6 });  // NOLINT
    ls_area_right = LineString3d(getId(), { p14, p15 });  // NOLINT
    ls_area_top = LineString3d(getId(), { p15, p6 });
    ls_area_bottom = LineString3d(getId(), { p14, p5 });
    ls_unreg = LineString3d(getId(), { p_unreg1, p_unreg2 });
    
    road_lanelet = Lanelet(getId(), ls_left, ls_right);
    road_lanelet.attributes()[lanelet::AttributeName::Subtype] = lanelet::AttributeValueString::Road;
    centerline = road_lanelet.centerline();
    road_lanelet1 = Lanelet(getId(), ls_right, ls_right_right);
    road_lanelet1.attributes()[lanelet::AttributeName::Subtype] = lanelet::AttributeValueString::Road;
    crosswalk_lanelet = Lanelet(getId(), ls_left, ls_right);
    crosswalk_lanelet.attributes()[lanelet::AttributeName::Subtype] = lanelet::AttributeValueString::Crosswalk;
    side_area = Area(getId(), {ls_area_top, ls_area_right, ls_area_bottom, ls_right_right});

    // create sample traffic light
    p7 = Point3d(getId(), 0., 1., 4.);
    p8 = Point3d(getId(), 1., 1., 4.);

    p9 = Point3d(getId(), 0., 1., 4.5);
    p10 = Point3d(getId(), 0.5, 1., 4.5);
    p11 = Point3d(getId(), 1., 1., 4.5);

    p12 = Point3d(getId(), 0., 0., 0.);
    p13 = Point3d(getId(), 1., 0., 0.);

    traffic_light_base = LineString3d(getId(), Points3d{ p7, p8 });        // NOLINT
    traffic_light_bulbs = LineString3d(getId(), Points3d{ p9, p10, p11 });  // NOLINT
    stop_line = LineString3d(getId(), Points3d{ p12, p13 });               // NOLINT
    pcl_unreg_ls = LineString3d(getId(), { p16, p17 });

    tl = lanelet::autoware::AutowareTrafficLight::make(getId(), lanelet::AttributeMap(), { traffic_light_base },
                                                            stop_line, { traffic_light_bulbs });  // NOLINT

    pcl = std::make_shared<lanelet::PassingControlLine>(lanelet::PassingControlLine::buildData(
      lanelet::utils::getId(), {road_lanelet1.leftBound() }, {}, { lanelet::Participants::Vehicle }));

    pcl_unreg = std::make_shared<lanelet::PassingControlLine>(lanelet::PassingControlLine::buildData(
      lanelet::utils::getId(), {pcl_unreg_ls}, {}, { lanelet::Participants::Vehicle }));

    road_lanelet.addRegulatoryElement(tl);
    road_lanelet.addRegulatoryElement(pcl);
    side_area.addRegulatoryElement(pcl);
    // add items to map
    sample_map_ptr->add(road_lanelet);
    sample_map_ptr->add(road_lanelet1);
    sample_map_ptr->add(crosswalk_lanelet);
    sample_map_ptr->add(side_area);
    sample_map_ptr->add(pcl_unreg);
  }
  ~TestSuite(){}

  lanelet::LaneletMapPtr sample_map_ptr;

private:
};

TEST_F(TestSuite, QueryReferences)
{
  // Test references to a point
  lanelet::utils::query::References rf = lanelet::utils::query::findReferences(p1, sample_map_ptr);
  ASSERT_EQ(rf.lss.size(), 0);
  ASSERT_EQ(rf.llts.size(), 2);
  rf = lanelet::utils::query::findReferences(p3, sample_map_ptr);
  ASSERT_EQ(rf.lss.size(), 0);
  ASSERT_EQ(rf.llts.size(), 3);
  Point3d ptest = Point3d(getId(), 2.0, 2.0, 3.0);
  rf = lanelet::utils::query::findReferences(ptest, sample_map_ptr);
  ASSERT_EQ(rf.lss.size(), 0);
  ASSERT_EQ(rf.llts.size(), 0);

  // Test references to a linestring
  rf = lanelet::utils::query::findReferences(ls_left, sample_map_ptr);
  ASSERT_EQ(rf.lss.size(), 0);
  ASSERT_EQ(rf.llts.size(), 2);
  rf = lanelet::utils::query::findReferences(ls_right, sample_map_ptr);
  ASSERT_EQ(rf.lss.size(), 0);
  ASSERT_EQ(rf.llts.size(), 3);
  // linestrings that don't exist
  Point3d pne1 = Point3d(getId(), 5., 7., 0.);
  Point3d pne2 = Point3d(getId(), 5., 8., 0.);
  LineString3d ls_nonexistent  = LineString3d(getId(), { pne1, pne2 });
  rf = lanelet::utils::query::findReferences(ls_nonexistent, sample_map_ptr);
  ASSERT_EQ(rf.lss.size(), 0);
  ASSERT_EQ(rf.llts.size(), 0);
  ASSERT_EQ(rf.regems.size(), 0);
  // linestring that exist individually (not part of any other primitive)
  sample_map_ptr->add(ls_unreg);
  rf = lanelet::utils::query::findReferences(ls_unreg, sample_map_ptr);
  ASSERT_EQ(rf.lss.size(), 1);
  ASSERT_EQ(rf.llts.size(), 0);
  ASSERT_EQ(rf.regems.size(), 0);
  // linestrings that half exist, sharing a point
  LineString3d ls_halfexist  = LineString3d(getId(), { p1, pne2 });
  rf = lanelet::utils::query::findReferences(ls_halfexist, sample_map_ptr);
  ASSERT_EQ(rf.lss.size(), 0);
  ASSERT_EQ(rf.llts.size(), 2);
  ls_halfexist  = LineString3d(getId(), { p3, pne2 });
  rf = lanelet::utils::query::findReferences(ls_halfexist, sample_map_ptr);
  ASSERT_EQ(rf.lss.size(), 0);
  ASSERT_EQ(rf.llts.size(), 3);

  // Test references to regulatory elements
  rf = lanelet::utils::query::findReferences(tl, sample_map_ptr);
  ASSERT_EQ(rf.regems.size(), 0);
  ASSERT_EQ(rf.lss.size(), 0); // tl has 3 unregistered lss, but it is part of llt itself which is registered
  ASSERT_EQ(rf.llts.size(), 1);
  ASSERT_EQ(rf.areas.size(), 0);
  rf = lanelet::utils::query::findReferences(pcl, sample_map_ptr);
  ASSERT_EQ(rf.regems.size(), 0);
  ASSERT_EQ(rf.lss.size(), 0);
  ASSERT_EQ(rf.llts.size(), 3); //road_lanelet(directly added), road_lanelet1(leftBound is pcl), crosswalk_lanelet(rightBound is pcl)
  ASSERT_EQ(rf.areas.size(), 1);
  // regem that exists by itself, not referenced by any other primitive
  rf = lanelet::utils::query::findReferences(pcl_unreg, sample_map_ptr);
  ASSERT_EQ(rf.regems.size(), 1); //itself
  ASSERT_EQ(rf.lss.size(), 0); //not part of any other primitives
  ASSERT_EQ(rf.llts.size(), 0); 
  ASSERT_EQ(rf.areas.size(), 0);

  // Test references to Area
  rf = lanelet::utils::query::findReferences(side_area, sample_map_ptr);
  ASSERT_EQ(rf.regems.size(), 0);
  ASSERT_EQ(rf.lss.size(), 0);
  ASSERT_EQ(rf.llts.size(), 3); //it has pcl, which is referenced by 3 llts
  ASSERT_EQ(rf.areas.size(), 1); 
  ASSERT_EQ(rf.areas.begin()->id(), side_area.id()); //referencing itself
  // area that half exist (shares borders, but not in the map)
  LineString3d ls_nonexistent_top  = LineString3d(getId(), { p15, pne2 });
  LineString3d ls_nonexistent_bottom  = LineString3d(getId(), { p14, pne1 });
  Area area_halfexist = Area(getId(), {ls_nonexistent_top, ls_nonexistent, ls_nonexistent_bottom, ls_area_right});
  rf = lanelet::utils::query::findReferences(area_halfexist, sample_map_ptr);
  ASSERT_EQ(rf.regems.size(), 0);
  ASSERT_EQ(rf.lss.size(), 0);
  ASSERT_EQ(rf.llts.size(), 0);
  ASSERT_EQ(rf.areas.size(), 1); // sharing border with only this area el

  // Test references to Lanelet
  rf = lanelet::utils::query::findReferences(road_lanelet, sample_map_ptr);
  ASSERT_EQ(rf.regems.size(), 0); //tl and pcl both are accounted for inside road_lanelet
  ASSERT_EQ(rf.lss.size(), 0); //it has tl, which has stop_line,traffic_light_base,traffic_light_bulbs linestrings
                                // which are not in the map, although tl itself is in the lanelet
  ASSERT_EQ(rf.llts.size(), 3); //itself + llt that overlays but different type + and llt that shares border, 
  ASSERT_EQ(rf.areas.size(), 1); // due to having a same regem pcl
}

TEST_F(TestSuite, QueryLanelets)
{
  lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(sample_map_ptr);
  ASSERT_EQ(3, all_lanelets.size()) << "failed to retrieve all lanelets";

  lanelet::ConstLanelets subtype_lanelets =
      lanelet::utils::query::subtypeLanelets(all_lanelets, lanelet::AttributeValueString::Road);
  ASSERT_EQ(2, subtype_lanelets.size()) << "failed to retrieve road lanelet by subtypeLanelets";

  lanelet::ConstLanelets road_lanelets = lanelet::utils::query::roadLanelets(all_lanelets);
  ASSERT_EQ(2, road_lanelets.size()) << "failed to retrieve road lanelets";

  lanelet::ConstLanelets crosswalk_lanelets = lanelet::utils::query::crosswalkLanelets(all_lanelets);
  ASSERT_EQ(1, crosswalk_lanelets.size()) << "failed to retrieve crosswalk lanelets";
}

TEST_F(TestSuite, QueryTrafficLights)
{
  lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(sample_map_ptr);

  auto traffic_lights = lanelet::utils::query::trafficLights(all_lanelets);
  ASSERT_EQ(1, traffic_lights.size()) << "failed to retrieve traffic lights";

  auto autoware_traffic_lights = lanelet::utils::query::autowareTrafficLights(all_lanelets);
  ASSERT_EQ(1, autoware_traffic_lights.size()) << "failed to retrieve autoware traffic lights";
}

TEST_F(TestSuite, QueryStopLine)
{
  lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(sample_map_ptr);
  lanelet::ConstLanelets road_lanelets = lanelet::utils::query::roadLanelets(all_lanelets);

  auto stop_lines = lanelet::utils::query::stopLinesLanelets(all_lanelets);
  ASSERT_EQ(1, stop_lines.size()) << "failed to retrieve stop lines from all lanelets";

  auto stop_lines2 = lanelet::utils::query::stopLinesLanelet(road_lanelets.back());
  ASSERT_EQ(1, stop_lines2.size()) << "failed to retrieve stop lines from a lanelet";
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "TestNode");
  return RUN_ALL_TESTS();
}
