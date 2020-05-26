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
#include <lanelet2_extension/utility/utilities.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <lanelet2_extension/regulatory_elements/PassingControlLine.h>
#include <map>
#include <ros/ros.h>

#include <lanelet2_extension/projection/local_frame_projector.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/io_handlers/Factory.h>
#include <lanelet2_io/io_handlers/Writer.h>
#include <lanelet2_extension/io/autoware_osm_parser.h>
#include <lanelet2_validation/Validation.h>

using lanelet::Lanelet;
using lanelet::LineString3d;
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

class TestSuite1 : public ::testing::Test
{
public:
  TestSuite1() : sample_map_ptr(new lanelet::LaneletMap())
  {  // NOLINT
    // create sample lanelets
    Point3d p1, p2, p3, p4, p5, p6, p7, p8, p9, p10;

    p1 = Point3d(getId(), 0., 0., 0.);
    p2 = Point3d(getId(), 0., 1., 0.);
    p3 = Point3d(getId(), 1., 0., 0.);
    p4 = Point3d(getId(), 1., 1., 0.);

    LineString3d ls_left(getId(), { p1, p2 });   // NOLINT
    LineString3d ls_right(getId(), { p3, p4 });  // NOLINT

    p5 = Point3d(getId(), 0., 2., 0.);
    p6 = Point3d(getId(), 1., 2., 0.);

    LineString3d ls_left2(getId(), { p2, p5 });   // NOLINT
    LineString3d ls_right2(getId(), { p4, p6 });  // NOLINT

    p7 = Point3d(getId(), 0., 3., 0.);
    p8 = Point3d(getId(), 1., 3., 0.);

    LineString3d ls_left3(getId(), { p5, p7 });   // NOLINT
    LineString3d ls_right3(getId(), { p6, p8 });  // NOLINT

    p9 = Point3d(getId(), 0., 1., 0.);
    p10 = Point3d(getId(), 1., 1., 0.);

    LineString3d ls_left4(getId(), { p9, p5 });   // NOLINT
    LineString3d ls_right4(getId(), { p10, p6 });  // NOLINT

    road_lanelet = Lanelet(getId(), ls_left, ls_right);
    road_lanelet.attributes()[lanelet::AttributeName::Subtype] = lanelet::AttributeValueString::Road;

    next_lanelet = Lanelet(getId(), ls_left2, ls_right2);
    next_lanelet.attributes()[lanelet::AttributeName::Subtype] = lanelet::AttributeValueString::Road;

    next_lanelet2 = Lanelet(getId(), ls_left3, ls_right3);
    next_lanelet2.attributes()[lanelet::AttributeName::Subtype] = lanelet::AttributeValueString::Road;

    merging_lanelet = Lanelet(getId(), ls_left4, ls_right4);
    merging_lanelet.attributes()[lanelet::AttributeName::Subtype] = lanelet::AttributeValueString::Road;

    sample_map_ptr->add(road_lanelet);
    sample_map_ptr->add(next_lanelet);
    sample_map_ptr->add(next_lanelet2);
    sample_map_ptr->add(merging_lanelet);
  }
  ~TestSuite1()
  {
  }

  lanelet::LaneletMapPtr sample_map_ptr;
  Lanelet road_lanelet;
  Lanelet next_lanelet;
  Lanelet next_lanelet2;
  Lanelet merging_lanelet;

private:
};

TEST_F(TestSuite1, MatchWaypointAndLanelet)
{
  std::map<int, lanelet::Id> waypointid2laneletid;
  autoware_msgs::LaneArray lane_array;
  autoware_msgs::Lane lane;
  autoware_msgs::Waypoint waypoint;

  // waypoints that overlap with road_lanelet
  for (int i = 1; i < 4; i++)
  {
    waypoint.gid = i;
    waypoint.pose.pose.position.x = 0.5;
    waypoint.pose.pose.position.y = i - 0.5;
    lane.waypoints.push_back(waypoint);
  }

  // waypoint that overlaps with no lanelet
  waypoint.gid = 4;
  waypoint.pose.pose.position.x = 1.5;
  waypoint.pose.pose.position.y = 1.5;
  lane.waypoints.push_back(waypoint);

  lane_array.lanes.push_back(lane);

  lanelet::traffic_rules::TrafficRulesPtr traffic_rules =
      lanelet::traffic_rules::TrafficRulesFactory::create(lanelet::Locations::Germany, lanelet::Participants::Vehicle);
  lanelet::routing::RoutingGraphPtr routing_graph =
      lanelet::routing::RoutingGraph::build(*sample_map_ptr, *traffic_rules);

  lanelet::utils::matchWaypointAndLanelet(sample_map_ptr, routing_graph, lane_array, &waypointid2laneletid);

  ASSERT_EQ(3, waypointid2laneletid.size()) << "failed to match waypoints with lanelets";
  ASSERT_EQ(road_lanelet.id(), waypointid2laneletid.at(1)) << "failed to match waypoints with lanelet";
  ASSERT_EQ(next_lanelet.id(), waypointid2laneletid.at(2)) << "failed to match waypoints with lanelet";
  ASSERT_EQ(next_lanelet2.id(), waypointid2laneletid.at(3)) << "failed to match waypoints with lanelet";
}

TEST_F(TestSuite1, OverwriteLaneletsCenterline)
{
  lanelet::utils::overwriteLaneletsCenterline(sample_map_ptr);

  for (const auto& lanelet : sample_map_ptr->laneletLayer)
  {
    ASSERT_TRUE(lanelet.hasCustomCenterline()) << "failed to calculate fine centerline";
  }
}

TEST_F(TestSuite, RemoveRegulatoryElements)
{
  // call remove function for REG elem
  std::vector<lanelet::RegulatoryElementPtr> regem_list = {pcl_unreg};
  lanelet::utils::removeRegulatoryElements(regem_list, sample_map_ptr);

  // check if reg was removed
  lanelet::utils::query::References rf = lanelet::utils::query::findReferences(pcl_unreg, sample_map_ptr);
  ASSERT_EQ(rf.regems.size(), 0); //pcl_unreg is gone
  ASSERT_EQ(rf.lss.size(), 1); //not part of pcl_unreg anymore so stand alone
  ASSERT_EQ(rf.lss.begin()->id(), pcl_unreg_ls.id());

  // find regems that are using pcl_unreg_ls
  std::vector<lanelet::RegulatoryElementPtr> regems1 = sample_map_ptr->regulatoryElementLayer.findUsages(pcl_unreg_ls);
  ASSERT_EQ(regems1.size(), 0); //there should be none as it was removed

  // check if other primitives are referencing the removed regem
  // road_lanelet had 2 regems originally: pcl, tf
  ASSERT_EQ(sample_map_ptr->laneletLayer.find(road_lanelet.id())->regulatoryElements().size(),2);
  lanelet::utils::removeRegulatoryElements({tl}, sample_map_ptr);

  // now it should only have 1 regem: pcl
  ASSERT_EQ(sample_map_ptr->regulatoryElementLayer.find(tl->id()),sample_map_ptr->regulatoryElementLayer.end());
  ASSERT_EQ(sample_map_ptr->regulatoryElementLayer.findUsages(stop_line).size(), 0); //dangling parameter not referencing back
  ASSERT_EQ(sample_map_ptr->laneletLayer.find(road_lanelet.id())->regulatoryElements().size(),1);
  ASSERT_EQ(sample_map_ptr->laneletLayer.find(road_lanelet.id())->regulatoryElements()[0]->id(),pcl->id());
  rf = lanelet::utils::query::findReferences(tl, sample_map_ptr);
  ASSERT_EQ(rf.regems.size(), 0); 
  ASSERT_EQ(rf.lss.size(), 3); //tl local copy still references these 3 parameters, 
                              // but those parameters don't reference it back as it should 
  ASSERT_EQ(rf.llts.size(), 0); // connection with parent llt is severed

  // Test if map is valid by writing it to a file and loading it
  // Build new map from modified data
  std::vector<lanelet::Area> new_areas;
  std::vector<lanelet::Lanelet> new_lanelets;
  for (auto lanelet : sample_map_ptr->laneletLayer)
  {
    std::cerr << "Adding lanelet: " << lanelet.id() << std::endl;
    lanelet::Lanelet mutable_ll = sample_map_ptr->laneletLayer.get(lanelet.id());
    new_lanelets.emplace_back(mutable_ll);
  }
  for (auto area : sample_map_ptr->areaLayer)
  {
    std::cerr << "Adding area: " << area.id() << std::endl;
    lanelet::Area mutable_area = sample_map_ptr->areaLayer.get(area.id());
    new_areas.emplace_back(mutable_area);
  }
  auto new_map = lanelet::utils::createMap(new_lanelets, new_areas);
  // Write new map to file
  std::string new_file = "modified_map.osm";
  lanelet::projection::LocalFrameProjector local_projector("+proj=tmerc +lat_0=39.46636844371259 +lon_0=-76.16919523566943 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m +no_defs");
  lanelet::ErrorMessages write_errors;

  lanelet::write(new_file, *new_map, local_projector, &write_errors);

  if (write_errors.size() > 0)
  {
    std::cerr << "Errors occurred while writing the map! Output file located at " << new_file << std::endl;
  }
  else
  {
    std::cerr << "Map written without errors to: " << new_file << std::endl;
  }
  for (auto msg : write_errors)
  {
    std::cerr << "Write Error: " << msg << std::endl;
  }

  // Now try to load it back
  using namespace lanelet;
  LaneletMapPtr map = lanelet::load(new_file, local_projector);
  lanelet::validation::ValidationConfig config;
  auto issues = lanelet::validation::validateMap(*map, config);
  auto report = lanelet::validation::buildReport(issues);
  EXPECT_EQ(13, report.warnings.size()); // these 13 warnings have been verified  
                                         // that they are not related to our issue of focus
  EXPECT_EQ(0ul, report.errors.size());                                                          
  EXPECT_EQ(map->regulatoryElementLayer.size(), 1);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "TestNode");
  return RUN_ALL_TESTS();
}
