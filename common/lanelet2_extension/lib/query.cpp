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
 *
 * Authors: Simon Thompson, Ryohsuke Mitsudome
 */

#include <ros/ros.h>

#include <Eigen/Eigen>

#include <lanelet2_extension/utility/message_conversion.h>
#include <lanelet2_extension/utility/query.h>

#include <set>
#include <string>
#include <vector>

namespace lanelet
{
namespace utils
{
//====================================================================================================> DEVELOP START

// Comparison function

struct comparator
{
  template <typename PrimT>
  bool operator()(const PrimT& t1, const PrimT& t2) const {
    // TODO:
    // 1. assume every lanelet object has different id
    // 2. assume every lanelet object has unique id within its layer only
    return true;
  }
};
// Depending on the type of input, different recurse functions will be called
// primT: Point, LS, llt, regem, polygon etc
template <typename T, typename primT>
std::vector<lanelet::Primitive<T>> query::findReferences (const primT prim, const lanelet::LaneletMapPtr ll_Map)
{
  std::vector<lanelet::Primitive<T>> return_list;
  // use set to record only once
  // some_comparison_func should be implemented to detect type and compare ids appropriately
  std::unordered_set<lanelet::Primitive<T>, std::hash<lanelet::Primitive<T>>, comparator> recorded_set;
  // call relevant recurse function based on prim's typedef 
  // and pass recorded_set by reference
  recurse(prim, recorded_set, ll_Map, query::CHECK_CHILD);
  // somehow turn the set into vector here
  // recorded_set-> return_list
  return return_list;
} 

// following recurse functions are helper functions for each primitives
// not sure if its possible to reduce it using template, but the skeleton code is as follows

// Point
template <typename T>
void recurse (lanelet::Point3d prim, std::unordered_set<lanelet::Primitive<T>>& recorded_set, const lanelet::LaneletMapPtr ll_Map, query::direction check_dir)
{
  // no primitive to go down

  // so always go back up : query::CHECK_PARENT
  // get LSs that owns this point
  auto ls_list_owning_point = ll_Map->lineStringLayer.findUsages(prim);
  for (auto ls : ls_list_owning_point)
  {
    // get the owners of this ls
    // checking size makes sure we don't add this primitive again if its owner is added already when recursion finishes
    // although most likely some other primitive will own this point
    //int size_before = recorded_set.size();
    recurse(ls, recorded_set, query::CHECK_PARENT);
    //if (size_before == recorded_set.size())
    //  recorded_set.insert(ls);
  }
  // return
}
// LS
template <typename T>
void recurse (lanelet::LineString3d prim, std::unordered_set<lanelet::Primitive<T>>& recorded_set, const lanelet::LaneletMapPtr ll_Map, query::direction check_dir)
{
  // go down?
  if (check_dir == query::CHECK_CHILD)
  {
    // loop through its child and call recurse on it
    for (auto p: prim.points())
    {
      recurse(p, recorded_set, check_dir);
    }
    // go back up once finished
    return;
  }
  
  // go up := check_Dir == query::CHECK_PARENT
  // process lanelets owning this ls
  auto llt_list_owning_ls = ll_Map->laneletLayer.findUsages(prim);
  for (auto llt : llt_list_owning_ls)
  {
    // recurse on this lanelet
    int size_before = recorded_set.size();
    recurse(llt, recorded_set, query::CHECK_PARENT);
  }
  // process areas owning this ls
  // similar to above

  // process regems owning this ls
  // similar to above

}

// Lanelet
template <typename T>
void recurse (lanelet::Lanelet prim, std::unordered_set<lanelet::Primitive<T>>& recorded_set, const lanelet::LaneletMapPtr ll_Map, query::direction check_dir)
{
  // go down := query::CHECK_CHILD
  if (check_dir == query::CHECK_CHILD)
  {
    // loop through its child and call recurse on it
    recurse(prim.leftBound(), recorded_set, check_dir);
    recurse(prim.rightBound(), recorded_set, check_dir);
    recurse(prim.centerline(), recorded_set, check_dir);
    recurse(prim.regulatoryElements(), recorded_set, check_dir);
    // go back up once finished
    return;
  }

  // go up := query::CHECK_PARENT
  // no one 'owns' lanelet, so just add it
  recorded_set.insert(prim);
  return;
}

// Area
template <typename T>
void recurse (lanelet::Area prim, std::unordered_set<lanelet::Primitive<T>>& recorded_set, const lanelet::LaneletMapPtr ll_Map, query::direction check_dir)
{

  // basically same as Lanelet

} 

// RegulatoryElement
template <typename T>
void recurse (lanelet::RegulatoryElement prim, std::unordered_set<lanelet::Primitive<T>>& recorded_set, const lanelet::LaneletMapPtr ll_Map, query::direction check_dir)
{
  // Basically same as linestring, except it doesn't go down to point

}
//==================================================================================================> END

// returns all lanelets in laneletLayer - don't know how to convert
// PrimitveLayer<Lanelets> -> std::vector<Lanelets>
lanelet::ConstLanelets query::laneletLayer(const lanelet::LaneletMapPtr ll_map)
{
  lanelet::ConstLanelets lanelets;
  if (!ll_map)
  {
    ROS_WARN("No map received!");
    return lanelets;
  }

  for (auto li = ll_map->laneletLayer.begin(); li != ll_map->laneletLayer.end(); li++)
  {
    lanelets.push_back(*li);
  }

  return lanelets;
}

lanelet::ConstLanelets query::subtypeLanelets(const lanelet::ConstLanelets lls, const char subtype[])
{
  lanelet::ConstLanelets subtype_lanelets;

  for (auto li = lls.begin(); li != lls.end(); li++)
  {
    lanelet::ConstLanelet ll = *li;

    if (ll.hasAttribute(lanelet::AttributeName::Subtype))
    {
      lanelet::Attribute attr = ll.attribute(lanelet::AttributeName::Subtype);
      if (attr.value() == subtype)
      {
        subtype_lanelets.push_back(ll);
      }
    }
  }

  return subtype_lanelets;
}

lanelet::ConstLanelets query::crosswalkLanelets(const lanelet::ConstLanelets lls)
{
  return (query::subtypeLanelets(lls, lanelet::AttributeValueString::Crosswalk));
}

lanelet::ConstLanelets query::roadLanelets(const lanelet::ConstLanelets lls)
{
  return (query::subtypeLanelets(lls, lanelet::AttributeValueString::Road));
}

std::vector<lanelet::TrafficLightConstPtr> query::trafficLights(const lanelet::ConstLanelets lanelets)
{
  std::vector<lanelet::TrafficLightConstPtr> tl_reg_elems;

  for (auto i = lanelets.begin(); i != lanelets.end(); i++)
  {
    lanelet::ConstLanelet ll = *i;
    std::vector<lanelet::TrafficLightConstPtr> ll_tl_re = ll.regulatoryElementsAs<lanelet::TrafficLight>();

    // insert unique tl into array
    for (auto tli = ll_tl_re.begin(); tli != ll_tl_re.end(); tli++)
    {
      lanelet::TrafficLightConstPtr tl_ptr = *tli;
      lanelet::Id id = tl_ptr->id();
      bool unique_id = true;
      for (auto ii = tl_reg_elems.begin(); ii != tl_reg_elems.end(); ii++)
      {
        if (id == (*ii)->id())
        {
          unique_id = false;
          break;
        }
      }
      if (unique_id)
      {
        tl_reg_elems.push_back(tl_ptr);
      }
    }
  }
  return tl_reg_elems;
}

std::vector<lanelet::AutowareTrafficLightConstPtr> query::autowareTrafficLights(const lanelet::ConstLanelets lanelets)
{
  std::vector<lanelet::AutowareTrafficLightConstPtr> tl_reg_elems;

  for (auto i = lanelets.begin(); i != lanelets.end(); i++)
  {
    lanelet::ConstLanelet ll = *i;
    std::vector<lanelet::AutowareTrafficLightConstPtr> ll_tl_re =
        ll.regulatoryElementsAs<lanelet::autoware::AutowareTrafficLight>();

    // insert unique tl into array
    for (auto tli = ll_tl_re.begin(); tli != ll_tl_re.end(); tli++)
    {
      lanelet::AutowareTrafficLightConstPtr tl_ptr = *tli;
      lanelet::Id id = tl_ptr->id();
      bool unique_id = true;

      for (auto ii = tl_reg_elems.begin(); ii != tl_reg_elems.end(); ii++)
      {
        if (id == (*ii)->id())
        {
          unique_id = false;
          break;
        }
      }

      if (unique_id)
        tl_reg_elems.push_back(tl_ptr);
    }
  }
  return tl_reg_elems;
}

// return all stop lines and ref lines from a given set of lanelets
std::vector<lanelet::ConstLineString3d> query::stopLinesLanelets(const lanelet::ConstLanelets lanelets)
{
  std::vector<lanelet::ConstLineString3d> stoplines;

  for (auto lli = lanelets.begin(); lli != lanelets.end(); lli++)
  {
    std::vector<lanelet::ConstLineString3d> ll_stoplines;
    ll_stoplines = query::stopLinesLanelet(*lli);
    stoplines.insert(stoplines.end(), ll_stoplines.begin(), ll_stoplines.end());
  }

  return stoplines;
}

// return all stop and ref lines from a given lanel
std::vector<lanelet::ConstLineString3d> query::stopLinesLanelet(const lanelet::ConstLanelet ll)
{
  std::vector<lanelet::ConstLineString3d> stoplines;

  // find stop lines referened by right ofway reg. elems.
  std::vector<std::shared_ptr<const lanelet::RightOfWay> > right_of_way_reg_elems =
      ll.regulatoryElementsAs<const lanelet::RightOfWay>();

  if (right_of_way_reg_elems.size() > 0)
  {
    // lanelet has a right of way elem elemetn
    for (auto j = right_of_way_reg_elems.begin(); j < right_of_way_reg_elems.end(); j++)
    {
      if ((*j)->getManeuver(ll) == lanelet::ManeuverType::Yield)
      {
        // lanelet has a yield reg. elem.
        lanelet::Optional<lanelet::ConstLineString3d> row_stopline_opt = (*j)->stopLine();
        if (!!row_stopline_opt)
          stoplines.push_back(row_stopline_opt.get());
      }
    }
  }

  // find stop lines referenced by traffic lights
  std::vector<std::shared_ptr<const lanelet::TrafficLight> > traffic_light_reg_elems =
      ll.regulatoryElementsAs<const lanelet::TrafficLight>();

  if (traffic_light_reg_elems.size() > 0)
  {
    // lanelet has a traffic light elem elemetn
    for (auto j = traffic_light_reg_elems.begin(); j < traffic_light_reg_elems.end(); j++)
    {
      lanelet::Optional<lanelet::ConstLineString3d> traffic_light_stopline_opt = (*j)->stopLine();
      if (!!traffic_light_stopline_opt)
        stoplines.push_back(traffic_light_stopline_opt.get());
    }
  }
  // find stop lines referenced by traffic signs
  std::vector<std::shared_ptr<const lanelet::TrafficSign> > traffic_sign_reg_elems =
      ll.regulatoryElementsAs<const lanelet::TrafficSign>();

  if (traffic_sign_reg_elems.size() > 0)
  {
    // lanelet has a traffic sign reg elem - can have multiple ref lines (but
    // stop sign shod have 1
    for (auto j = traffic_sign_reg_elems.begin(); j < traffic_sign_reg_elems.end(); j++)
    {
      lanelet::ConstLineStrings3d traffic_sign_stoplines = (*j)->refLines();
      if (traffic_sign_stoplines.size() > 0)
        stoplines.push_back(traffic_sign_stoplines.front());
    }
  }
  return stoplines;
}

std::vector<lanelet::ConstLineString3d> query::stopSignStopLines(const lanelet::ConstLanelets lanelets,
                                                                 const std::string& stop_sign_id)
{
  std::vector<lanelet::ConstLineString3d> stoplines;

  std::set<lanelet::Id> checklist;

  for (const auto& ll : lanelets)
  {
    // find stop lines referenced by traffic signs
    std::vector<std::shared_ptr<const lanelet::TrafficSign> > traffic_sign_reg_elems =
        ll.regulatoryElementsAs<const lanelet::TrafficSign>();

    if (traffic_sign_reg_elems.size() > 0)
    {
      // lanelet has a traffic sign reg elem - can have multiple ref lines (but
      // stop sign shod have 1
      for (const auto& ts : traffic_sign_reg_elems)
      {
        // skip if traffic sign is not stop sign
        if (ts->type() != stop_sign_id)
        {
          continue;
        }

        lanelet::ConstLineStrings3d traffic_sign_stoplines = ts->refLines();

        // only add new items
        if (traffic_sign_stoplines.size() > 0)
        {
          auto id = traffic_sign_stoplines.front().id();
          if (checklist.find(id) == checklist.end())
          {
            checklist.insert(id);
            stoplines.push_back(traffic_sign_stoplines.front());
          }
        }
      }
    }
  }
  return stoplines;
}

}  // namespace utils
}  // namespace lanelet
