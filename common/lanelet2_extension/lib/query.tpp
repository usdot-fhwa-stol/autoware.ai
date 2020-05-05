#pragma once

namespace lanelet
{
namespace utils
{
// Declaration of recurse func. Following recurse functions are helper functions for each primitives
void recurse (lanelet::Point3d prim,const lanelet::LaneletMapPtr ll_Map, query::direction check_dir, query::References& rfs);
void recurse (lanelet::LineString3d prim, const lanelet::LaneletMapPtr ll_Map, query::direction check_dir, query::References& rfs);
void recurse (lanelet::Lanelet prim,const lanelet::LaneletMapPtr ll_Map, query::direction check_dir, query::References& rfs);
void recurse (lanelet::Area prim,const lanelet::LaneletMapPtr ll_Map, query::direction check_dir, query::References& rfs);
void recurse (lanelet::RegulatoryElementPtr prim_ptr,const lanelet::LaneletMapPtr ll_Map, query::direction check_dir, query::References& rfs);

// Depending on the type of input, different recurse functions will be called
// primT: Point, LS, llt, regem, polygon
template <class primT>
query::References query::findReferences(primT prim, const lanelet::LaneletMapPtr ll_Map)
{
  query::References references;
  recurse(prim, ll_Map, query::direction::CHECK_CHILD, references);
  return references;
} 
} // namespace utils
} // namespace lanelet
