#pragma once

namespace lanelet
{
namespace utils
{
// Declaration of recurse func. Following recurse functions are helper functions for each primitives
void recurse (const lanelet::ConstPoint3d& prim,const lanelet::LaneletMapPtr ll_Map, query::direction check_dir, query::References& rfs);
void recurse (const lanelet::ConstLineString3d& prim, const lanelet::LaneletMapPtr ll_Map, query::direction check_dir, query::References& rfs);
void recurse (const lanelet::ConstLanelet& prim,const lanelet::LaneletMapPtr ll_Map, query::direction check_dir, query::References& rfs);
void recurse (const lanelet::ConstArea& prim,const lanelet::LaneletMapPtr ll_Map, query::direction check_dir, query::References& rfs);
void recurse (const lanelet::RegulatoryElementConstPtr& prim_ptr,const lanelet::LaneletMapPtr ll_Map, query::direction check_dir, query::References& rfs);

/**
 * [findReferences finds all primitives that reference the given primitive in a given map]
 * @param  ll_Map [input lanelet map]
 * @return        [References object with referenced element sets (including the input if applicable) for each primitive layers]
 */
template <class primT>
query::References query::findReferences(const primT& prim, const lanelet::LaneletMapPtr ll_Map)
{
  query::References references;
  recurse(prim, ll_Map, query::direction::CHECK_CHILD, references);
  return references;
} 
} // namespace utils
} // namespace lanelet
