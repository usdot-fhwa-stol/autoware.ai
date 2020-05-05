#pragma once

#include <ros/ros.h>
#include <lanelet2_extension/utility/query.h>

namespace lanelet
{
namespace utils
{
// Depending on the type of input, different recurse functions will be called
// primT: Point, LS, llt, regem, polygon

template <class primT>
void query::referenceFinder::run(primT prim, const lanelet::LaneletMapPtr ll_Map)
{
  recurse(prim, ll_Map, query::direction::CHECK_CHILD);
} 
} // namespace utils
} // namespace lanelet
