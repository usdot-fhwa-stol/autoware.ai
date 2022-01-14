/*
 * Copyright (C) 2020-2021 LEIDOS.
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

#pragma once

// NOTE: This file is not meant to be included directly. Include query.h instead
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
 * @return        [References object with referenced element sets for each primitive layers]
 * NOTE: Polygons and Compound primitives such as LaneletOrArea are not currently supported
 */
//template <class primT>
void resolveMemory (WeakLanelet& prim, const lanelet::LaneletMapPtr ll_Map);


// Helper visitor class for finding all references in other primities for a given RuleParameter, which is boost::variant
struct RecurseVisitor : public RuleParameterVisitor {
  explicit RecurseVisitor (const lanelet::LaneletMapPtr ll_Map, query::direction check_dir, query::References& rfs) :
                  ll_Map_(ll_Map), check_dir_(check_dir), rfs_(rfs){}
  void operator()(const ConstPoint3d& p) override {recurse(p, ll_Map_, check_dir_, rfs_);}
  void operator()(const ConstLineString3d& ls) override{ recurse(ls, ll_Map_, check_dir_, rfs_);}
  void operator()(const ConstWeakLanelet& llt) override { 
    if (llt.expired()) {  // NOLINT
      return;
    }
    recurse(llt.lock(), ll_Map_, check_dir_, rfs_);
  }
  void operator()(const ConstWeakArea& area) override { 
    if (area.expired()) {  // NOLINT
      return;
    }
    recurse(area.lock(), ll_Map_, check_dir_, rfs_);}
  private:
  lanelet::LaneletMapPtr ll_Map_;
  query::direction check_dir_;
  query::References &rfs_;
};

// TODO create function for each
// Helper visitor class for finding existing elements with same id in the map and changing input to it
struct ResolveMemoryVisitor : public lanelet::internal::MisheelMutableParameterVisitor {
  explicit ResolveMemoryVisitor (lanelet::LaneletMapPtr ll_Map) : ll_Map_(ll_Map) {std::cerr << "It was created!\n";}
  void operator() (Point3d& p) override {}
  void operator() (LineString3d& ls) override {}

  void operator() (Polygon3d& /*unused*/) override {}
  void operator() (WeakLanelet& llt) override { 
    std::cerr << "inside operator for WeakLanelet!" << std::endl;
    if (llt.expired()) {  // NOLINT
      std::cerr << "It is expired!!" << std::endl;
      return;
    }
    resolveMemory(llt, ll_Map_);
  }
  void operator()(WeakArea& area) {}
  private:
  lanelet::LaneletMapPtr ll_Map_;
};

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
  recurse(prim, ll_Map, query::direction::CHECK_PARENT, references);
  return references;
} 
} // namespace utils
} // namespace lanelet