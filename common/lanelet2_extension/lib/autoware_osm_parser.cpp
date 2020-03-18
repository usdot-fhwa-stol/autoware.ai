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
 * Authors: Ryohsuke Mitsudome
 */

#include <lanelet2_extension/io/autoware_osm_parser.h>

#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_io/io_handlers/Factory.h>
#include <lanelet2_io/io_handlers/OsmFile.h>
#include <lanelet2_io/io_handlers/OsmHandler.h>
#include <string>
#include <sstream>
#include <boost/algorithm/string.hpp> 

namespace lanelet
{
namespace io_handlers
{
std::unique_ptr<LaneletMap> AutowareOsmParser::parse(const std::string& filename, ErrorMessages& errors) const
{
  auto map = OsmParser::parse(filename, errors);

  // overwrite x and y values if there are local_x, local_y tags
  for (Point3d point : map->pointLayer)
  {
    if (point.hasAttribute("local_x"))
    {
      point.x() = point.attribute("local_x").asDouble().value();
    }
    if (point.hasAttribute("local_y"))
    {
      point.y() = point.attribute("local_y").asDouble().value();
    }
  }

  // rerun align function in just in case
  for (Lanelet& lanelet : map->laneletLayer)
  {
    LineString3d new_left, new_right;
    std::tie(new_left, new_right) = geometry::align(lanelet.leftBound(), lanelet.rightBound());
    lanelet.setLeftBound(new_left);
    lanelet.setRightBound(new_right);
  }

  return map;
}

namespace
{
RegisterParser<AutowareOsmParser> regParser;
}

void AutowareOsmParser::parseVersions(const std::string& filename, std::string* format_version,
                                      std::string* map_version)
{
  if (format_version == nullptr || map_version == nullptr)
  {
    std::cerr << __FUNCTION__ << ": either format_version or map_version is null pointer!";
    return;
  }

  pugi::xml_document doc;
  auto result = doc.load_file(filename.c_str());
  if (!result)
  {
    throw lanelet::ParseError(std::string("Errors occured while parsing osm file: ") + result.description());
  }

  auto osmNode = doc.child("osm");
  auto metainfo = osmNode.child("MetaInfo");
  if (metainfo.attribute("format_version"))
    *format_version = metainfo.attribute("format_version").value();
  if (metainfo.attribute("map_version"))
    *map_version = metainfo.attribute("map_version").value();
}

void AutowareOsmParser::parseMapParams (const std::string& filename, int* projector_type, std::string* target_frame)
{
  if (target_frame == nullptr)
  {
    throw lanelet::ParseError(std::string("In function ") + __FUNCTION__ + 
    std::string(": Errors occured while parsing .osm file - target frame is a null pointer!"));
    return;
  }

  pugi::xml_document doc;
  auto result = doc.load_file(filename.c_str());
  if (!result)
  {
    throw lanelet::ParseError(std::string("Errors occured while parsing .osm file: ") + result.description());
  }

  auto osmNode = doc.child("osm");
  auto geoRef = osmNode.child("geoReference");

  if (geoRef)
  {
    std::string raw_geo_ref = geoRef.child_value();

    // Filter unnecessary part out of georeference.
    std::vector<std::string> buffer;
    boost::split(buffer,  raw_geo_ref, boost::is_any_of(" "));

    for (int i = 0; i < buffer.size(); i++)
    {
      if (!boost::algorithm::contains(buffer[i], "+geoidgrids")) {
        target_frame->append(buffer[i] + " "); //geo reference value
      } else {
        std::cerr << "Removing +geoidgrids from input projection as this is not currently supported by AutowareOsmParser" << std::endl;
      }
    }
  }
  else
  {
    throw lanelet::ParseError(std::string("While parsing .osm file, geoReference was not found!"));
  }

  // Default values
  *projector_type = 1; // default value for autoware.ai projector type for CARMA purposes
}
} // namespace io_handlers
} // namespace lanelet
