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

#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/io_handlers/Factory.h>
#include <lanelet2_io/io_handlers/Writer.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <lanelet2_extension/projection/mgrs_projector.h>
#include <lanelet2_extension/regulatory_elements/autoware_traffic_light.h>

#include <iostream>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <pugixml.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>

namespace validation
{
namespace keyword
{
constexpr const char* Id = "id";
constexpr const char* Osm = "osm";
constexpr const char* Tag = "tag";
constexpr const char* Key = "k";
constexpr const char* Node = "node";
constexpr const char* Elevation = "ele";
}  // namespace keyword

void printUsage()
{
  std::cout << "Usage:" << std::endl
            << "rosrun autoware_lanelet2_ros_interface autoware_lanelet2_validation"
               "_map_file:=<path to osm file>"
            << std::endl;
}


class Validation : public rclcpp::Node
{
 
    public:

    Validation() : Node("Validation") {}

    void configure(const rclcpp_lifecycle::State &){
        
        get_parameter<std::string>("map_file", map_path_);

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn SUCCESS;
    }

    void activate(const rclcpp_lifecycle::State &){

        if( !this->get_parameter("map_file", map_path_) )
        {
            RCLCPP_FATAL_STREAM(get_logger(), "failed find map_file parameter! No file to load");
            printUsage();
            rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ERROR;
        }

        lanelet::LaneletMapPtr lanelet_map;
        lanelet::ErrorMessages errors;
        lanelet::projection::MGRSProjector projector;
        lanelet_map = lanelet::load(map_path_, "autoware_osm_handler", projector, &errors);

        std::cout << "starting validation" << std::endl;

        validateElevationTag(map_path_);
        validateTrafficLight(lanelet_map);
        validateTurnDirection(lanelet_map);

        std::cout << "finished validation" << std::endl;
    }

    private:

        std::string map_path_="";

        void validateElevationTag(const std::string filename)
        {
            pugi::xml_document doc;
            auto result = doc.load_file(filename.c_str());
            if (!result)
            {
                RCLCPP_FATAL_STREAM(get_logger(), result.description());
                exit(1);
            }

            auto osmNode = doc.child("osm");
            for (auto node = osmNode.child(keyword::Node); node;  // NOLINT
                node = node.next_sibling(keyword::Node))
            {
                const auto id = node.attribute(keyword::Id).as_llong(lanelet::InvalId);
                if (!node.find_child_by_attribute(keyword::Tag, keyword::Key, keyword::Elevation))
                {
                RCLCPP_ERROR_STREAM(get_logger(), "failed to find elevation tag for node: " << id);
                }
            }
        }


        void validateTrafficLight(const lanelet::LaneletMapPtr lanelet_map)
        {
            if (!lanelet_map)
            {
                RCLCPP_FATAL_STREAM(get_logger(), "Missing map. Are you sure you set correct path for map?");
                exit(1);
            }

            for (auto lanelet : lanelet_map->laneletLayer)
            {
                auto autoware_traffic_lights = lanelet.regulatoryElementsAs<lanelet::autoware::AutowareTrafficLight>();
                if (autoware_traffic_lights.empty())
                continue;
                for (auto light : autoware_traffic_lights)
                {
                if (light->lightBulbs().size() == 0)
                {
                    RCLCPP_WARN_STREAM(get_logger(), "regulatory element traffic light " << light->id()
                                                                        << " is missing optional light_bulb member. You won't "
                                                                        "be able to use region_tlr node with this map");
                }
                for (auto light_string : light->lightBulbs())
                {
                    if (!light_string.hasAttribute("traffic_light_id"))
                    {
                    RCLCPP_ERROR_STREAM(get_logger(), "light_bulb " << light_string.id() << " is missing traffic_light_id tag");
                    }
                }
                for (auto base_string_or_poly : light->trafficLights())
                {
                    if (!base_string_or_poly.isLineString())
                    {
                    RCLCPP_ERROR_STREAM(get_logger(), "traffic_light " << base_string_or_poly.id()
                                                        << " is polygon, and only linestring class is currently supported for "
                                                        "traffic lights");
                    }
                    auto base_string = static_cast<lanelet::LineString3d>(base_string_or_poly);
                    if (!base_string.hasAttribute("height"))
                    {
                    RCLCPP_ERROR_STREAM(get_logger(), "traffic_light " << base_string.id() << " is missing height tag");
                    }
                }
                }
            }
        }

        void validateTurnDirection(const lanelet::LaneletMapPtr lanelet_map)
        {
        if (!lanelet_map)
        {
            RCLCPP_FATAL_STREAM(get_logger(), "Missing map. Are you sure you set correct path for map?");
            exit(1);
        }

        lanelet::traffic_rules::TrafficRulesPtr traffic_rules =
            lanelet::traffic_rules::TrafficRulesFactory::create(lanelet::Locations::Germany, lanelet::Participants::Vehicle);
        lanelet::routing::RoutingGraphPtr vehicle_graph = lanelet::routing::RoutingGraph::build(*lanelet_map, *traffic_rules);

        for (const auto& lanelet : lanelet_map->laneletLayer)
        {
            if (!traffic_rules->canPass(lanelet))
            {
            continue;
            }

            const auto conflicting_lanelets_or_areas = vehicle_graph->conflicting(lanelet);
            if (conflicting_lanelets_or_areas.size() == 0)
            continue;
            if (!lanelet.hasAttribute("turn_direction"))
            {
            RCLCPP_ERROR_STREAM(get_logger(), "lanelet " << lanelet.id() << " seems to be intersecting other lanelet, but does "
                                                            "not have turn_direction tagging.");
            }
        }
        }
    
};


}  // namespace



int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<validation::Validation>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
