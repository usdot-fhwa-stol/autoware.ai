# lanelet2_extension package
This package contains external library for Lanelet2 and is meant to ease the use of Lanelet2 in CARMA and Autoware.

## CARMA Lanelet2 OSM File Format Changes

By default Lanelet2 supports a modified version of the OpenStreetMaps (OSM) file format to describe its roadways. See [here](https://github.com/fzi-forschungszentrum-informatik/Lanelet2/blob/master/lanelet2_maps/README.md) for an introduction. Part of this standard describes how road regulations are defined by interpreting tags on various Lanelet2 primitives. In addition, regulatory elements also fill a similar role. Unfortunately, this creates a confusing situation where some regulations are tied intrinsically to the physical markings on the road. For example, a double yellow line could never be crossed even if nearby signs instructed the user to do so around a work zone. To resolve this, while also preserving the description of the physical markings on the road, CARMA Platform supports an additional set of [regulatory elements](./docs/RegulatoryElements.md). Using these regulations separates all road regulations from physical road markings allowing for situations such as the work zone use case described above. In addition, all Lanelet2 Maps must follow the Autoware.ai tagging specification rules described in this document and [here](./docs/lanelet2_format_extension.md) due to the use of legacy map handling nodes. This is a requirement which may be depercated in the future. If a Lanelet2 map is loaded which does not contain these elements the carma_wm_ctrl package will make a best effort attempt to add them using Lanelet2's default tagging rules before forwarding the map to components using the carma_wm library.

### Local CARMA mods to this readme file

- Added section on CARMA file format requirements
  - 2/19/2019
  - Michael McConnell

## Lanelet Format for Autoware
Autoware uses extended Lanelet2 Format for Autoware, which means you need to add some tags to default OSM file if you want to fully use Lanelet2 maps. For details about custom tags, please refer to this [document](./docs/lanelet2_format_extension.md).

## Code API
### IO 
#### Autoware OSM Parser
Autoware Lanelet2 Format uses .osm extension as original Lanelet2.
However, there are some custom tags that is used by the parser.

Currently, this includes:
* overwriting x,y values with `local_x` and `local_y` tags.
* reading `<MapMetaInfo>` tag wich contains information about map format version and map version.

The parser is registered as "autoware_osm_handler" as lanelet parser

### Projection
#### MGRS Projector
MGRS projector projects latitude longitude into MGRS Coordinates. 

### Regulatory Elements
#### Autoware Traffic Light
Autoware Traffic Light class allows you to retrieve information about traffic lights.
Autoware Traffic Light class contains following members:
* traffic light shape
* light bulbs information of traffic lights
* stopline associated to traffic light

### Utility
#### Message Conversion
This contains functions to convert lanelet map objects into ROS messages.
Currently it contains following conversions:
* lanelet::LaneletMapPtr to/from lanelet_msgs::MapBinMsg
* lanelet::Point3d to geometry_msgs::Point
* lanelet::Point2d to geometry_msgs::Point
* lanelet::BasicPoint3d to geometry_msgs::Point

#### Query
This module contains functions to retrieve various information from maps.
e.g. crosswalks, trafficlights, stoplines

#### Utilties
This module contains other useful functions related to Lanelet.
e.g. matching waypoint with lanelets

### Visualization
Visualization contains functions to convert lanelet objects into visualization marker messages.
Currenly it contains following conversions:
* lanelet::Lanelet to Triangle Markers
* lanelet::LineString to LineStrip Markers
* TrafficLights to Triangle Markers

## Nodes
### lanelet2_extension_sample
Code for this explains how this lanelet2_extension library is used.
The executable is not meanto to do anything. 

### autoware_lanelet2_extension
This node checks if an .osm file follows the Autoware version of Lanelet2 format.
You can check by running:
```
rosrun lanelet2_extension autoware_lanelet2_validation _map_file:=<path/to/map.osm>
```
