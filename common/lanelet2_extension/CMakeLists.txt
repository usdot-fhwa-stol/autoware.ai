cmake_minimum_required(VERSION 3.5.1)
project(lanelet2_extension)

#
#- Modified to install add CARMA regulatory elements to build
#    - 2/19/2019
#    - Michael McConnell
#

set(CMAKE_CXX_FLAGS "-std=c++17")

find_package(PkgConfig)
find_path(GeographicLib_INCLUDE_DIR GeographicLib/Config.h
  PATH_SUFFIXES GeographicLib
  )
set(GeographicLib_INCLUDE_DIRS ${GeographicLib_INCLUDE_DIR})

find_library(GeographicLib_LIBRARIES
  NAMES Geographic
)

find_library(PUGIXML_LIBRARIES
  NAMES pugixml
)

find_path(PUGIXML_INCLUDE_DIRS
  NAMES pugixml.hpp
  PATH_SUFFIXES pugixml
)

find_package(PROJ4 REQUIRED)
find_package(Boost REQUIRED)


set(deps_using_buildtool
  lanelet2_core
  lanelet2_io
  lanelet2_maps
  lanelet2_projection
  lanelet2_routing
  lanelet2_traffic_rules
  lanelet2_validation
  hardcoded_params
)


# Extract environment variable value
set(LANELET2_EXTENSION_LOGGER_TYPE $ENV{LANELET2_EXTENSION_LOGGER_TYPE})
set(ROS_VERSION $ENV{ROS_VERSION})

if(LANELET2_EXTENSION_LOGGER_TYPE EQUAL 1 )

  message("lanelet2_extension using std::cout and cerr for logging")
  add_definitions(-DLANELET2_EXTENSION_LOGGER_TYPE=1)

  set(catkin_depends_deps
    ${deps_using_buildtool}
  )

elseif(ROS_VERSION EQUAL 2)

   # ROS 2 logging
  message("lanelet2_extension using rclcpp for logging")
  add_definitions(-DLANELET2_EXTENSION_LOGGER_TYPE=0) 
  add_definitions(-DLANELET2_EXTENSION_ROS_VERSION=2)
  set(LOGGER_DEP rclcpp)
  find_package(rclcpp REQUIRED)

  set(catkin_depends_deps
    ${deps_using_buildtool} # Don't include rclcpp as its not a catkin package
  )

else() # Default to ROS 1 logging

  message("lanelet2_extension using roscpp for logging")
  add_definitions(-DLANELET2_EXTENSION_LOGGER_TYPE=0)
  add_definitions(-DLANELET2_EXTENSION_ROS_VERSION=1)
  
  set(LOGGER_DEP roscpp)
  
  find_package(catkin REQUIRED COMPONENTS
    ${LOGGER_DEP}
  )

  set(catkin_depends_deps
    ${LOGGER_DEP} ${deps_using_buildtool} # Include roscpp as its a catkin dep
  )
  
endif()

find_package(catkin REQUIRED COMPONENTS
  ${deps_using_buildtool}
)

message("Logger includes: " ${${LOGGER_DEP}_INCLUDE_DIRS})
message("Logger Libs: " ${${LOGGER_DEP}_LIBRARIES})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES lanelet2_extension_lib
  CATKIN_DEPENDS ${catkin_depends_deps}
  DEPENDS Boost ${LOGGER_DEP}
)

include_directories(
  include
  ${GeographicLib_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${PUGIXML_INCLUDE_DIRS}
  ${PROJ4_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${${LOGGER_DEP}_INCLUDE_DIRS}
)

add_definitions (${GeographicLib_DEFINITIONS})

add_library( lanelet2_extension_lib
  lib/autoware_osm_parser.cpp
  lib/autoware_traffic_light.cpp
  lib/local_frame_projector.cpp
  lib/mgrs_projector.cpp
  lib/CarmaUSTrafficRules.cpp
  lib/CarmaTrafficLight.cpp
  lib/PassingControlLine.cpp
  lib/StopRule.cpp
  lib/DigitalSpeedLimit.cpp
  lib/DigitalMinimumGap.cpp
  lib/RegionAccessRule.cpp
  lib/DirectionOfTravel.cpp
)

add_dependencies(lanelet2_extension_lib
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(lanelet2_extension_lib
  ${catkin_LIBRARIES}
  ${GeographicLib_LIBRARIES}
  ${PROJ4_LIBRARIES}
  ${Boost_LIBRARIES}
  ${${LOGGER_DEP}_LIBRARIES}
)

install(TARGETS lanelet2_extension_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
 catkin_add_gmock(projector-test test/src/test_projector.cpp)
 target_link_libraries(projector-test ${catkin_LIBRARIES} ${${LOGGER_DEP}_LIBRARIES} lanelet2_extension_lib)
 catkin_add_gmock(regulatory_elements-test test/src/test_regulatory_elements.cpp)
 target_link_libraries(regulatory_elements-test ${catkin_LIBRARIES} ${${LOGGER_DEP}_LIBRARIES} lanelet2_extension_lib)

 catkin_add_gmock(carma-additions-test
  test/src/CarmaTestsMain.cpp
  test/src/RegionAccessRuleTest.cpp
  test/src/PassingControlLineTest.cpp
  test/src/StopRuleTest.cpp
  test/src/DirectionOfTravelTest.cpp
  test/src/DigitalSpeedLimitTest.cpp
  test/src/DigitalMinimumGapTest.cpp
  test/src/CarmaUSTrafficRulesTest.cpp
  test/src/MapLoadingTest.cpp
  test/src/test_local_projector.cpp
  test/src/CarmaTrafficTest.cpp
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test # Add test directory as working directory for unit tests
 )

 target_link_libraries(carma-additions-test lanelet2_extension_lib ${catkin_LIBRARIES} ${${LOGGER_DEP}_LIBRARIES})
endif()
