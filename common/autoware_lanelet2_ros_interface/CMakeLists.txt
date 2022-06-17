cmake_minimum_required(VERSION 3.5)
project(autoware_lanelet2_ros_interface)

find_package(carma_cmake_common REQUIRED)
find_package(ros_environment REQUIRED)
set(ROS_VERSION $ENV{ROS_VERSION})

#
#- Modified to install add CARMA regulatory elements to build
#    - 2/19/2019
#    - Michael McConnell
#

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


if(${ROS_VERSION} EQUAL 1)

  set(CMAKE_CXX_FLAGS "-std=c++14 -fext-numeric-literals")
  set(CMAKE_CXX_STANDARD 14)

  find_package(autoware_build_flags REQUIRED)

  find_package(PROJ4 REQUIRED)

  find_package(catkin REQUIRED COMPONENTS
    amathutils_lib
    roscpp
    lanelet2_core
    lanelet2_io
    lanelet2_maps
    lanelet2_projection
    lanelet2_routing
    lanelet2_traffic_rules
    lanelet2_validation
    lanelet2_extension
    autoware_lanelet2_msgs
    autoware_msgs
    geometry_msgs
    visualization_msgs
    roslint
  )

  set(ROSLINT_CPP_OPTS "--filter=-build/c++14")
  roslint_cpp()

  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES autoware_lanelet2_ros_interface_lib
    CATKIN_DEPENDS amathutils_lib roscpp lanelet2_core lanelet2_io lanelet2_maps lanelet2_projection lanelet2_routing lanelet2_traffic_rules lanelet2_validation autoware_lanelet2_msgs autoware_msgs geometry_msgs visualization_msgs lanelet2_extension
  )

  include_directories(
    include
    ${GeographicLib_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${PUGIXML_INCLUDE_DIRS}
    ${PROJ4_INCLUDE_DIRS}
  )

  add_definitions (${GeographicLib_DEFINITIONS})

  add_library( autoware_lanelet2_ros_interface_lib SHARED
    lib/message_conversion.cpp
    lib/query.cpp
    lib/utilities.cpp
    lib/visualization.cpp
  )

  add_dependencies(autoware_lanelet2_ros_interface_lib
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
  )

  target_link_libraries(autoware_lanelet2_ros_interface_lib
    ${catkin_LIBRARIES}
    ${GeographicLib_LIBRARIES}
    ${PROJ4_LIBRARIES}
  )

  add_executable(autoware_lanelet2_ros_interface_sample src/sample_code.cpp)
  add_dependencies(autoware_lanelet2_ros_interface_sample ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(autoware_lanelet2_ros_interface_sample
    ${catkin_LIBRARIES}
    autoware_lanelet2_ros_interface_lib
  )

  add_executable(autoware_lanelet2_validation src/validation.cpp)
  add_dependencies(autoware_lanelet2_validation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(autoware_lanelet2_validation
    ${catkin_LIBRARIES}
    ${PUGIXML_LIBRAREIS}
    autoware_lanelet2_ros_interface_lib
  )

  install(TARGETS autoware_lanelet2_ros_interface_lib autoware_lanelet2_ros_interface_sample autoware_lanelet2_validation
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

  if(CATKIN_ENABLE_TESTING)
    roslint_add_test()
    find_package(rostest REQUIRED)
    set(CMAKE_CXX_FLAGS "-std=c++14 -fext-numeric-literals")
    set(CMAKE_CXX_STANDARD 14)
    add_rostest_gtest(message_conversion-test test/test_message_conversion.test test/src/test_message_conversion.cpp)
    target_link_libraries(message_conversion-test ${catkin_LIBRARIES} autoware_lanelet2_ros_interface_lib)
    add_rostest_gtest(query-test test/test_query.test test/src/test_query.cpp)
    target_link_libraries(query-test ${catkin_LIBRARIES} autoware_lanelet2_ros_interface_lib)
    add_rostest_gtest(utilities-test test/test_utilities.test test/src/test_utilities.cpp)
    target_link_libraries(utilities-test ${catkin_LIBRARIES} autoware_lanelet2_ros_interface_lib)

  endif()

else() #ROS2

  ## Find dependencies using ament auto
  find_package(ament_cmake_auto REQUIRED)
  ament_auto_find_build_dependencies()

  find_package(lanelet2_extension REQUIRED)

  # Includes
  include_directories(
    include/autoware_lanelet2_ros2_interface
    ${GeographicLib_INCLUDE_DIRS}
    ${PUGIXML_INCLUDE_DIRS}
    ${PROJ4_INCLUDE_DIRS}
  )

  # Build
  ament_auto_add_library(autoware_lanelet2_ros2_interface_lib SHARED
          lib/autoware_lanelet2_ros2_interface/message_conversion.cpp
          lib/autoware_lanelet2_ros2_interface/query.cpp
          lib/autoware_lanelet2_ros2_interface/utilities.cpp
          lib/autoware_lanelet2_ros2_interface/visualization.cpp
          ${GeographicLib_LIBRARIES}
          ${PROJ4_LIBRARIES}
  )

  ament_auto_add_executable(autoware_lanelet2_ros2_interface_sample
          src/autoware_lanelet2_ros2_interface/sample_code.cpp 
  )
  target_link_libraries(autoware_lanelet2_ros2_interface_sample 
    autoware_lanelet2_ros2_interface_lib
  )

  ament_auto_add_executable(autoware_lanelet2_ros2_interface_validation
          src/autoware_lanelet2_ros2_interface/validation.cpp 
  )
  target_link_libraries(autoware_lanelet2_ros2_interface_validation 
    autoware_lanelet2_ros2_interface_lib
  )

  # Testing
  if(BUILD_TESTING)  

    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies() # This populates the ${${PROJECT_NAME}_FOUND_TEST_DEPENDS} variable

    ament_add_gtest(message_conversion-test test/autoware_lanelet2_ros2_interface/src/test_message_conversion.cpp)
    ament_target_dependencies(message_conversion-test ${${PROJECT_NAME}_FOUND_TEST_DEPENDS})
    target_link_libraries(message_conversion-test autoware_lanelet2_ros2_interface_lib)

    ament_add_gtest(query-test test/autoware_lanelet2_ros2_interface/src/test_query.cpp)
    ament_target_dependencies(query-test ${${PROJECT_NAME}_FOUND_TEST_DEPENDS})
    target_link_libraries(query-test autoware_lanelet2_ros2_interface_lib)

    ament_add_gtest(utilities-test test/autoware_lanelet2_ros2_interface/src/test_utilities.cpp)
    ament_target_dependencies(utilities-test ${${PROJECT_NAME}_FOUND_TEST_DEPENDS})
    target_link_libraries(utilities-test autoware_lanelet2_ros2_interface_lib)

  endif()


  # Install
  ament_auto_package()

endif()
