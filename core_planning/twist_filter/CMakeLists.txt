# Modifications:
# - Add unit testing for CARMA velocity limit code and adjust CMake to account
#   for refactoring required in that process
#    - Kyle Rush
#    - 9/13/2020

cmake_minimum_required(VERSION 2.8.3)
project(twist_filter)

find_package(carma_cmake_common REQUIRED)
carma_check_ros_version(1)

find_package(autoware_build_flags REQUIRED)

find_package(
  catkin REQUIRED COMPONENTS
    autoware_config_msgs
    autoware_health_checker
    autoware_msgs
    geometry_msgs
    roscpp
    rostest
    std_msgs
    roslint
    hardcoded_params
)

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS
    autoware_config_msgs
    autoware_health_checker
    autoware_msgs
    geometry_msgs
    roscpp
    std_msgs
)

set(ROSLINT_CPP_OPTS "--filter=-build/c++14")
roslint_cpp()

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(twist_filter 
  src/twist_filter_node.cpp
  src/velocity_limit.cpp
  src/accel_limit.cpp
  src/twist_filter.cpp)
target_link_libraries(twist_filter ${catkin_LIBRARIES})
add_dependencies(twist_filter ${catkin_EXPORTED_TARGETS})

install(
  TARGETS twist_filter
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  roslint_add_test()
  catkin_add_gtest(${PROJECT_NAME}-test 
    test/test_twist_filter.cpp
    src/accel_limit.cpp
    src/velocity_limit.cpp)
  target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES})
endif ()
