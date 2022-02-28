cmake_minimum_required(VERSION 3.5)
project(amathutils_lib) # autoware math utility

find_package(carma_cmake_common REQUIRED)
find_package(ros_environment REQUIRED)
set(ROS_VERSION $ENV{ROS_VERSION})

if(${ROS_VERSION} EQUAL 1)

  find_package(autoware_build_flags REQUIRED)

  find_package(Eigen3 REQUIRED)

  if (NOT EIGEN3_FOUND)
      # Fallback to cmake_modules
      find_package(cmake_modules REQUIRED)
      find_package(Eigen REQUIRED)
      set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
      set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
      # Possibly map additional variables to the EIGEN3_ prefix.
  else ()
      set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
  endif ()

  find_package(catkin REQUIRED COMPONENTS
    autoware_msgs
    roscpp
    roslint
    tf
    tf2
    tf2_geometry_msgs
  )

  set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")

  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES amathutils_lib
    CATKIN_DEPENDS
      autoware_msgs
      tf
      tf2
      tf2_geometry_msgs
  )

  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
  )

  add_library(amathutils_lib
    src/Amathutils.cpp
    src/kalman_filter.cpp
    src/time_delay_kalman_filter.cpp
    src/butterworth_filter.cpp
  )
  target_link_libraries(amathutils_lib
    ${catkin_LIBRARIES}
  )

  add_dependencies(amathutils_lib
    ${catkin_EXPORTED_TARGETS}
  )

  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.hpp"
  )

  install(TARGETS amathutils_lib
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  file(GLOB_RECURSE ROSLINT_FILES
    LIST_DIRECTORIES false
    *.cpp *.h *.hpp
  )

  list(APPEND ROSLINT_CPP_OPTS "--extensions=cc,h,hpp,cpp,cu,cuh" "--filter=-build/c++11,-runtime/references")
  roslint_cpp(${ROSLINT_FILES})

  if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest_gtest(amathutils-test test/test_amathutils_lib.test test/src/test_amathutils_lib.cpp)
    target_link_libraries(amathutils-test ${catkin_LIBRARIES} amathutils_lib)
    add_rostest_gtest(test-kalman_filter 
      test/test_kalman_filter.test
      test/src/test_kalman_filter.cpp
      src/kalman_filter.cpp
      src/time_delay_kalman_filter.cpp
    )
    target_link_libraries(test-kalman_filter ${catkin_LIBRARIES})

    add_rostest_gtest(test-butterworth_filter 
      test/test_butterworth_filter.test
      test/src/test_butterworth_filter.cpp
      src/butterworth_filter.cpp
    )
    target_link_libraries(test-butterworth_filter ${catkin_LIBRARIES})
    roslint_add_test()
  endif()     

else() #ROS2

  find_package(ament_cmake_auto REQUIRED)
  ament_auto_find_build_dependencies()

  include_directories(
    amathutils_lib_ros2/include
  )

  ament_auto_add_library(amathutils_lib SHARED
    amathutils_lib_ros2/src/Amathutils.cpp
    amathutils_lib_ros2/src/butterworth_filter.cpp
    amathutils_lib_ros2/src/kalman_filter.cpp
    amathutils_lib_ros2/src/time_delay_kalman_filter.cpp
  )
  if(BUILD_TESTING)  

    find_package(ament_lint_auto REQUIRED)
    find_package(launch_testing_ament_cmake)
    ament_lint_auto_find_test_dependencies() # This populates the ${${PROJECT_NAME}_FOUND_TEST_DEPENDS} variable

    ament_add_gtest(test_amathutils_lib_ros2 amathutils_lib_ros2/test/test_amathutils_lib.cpp)
    ament_target_dependencies(test_amathutils_lib_ros2 ${${PROJECT_NAME}_FOUND_TEST_DEPENDS})
    target_link_libraries(test_amathutils_lib_ros2 amathutils_lib)

    ament_add_gtest(test_butterworth_filter amathutils_lib_ros2/test/test_butterworth_filter.cpp)
    ament_target_dependencies(test_butterworth_filter ${${PROJECT_NAME}_FOUND_TEST_DEPENDS})
    target_link_libraries(test_butterworth_filter amathutils_lib)

    ament_add_gtest(test_kalman_filter amathutils_lib_ros2/test/test_kalman_filter.cpp)
    ament_target_dependencies(test_kalman_filter ${${PROJECT_NAME}_FOUND_TEST_DEPENDS})
    target_link_libraries(test_kalman_filter amathutils_lib)

  endif()

  ament_auto_package()

endif()