# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 3.5.1)
project(jsk_hark_msgs)

find_package(ros_environment REQUIRED)

set(ROS_VERSION $ENV{ROS_VERSION})

if(${ROS_VERSION} EQUAL 1) # ROS 1
  set(msg_directory ) # Empty as ROS1 defaults to msg/
else()
  set(msg_directory "msg/")
endif()

## Collection Message Paths 
file(GLOB msg_files msg/*.msg)
set(msgs )
foreach(filePath ${msg_files})
  get_filename_component(name ${filePath} NAME)
  set(msgs ${msgs} ${msg_directory}${name})
endforeach(filePath)


if(${ROS_VERSION} EQUAL 1) # ROS 1
  
  # Load catkin and all dependencies required for this package
  # TODO: remove all from COMPONENTS that are not catkin packages.
  find_package(catkin REQUIRED message_generation std_msgs)

  # CATKIN_MIGRATION: removed during catkin migration
  # cmake_minimum_required(VERSION 2.4.6)

  # CATKIN_MIGRATION: removed during catkin migration
  # include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

  # Set the build type.  Options are:
  #  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
  #  Debug          : w/ debug symbols, w/o optimization
  #  Release        : w/o debug symbols, w/ optimization
  #  RelWithDebInfo : w/ debug symbols, w/ optimization
  #  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
  #set(ROS_BUILD_TYPE RelWithDebInfo)


  # CATKIN_MIGRATION: removed during catkin migration
  # rosbuild_init()

  #set the default path for built executables to the "bin" directory
  set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
  #set the default path for built libraries to the "lib" directory
  set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

  #uncomment if you have defined messages
  add_message_files(
    DIRECTORY msg
    FILES ${msgs}
  )
  #uncomment if you have defined services
  #add_service_files(
    # FILES
    # TODO: List your msg files here
  #)

  #common commands for building c++ executables and libraries
  #add_library(${PROJECT_NAME} src/example.cpp)
  #target_link_libraries(${PROJECT_NAME} another_library)
  #
  # CATKIN_MIGRATION: removed during catkin migration
  # rosbuild_add_boost_directories()
  #find_package(Boost REQUIRED COMPONENTS thread)
  include_directories(${Boost_INCLUDE_DIRS})
  #target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
  #add_executable(example examples/example.cpp)
  #target_link_libraries(example ${PROJECT_NAME})
  ## Generate added messages and services with any dependencies listed here
  generate_messages(
    DEPENDENCIES std_msgs
  )
  # TODO: fill in what other packages will need to use this package
  ## LIBRARIES: libraries you create in this project that dependent projects also need
  ## CATKIN_DEPENDS: catkin_packages dependent projects also need
  ## DEPENDS: system dependencies of this project that dependent projects also need
  catkin_package(
      DEPENDS  # TODO
      CATKIN_DEPENDS std_msgs # TODO
      INCLUDE_DIRS # TODO include
      LIBRARIES # TODO
  )

else() # ROS 2

  find_package(rosidl_default_generators REQUIRED)
  find_package(ament_cmake REQUIRED)
  find_package(std_msgs REQUIRED)

  rosidl_generate_interfaces(
    ${PROJECT_NAME}
      ${msgs}
    DEPENDENCIES
      std_msgs
  )

  ament_export_dependencies(rosidl_default_runtime)
  ament_package()

endif()

