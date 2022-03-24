cmake_minimum_required(VERSION 3.5.1)
project(jsk_recognition_msgs)

find_package(ros_environment REQUIRED)

set(ROS_VERSION $ENV{ROS_VERSION})

if(${ROS_VERSION} EQUAL 1) # ROS 1
  set(msg_directory ) # Empty as ROS1 defaults to msg/
  set(srv_directory ) # Empty as ROS1 defaults to srv/
else()
  set(msg_directory "msg/")
  set(srv_directory "srv/")
endif()

## Collection ROS version interoperable Message Paths 
file(GLOB msg_files msg/*.msg)
set(msgs )
foreach(filePath ${msg_files})
  get_filename_component(name ${filePath} NAME)
  set(msgs ${msgs} ${msg_directory}${name})
endforeach(filePath)

file(GLOB srv_files srv/*.srv)
set(srvs )
foreach(filePath ${srv_files})
  get_filename_component(name ${filePath} NAME)
  set(srvs ${srvs} ${srv_directory}${name})
endforeach(filePath)

if(${ROS_VERSION} EQUAL 1) # ROS 1

  # Add the ros1 only messages
  file(GLOB msg_files ros1_only_msg/*.msg)
  set(ros1_only_msgs )
  foreach(filePath ${msg_files})
    get_filename_component(name ${filePath} NAME)
    set(ros1_only_msgs ${ros1_only_msgs} ${name})
  endforeach(filePath)

  find_package(catkin REQUIRED
  std_msgs sensor_msgs geometry_msgs message_generation pcl_msgs)
  
  add_message_files(
    FILES
    ${msgs}
    )
  
  add_message_files(
    DIRECTORY ros1_only_msg
    FILES
    ${ros1_only_msgs}
    )

  # TODO(wkentaro): Most of srv files are duplicated in jsk_pcl_ros,
  #   and jsk_perception, so those in the packages should be removed.
  add_service_files(FILES
    ${srvs}
    )

  generate_messages(
    DEPENDENCIES
    std_msgs sensor_msgs geometry_msgs pcl_msgs
  )
  catkin_package(
    CATKIN_DEPENDS std_msgs sensor_msgs geometry_msgs pcl_msgs
  )

  # From https://github.com/jsk-ros-pkg/jsk_recognition/pull/2345
  # Install header files directly into ${CATKIN_PACKAGE_INCLUDE_DESTINATION}.
  # If the package has setup.py and modules under src/${PROJECT_NAME}/,
  # install python executables directly into ${CATKIN_PACKAGE_BIN_DESTINATION}.
  # However, if it doesn't have setup.py, directories including python executables
  # should be installed recursively into ${CATKIN_PACKAGE_SHARE_DESTINATION}.
  # Also, other directories like 'launch' or 'sample' must be installed
  # recursively into ${CATKIN_PACKAGE_SHARE_DESTINATION}.
  # Be careful that 'launch' and 'launch/' are different: the former is directory
  # and the latter is each content.
  install(DIRECTORY sample scripts
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    USE_SOURCE_PERMISSIONS
  )

  if (CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest(test/object_array_publisher.test)
    add_rostest(test/people_pose_array_to_pose_array.test)
    add_rostest(test/plot_data_to_csv.test)
    add_rostest(test/save_mesh_server.test)
  endif()

else() # ROS 2 

  # Add the ros2 only messages
  file(GLOB msg_files ros2_only_msg/*.msg)
  foreach(filePath ${msg_files})
    get_filename_component(name ${filePath} NAME)
    set(msgs ${msgs} ros2_only_msg/${name})
  endforeach(filePath)

  find_package(rosidl_default_generators REQUIRED)
  find_package(ament_cmake REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(pcl_msgs REQUIRED)

  rosidl_generate_interfaces(
    ${PROJECT_NAME}
      ${msgs}
      ${srvs}
    DEPENDENCIES
      std_msgs
      sensor_msgs
      geometry_msgs
      pcl_msgs
  )

  ament_export_dependencies(rosidl_default_runtime)
  ament_package()

endif()

