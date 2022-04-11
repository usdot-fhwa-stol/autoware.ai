cmake_minimum_required(VERSION 3.5.1)
project(autoware_system_msgs)

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
  
  # Add the ros1 only messages
  file(GLOB msg_files ros1_only_msg/*.msg)
  set(ros1_msgs )
  foreach(filePath ${msg_files})
    get_filename_component(name ${filePath} NAME)
    set(ros1_msgs ${ros1_msgs} ${name})
  endforeach(filePath)

  find_package(catkin REQUIRED COMPONENTS
    message_generation
    rosgraph_msgs
    std_msgs
  )

  add_message_files(
    FILES
      ${msgs}
  )

  add_message_files(
    DIRECTORY ros1_only_msg
    FILES
      ${ros1_msgs}
  )

  generate_messages(DEPENDENCIES
    rosgraph_msgs
    std_msgs
  )

  catkin_package(
    CATKIN_DEPENDS
      message_runtime
      rosgraph_msgs
      std_msgs
  )

else()

  # Add the ros2 only messages
  file(GLOB msg_files ros2_only_msg/*.msg)
  foreach(filePath ${msg_files})
    get_filename_component(name ${filePath} NAME)
    set(msgs ${msgs} ros2_only_msg/${name})
  endforeach(filePath)

  find_package(rosidl_default_generators REQUIRED)
  find_package(ament_cmake REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(rosgraph_msgs REQUIRED)

  rosidl_generate_interfaces(
    ${PROJECT_NAME}
      ${msgs}
    DEPENDENCIES
      std_msgs
      rosgraph_msgs
  )

  ament_export_dependencies(rosidl_default_runtime)
  ament_package()

endif()

