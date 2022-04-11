cmake_minimum_required(VERSION 3.5.1)
project(autoware_msgs)

find_package(ros_environment REQUIRED)

set(ROS_VERSION $ENV{ROS_VERSION})

if(${ROS_VERSION} EQUAL 1) # ROS 1
  set(msg_directory ) # Empty as ROS1 defaults to msg/
  set(srv_directory ) # Empty as ROS1 defaults to srv/
else()
  set(msg_directory "msg/")
  set(srv_directory "srv/")
endif()

## Collection Message Paths 
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

  find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    jsk_recognition_msgs
    message_generation
    sensor_msgs
    std_msgs
  )

  add_message_files(
        DIRECTORY msg
        FILES
          ${msgs}
  )

  add_service_files(
    FILES ${srvs}
  )

  generate_messages(
    DEPENDENCIES
      geometry_msgs
      jsk_recognition_msgs
      sensor_msgs
      std_msgs
  )

  catkin_package(
    CATKIN_DEPENDS
      geometry_msgs
      jsk_recognition_msgs
      message_runtime
      sensor_msgs
      std_msgs
  )

else() # ROS2

  find_package(rosidl_default_generators REQUIRED)
  find_package(ament_cmake REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(builtin_interfaces REQUIRED)
  find_package(jsk_recognition_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)

  rosidl_generate_interfaces(
    ${PROJECT_NAME}
      ${msgs}
      ${srvs}
    DEPENDENCIES
      std_msgs
      builtin_interfaces
      jsk_recognition_msgs
      sensor_msgs
      geometry_msgs
  )

  ament_export_dependencies(rosidl_default_runtime)
  ament_package()

endif()

