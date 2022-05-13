cmake_minimum_required(VERSION 3.5)
project(autoware_build_flags)


find_package(ros_environment REQUIRED)
set(ROS_VERSION $ENV{ROS_VERSION})

if(${ROS_VERSION} EQUAL 1)
  
  find_package(catkin REQUIRED)  
  catkin_package(CFG_EXTRAS autoware_build_flags-extras.cmake)

else() #ROS2
  
  find_package(ament_cmake_auto REQUIRED)
  ament_auto_find_build_dependencies()
  ament_auto_package()

endif()
