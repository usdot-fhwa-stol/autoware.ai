cmake_minimum_required(VERSION 3.5)
project(autoware_system_msgs)

find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(std_msgs REQUIRED)
set(CMAKE_CXX_STANDARD 1 4)

rosidl_generate_interfaces(
    ${PROJECT_NAME}
        msg/SystemStatus.msg
        msg/DiagnosticStatus.msg
        msg/DiagnosticStatusArray.msg
        msg/HardwareStatus.msg
        msg/NodeStatus.msg
    DEPENDENCIES
        autoware_system_msgs
        rosgraph_msgs
        std_msgs
)

ament_package()

add_compile_options(-std=c++11)
