cmake_minimum_required(VERSION 3.5)
project(tablet_socket_msgs)

find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
set(CMAKE_CXX_STANDARD 1 4)

rosidl_generate_interfaces(
    ${PROJECT_NAME}
        msg/Waypoint.msg
        msg/error_info.msg
        msg/gear_cmd.msg
        msg/mode_info.msg
        msg/route_cmd.msg
        msg/mode_cmd.msg
    DEPENDENCIES
)

ament_package()
