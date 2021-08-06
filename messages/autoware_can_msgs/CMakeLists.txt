cmake_minimum_required(VERSION 3.5)
project(autoware_can_msgs)

find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
set(CMAKE_CXX_STANDARD 14)

rosidl_generate_interfaces(
    ${PROJECT_NAME}
        msg/CANData.msg
        msg/CANPacket.msg
        msg/CANInfo.msg
    DEPENDENCIES
        std_msgs
)

ament_package()
