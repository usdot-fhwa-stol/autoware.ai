cmake_minimum_required(VERSION 3.5)
project(autoware_map_msgs)

find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
set(CMAKE_CXX_STANDARD 14)

rosidl_generate_interfaces(
    ${PROJECT_NAME}
        msg/LaneRelationArray.msg
        msg/SignalArray.msg
        msg/OppositeLaneRelationArray.msg
        msg/WaypointRelationArray.msg
        msg/WaypointLaneRelation.msg
        msg/LaneArray.msg
        msg/PointArray.msg
        msg/SignalLightArray.msg
        msg/SignalLight.msg
        msg/WaypointSignalRelation.msg
        msg/Waypoint.msg
        msg/Point.msg
        msg/WayareaArray.msg
        msg/Area.msg
        msg/LaneSignalLightRelation.msg
        msg/LaneAttributeRelation.msg
        msg/WaypointRelation.msg
        msg/Lane.msg
        msg/AreaArray.msg
        msg/LaneChangeRelationArray.msg
        msg/LaneRelation.msg
        msg/Wayarea.msg
        msg/LaneAttributeRelationArray.msg
        msg/WaypointArray.msg
        msg/WaypointSignalRelationArray.msg
        msg/Signal.msg
        msg/LaneChangeRelation.msg
        msg/WaypointLaneRelationArray.msg
        msg/LaneSignalLightRelationArray.msg
        msg/OppositeLaneRelation.msg
    DEPENDENCIES
        std_msgs
)
ament_package()
