cmake_minimum_required(VERSION 3.5)
project(vector_map_msgs)

find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
set(CMAKE_CXX_STANDARD 1 4)

rosidl_generate_interfaces(
    ${PROJECT_NAME}
        msg/SignalArray.msg
        msg/WhiteLineArray.msg
        msg/RoadEdgeArray.msg
        msg/Curb.msg
        msg/LaneArray.msg
        msg/PointArray.msg
        msg/DriveOnPortion.msg
        msg/GuardRailArray.msg
        msg/SideStripArray.msg
        msg/RoadSign.msg
        msg/CurveMirrorArray.msg
        msg/Point.msg
        msg/NodeArray.msg
        msg/DTLane.msg
        msg/Area.msg
        msg/WayAreaArray.msg
        msg/RailCrossingArray.msg
        msg/WallArray.msg
        msg/RoadMark.msg
        msg/Gutter.msg
        msg/StopLine.msg
        msg/Lane.msg
        msg/CrossRoadArray.msg
        msg/RailCrossing.msg
        msg/SideStrip.msg
        msg/AreaArray.msg
        msg/Box.msg
        msg/RoadPoleArray.msg
        msg/SideWalkArray.msg
        msg/CrossWalk.msg
        msg/RoadSignArray.msg
        msg/Pole.msg
        msg/GuardRail.msg
        msg/CurbArray.msg
        msg/SideWalk.msg
        msg/Line.msg
        msg/DTLaneArray.msg
        msg/WayArea.msg
        msg/Signal.msg
        msg/CurveMirror.msg
        msg/Node.msg
        msg/PoleArray.msg
        msg/ZebraZone.msg
        msg/WhiteLine.msg
        msg/ZebraZoneArray.msg
        msg/CrossRoad.msg
        msg/StopLineArray.msg
        msg/UtilityPoleArray.msg
        msg/Fence.msg
        msg/StreetLight.msg
        msg/LineArray.msg
        msg/StreetLightArray.msg
        msg/UtilityPole.msg
        msg/GutterArray.msg
        msg/RoadPole.msg
        msg/Vector.msg
        msg/DriveOnPortionArray.msg
        msg/FenceArray.msg
        msg/BoxArray.msg
        msg/VectorArray.msg
        msg/Wall.msg
        msg/RoadMarkArray.msg
        msg/RoadEdge.msg
        msg/CrossWalkArray.msg
    DEPENDENCIES
)

ament_package()
