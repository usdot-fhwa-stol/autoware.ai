cmake_minimum_required(VERSION 3.5)
project(autoware_config_msgs)

find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
set(CMAKE_CXX_STANDARD 14)

rosidl_generate_interfaces(
    ${PROJECT_NAME}
        msg/ConfigNDT.msg
        msg/ConfigPlannerSelector.msg
        msg/ConfigTwistFilter.msg
        msg/ConfigVelocitySet.msg
        msg/ConfigPoints2Polygon.msg
        msg/ConfigApproximateNDTMapping.msg
        msg/ConfigLaneRule.msg
        msg/ConfigRingFilter.msg
        msg/ConfigCarKF.msg
        msg/ConfigDistanceFilter.msg
        msg/ConfigICP.msg
        msg/ConfigWaypointReplanner.msg
        msg/ConfigRingGroundFilter.msg
        msg/ConfigWaypointFollower.msg
        msg/ConfigCompareMapFilter.msg
        msg/ConfigDecisionMaker.msg
        msg/ConfigLaneStop.msg
        msg/ConfigVoxelGridFilter.msg
        msg/ConfigPedestrianKF.msg
        msg/ConfigLaneSelect.msg
        msg/ConfigPedestrianFusion.msg
        msg/ConfigRayGroundFilter.msg
        msg/ConfigPedestrianDPM.msg
        msg/ConfigCarDPM.msg
        msg/ConfigNDTMappingOutput.msg
        msg/ConfigRcnn.msg
        msg/ConfigNDTMapping.msg
        msg/ConfigSSD.msg
        msg/ConfigRandomFilter.msg
        msg/ConfigCarFusion.msg
        msg/ConfigLatticeVelocitySet.msg
    DEPENDENCIES
        std_msgs
)

ament_package()
