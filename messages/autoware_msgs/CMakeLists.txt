cmake_minimum_required(VERSION 3.5)
project(autoware_msgs)

find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
# find_package(jsk_recognition_msgs REQUIRED) does not exist in ros2
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
set(CMAKE_CXX_STANDARD 14)

rosidl_generate_interfaces(
    ${PROJECT_NAME}
        srv/RecognizeLightState.srv
        msg/DetectedObject.msg
        msg/ColorSet.msg
        msg/ICPStat.msg
        msg/TunedResult.msg
        msg/VehicleLocation.msg
        msg/LaneArray.msg
        msg/ValueSet.msg
        msg/GeometricRectangle.msg
        msg/ObjPose.msg
        msg/Waypoint.msg
        msg/IndicatorCmd.msg
        msg/VehicleCmd.msg
        msg/DTLane.msg
        msg/NDTStat.msg
        msg/ImageRect.msg
        msg/SteerCmd.msg
        msg/VehicleStatus.msg
        msg/StateCmd.msg
        msg/SyncTimeDiff.msg
        msg/AdjustXY.msg
        msg/ScanImage.msg
        msg/PointsImage.msg
        msg/Gear.msg
        msg/ImageLaneObjects.msg
        msg/ImageObjTracked.msg
        msg/ImageObjects.msg
        msg/ControlCommand.msg
        msg/Lane.msg
        # msg/CloudClusterArray.msg depends on jsk_recognition_msgs
        msg/ImageObjRanged.msg
        msg/AccelCmd.msg
        msg/ObjLabel.msg
        msg/SyncTimeMonitor.msg
        msg/ControlCommandStamped.msg
        msg/DetectedObjectArray.msg
        msg/LampCmd.msg
        msg/Signals.msg
        # msg/CloudCluster.msg depends on jsk_recognition_msgs
        msg/ExtractedPosition.msg
        msg/VscanTrackedArray.msg
        msg/TrafficLightResult.msg
        msg/TrafficLightResultArray.msg
        msg/ImageObj.msg
        msg/RemoteCmd.msg
        msg/BrakeCmd.msg
        msg/TrafficLight.msg
        msg/ImageRectRanged.msg
        msg/VscanTracked.msg
        msg/State.msg
        msg/CameraExtrinsic.msg
        msg/ProjectionMatrix.msg
        msg/Centroids.msg
        msg/WaypointState.msg
    DEPENDENCIES
        sensor_msgs
        geometry_msgs
        # jsk_recognition_msgs does not exist in ros2
        std_msgs
)

ament_package()
