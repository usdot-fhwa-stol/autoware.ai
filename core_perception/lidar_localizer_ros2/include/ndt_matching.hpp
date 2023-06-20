/*
 * Copyright (C) 2023 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */
#include <pthread.h>
#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#include <boost/filesystem.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <velodyne_pointcloud/rawdata.hpp>

#include <tf2_ros/static_transform_broadcaster.h>
#include "tf2_ros/transform_broadcaster.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/ndt.h>

#include <ndt_cpu_ros2/NormalDistributionsTransform.h>
#ifdef CUDA_FOUND
#include <ndt_gpu_ros2/NormalDistributionsTransform.h>
#endif
#include <autoware_config_msgs/msg/config_ndt.hpp>
#include <autoware_msgs/msg/ndt_stat.hpp>

#include <carma_ros2_utils/carma_lifecycle_node.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#define PREDICT_POSE_THRESHOLD 0.5

#define Wa 0.4
#define Wb 0.3
#define Wc 0.3

namespace ndt_matching{
struct pose
{
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
};

enum class MethodType
{
    PCL_GENERIC = 0,
    PCL_ANH = 1,
    PCL_ANH_GPU = 2,
    PCL_OPENMP = 3,
};

class NDTMatching : public carma_ros2_utils::CarmaLifecycleNode {

    private:
        
        // Define publishers                                        
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr predict_pose_pub;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr predict_pose_imu_pub;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr predict_pose_odom_pub;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr predict_pose_imu_odom_pub;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ndt_pose_pub;

        // current_pose is published by vel_pose_mux
        // static ros::Publisher current_pose_pub;
        // static geometry_msgs::PoseStamped current_pose_msg;
        

        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr localizer_pose_pub;
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr estimate_twist_pub;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr estimated_vel_mps_pub, estimated_vel_kmph_pub;
        rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr estimated_vel_pub;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr time_ndt_matching_pub;
        rclcpp::Publisher<autoware_msgs::msg::NDTStat>::SharedPtr ndt_stat_pub;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr ndt_reliability_pub;

        // Define subscribers
        rclcpp::Subscription<autoware_config_msgs::msg::ConfigNDT>::SharedPtr param_sub;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr gnss_sub;
        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_sub;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub;

        int method_type_tmp = 0;

        void param_callback(const autoware_config_msgs::msg::ConfigNDT::SharedPtr input);
        void map_callback(const sensor_msgs::msg::PointCloud2::SharedPtr input);
        void gnss_callback(const geometry_msgs::msg::PoseStamped::SharedPtr input);
        void initialpose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr input);
        void points_callback(const sensor_msgs::msg::PointCloud2::SharedPtr input);
        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr input);

        void imu_odom_calc(rclcpp::Time current_time);
        void odom_calc(rclcpp::Time current_time);
        void imu_calc(rclcpp::Time current_time);

        double wrapToPm(double a_num, const double a_max);
        double wrapToPmPi(const double a_angle_rad);
        double calcDiffForRadian(const double lhs_rad, const double rhs_rad);

        void imuUpsideDown(const sensor_msgs::msg::Imu::SharedPtr input);
        void imu_callback(const sensor_msgs::msg::Imu::SharedPtr input);
        pose convertPoseIntoRelativeCoordinate(const pose &target_pose, const pose &reference_pose);
        

        // Note: the function below and its definitions were taken from pcl_ros package, to support local use while the package is being ported to ros2
        // https://github.com/ros-perception/perception_pcl.git
        template<typename PointT>
        void transformPointCloud(const pcl::PointCloud<PointT> & cloud_in, pcl::PointCloud<PointT> & cloud_out, const tf2::Transform & transform);

    public:
        explicit NDTMatching(const rclcpp::NodeOptions &);
        carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State &prev_state);
        
};
}

