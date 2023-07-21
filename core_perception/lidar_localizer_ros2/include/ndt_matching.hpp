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

#include <ndt_cpu/NormalDistributionsTransform.h>
#ifdef CUDA_FOUND
#include <ndt_gpu/NormalDistributionsTransform.h>
#endif
#include <autoware_config_msgs/msg/config_ndt.hpp>
#include <autoware_msgs/msg/ndt_stat.hpp>

#include <carma_ros2_utils/carma_lifecycle_node.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <pcl/common/transforms.h>

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

        MethodType _method_type = MethodType::PCL_GENERIC;

        pose initial_pose, predict_pose, predict_pose_imu, predict_pose_odom, predict_pose_imu_odom, previous_pose,
            ndt_pose, current_pose, current_pose_imu, current_pose_odom, current_pose_imu_odom, localizer_pose;

        double offset_x, offset_y, offset_z, offset_yaw;  // current_pos - previous_pose
        double offset_imu_x, offset_imu_y, offset_imu_z, offset_imu_roll, offset_imu_pitch, offset_imu_yaw;
        double offset_odom_x, offset_odom_y, offset_odom_z, offset_odom_roll, offset_odom_pitch, offset_odom_yaw;
        double offset_imu_odom_x, offset_imu_odom_y, offset_imu_odom_z, offset_imu_odom_roll, offset_imu_odom_pitch,
            offset_imu_odom_yaw;

        // Can't load if typed "pcl::PointCloud<pcl::PointXYZRGB> map, add;"
        pcl::PointCloud<pcl::PointXYZ> map, add;

        // If the map is loaded, map_loaded will be 1.
        int map_loaded = 0;
        int _use_gnss = 1;
        int init_pos_set = 0;

        pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
        cpu::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> anh_ndt;
        #ifdef CUDA_FOUND
        std::shared_ptr<gpu::GNormalDistributionsTransform> anh_gpu_ndt_ptr =
            std::make_shared<gpu::GNormalDistributionsTransform>();
        #endif
        #ifdef USE_PCL_OPENMP
        pcl_omp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> omp_ndt;
        #endif

        // Default values
        int max_iter = 30;        // Maximum iterations
        float ndt_res = 1.0;      // Resolution
        double step_size = 0.1;   // Step size
        double trans_eps = 0.001;  // Transformation epsilon. In PCLv1.10 (ros noetic) this value is squared error not base epsilon 
                                        // NOTE: A value of 0.0001 can work as well. 
                                        // This will increase the required iteration count (and therefore execution time) but might increase accuracy.

        geometry_msgs::msg::PoseStamped predict_pose_msg;
        geometry_msgs::msg::PoseStamped predict_pose_imu_msg;
        geometry_msgs::msg::PoseStamped predict_pose_odom_msg;
        geometry_msgs::msg::PoseStamped predict_pose_imu_odom_msg;
        geometry_msgs::msg::PoseStamped ndt_pose_msg;

        geometry_msgs::msg::PoseStamped localizer_pose_msg;
        geometry_msgs::msg::TwistStamped estimate_twist_msg;


        double exe_time = 0.0;
        bool has_converged;
        int iteration = 0;
        double fitness_score = 0.0;
        double trans_probability = 0.0;

        // reference for comparing fitness_score, default value set to 500.0
        double _gnss_reinit_fitness = 500.0;

        double diff = 0.0;
        double diff_x = 0.0, diff_y = 0.0, diff_z = 0.0, diff_yaw;

        double current_velocity = 0.0, previous_velocity = 0.0, previous_previous_velocity = 0.0;  // [m/s]
        double current_velocity_x = 0.0, previous_velocity_x = 0.0;
        double current_velocity_y = 0.0, previous_velocity_y = 0.0;
        double current_velocity_z = 0.0, previous_velocity_z = 0.0;
        // static double current_velocity_yaw = 0.0, previous_velocity_yaw = 0.0;
        double current_velocity_smooth = 0.0;

        double current_velocity_imu_x = 0.0;
        double current_velocity_imu_y = 0.0;
        double current_velocity_imu_z = 0.0;

        double current_accel = 0.0, previous_accel = 0.0;  // [m/s^2]
        double current_accel_x = 0.0;
        double current_accel_y = 0.0;
        double current_accel_z = 0.0;
        // static double current_accel_yaw = 0.0;

        double angular_velocity = 0.0;

        int use_predict_pose = 0;      

        std_msgs::msg::Float32 estimated_vel_mps, estimated_vel_kmph, previous_estimated_vel_kmph;

        std::chrono::time_point<std::chrono::system_clock> matching_start, matching_end;          

        std_msgs::msg::Float32 time_ndt_matching;      

        int _queue_size = 1;

        autoware_msgs::msg::NDTStat ndt_stat_msg; 

        double predict_pose_error = 0.0;

        double _tf_x, _tf_y, _tf_z, _tf_roll, _tf_pitch, _tf_yaw;
        Eigen::Matrix4f tf_btol;

        std::string _localizer = "velodyne";
        std::string _offset = "linear";  // linear, zero, quadratic

        std_msgs::msg::Float32 ndt_reliability; 

        bool _get_height = false;
        bool _use_local_transform = false;
        bool _use_imu = false;
        bool _use_odom = false;
        bool _imu_upside_down = false;
        bool _output_log_data = false;

        std::string _imu_topic = "/imu_raw";

        std::ofstream ofs;
        std::string filename;  

        sensor_msgs::msg::Imu imu;
        nav_msgs::msg::Odometry odom;   

        // tf::TransformListener local_transform_listener;
        tf2::Stamped<tf2::Transform> local_transform;

        std::string _base_frame = "base_link";
        std::string _map_frame = "map";

        unsigned int points_map_num = 0;   

        pthread_mutex_t mutex;

    public:
        explicit NDTMatching(const rclcpp::NodeOptions &);
        carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State &prev_state);
        
};
}

