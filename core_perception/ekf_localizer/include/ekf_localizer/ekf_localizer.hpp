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

#include <iostream>
#include <vector>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <boost/array.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include "amathutils_lib_ros2/kalman_filter.hpp"
#include "amathutils_lib_ros2/time_delay_kalman_filter.hpp"
#include <carma_ros2_utils/carma_lifecycle_node.hpp>

namespace ekf_localizer{
    
class EKFLocalizer: public carma_ros2_utils::CarmaLifecycleNode
{
    private:

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;                           //!< @brief ekf estimated pose publisher
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_cov_;         //!< @brief estimated ekf pose with covariance publisher
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_twist_;                         //!< @brief ekf estimated twist publisher
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr pub_twist_cov_;       //!< @brief ekf estimated twist with covariance publisher
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_debug_;                         //!< @brief debug info publisher
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_measured_pose_;                  //!< @brief debug measurement pose publisher
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_yaw_bias_;                                //!< @brief ekf estimated yaw bias publisher
    
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_initialpose_;     //!< @brief initial pose subscriber
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_pose_with_cov_;   //!< @brief measurement pose with covariance subscriber
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;                          //!< @brief measurement pose subscriber
    rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr sub_twist_with_cov_; //!< @brief measurement twist with covariance subscriber
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_twist_;                        //!< @brief measurement twist subscriber

    rclcpp::TimerBase::SharedPtr timer_control_;             //!< @brief time for ekf calculation callback

    TimeDelayKalmanFilter ekf_;  //!< @brief  extended kalman filter instance.

    /* parameters */
    double proc_stddev_yaw_c_ = 0.005, proc_stddev_yaw_bias_c_ = 0.001, proc_stddev_vx_c_ = 2.0, proc_stddev_wz_c_ = 0.2;

    bool show_debug_info_ = false;
    double ekf_rate_ = 50.0;                  //!< @brief  EKF predict rate
    double ekf_dt_;                    //!< @brief  = 1 / ekf_rate_
    bool enable_yaw_bias_estimation_ = true;  //!< @brief  for LiDAR mount error. if true, publish /estimate_yaw_bias
    std::string pose_frame_id_ = "/map";        //!< @brief  Parent frame for pose and tf output
    std::string child_frame_id_ = "base_link";       //!< @brief  Child frame for pose and tf output

    int dim_x_;              //!< @brief  dimension of EKF state
    int extend_state_step_ = 50;  //!< @brief  for time delay compensation
    int dim_x_ex_;           //!< @brief  dimension of extended EKF state (dim_x_ * extended_state_step)

    /* Pose */
    double pose_additional_delay_ = 0.0;          //!< @brief  compensated pose delay time = (pose.header.stamp - now) +
                                            //!< additional_delay [s]
    double pose_measure_uncertainty_time_ = 0.01;  //!< @brief  added for measurement covariance
    double pose_rate_ = 10.0;                      //!< @brief  pose rate [s], used for covariance calculation
    double pose_gate_dist_ = 10000.0;   //!< @brief  pose measurement is ignored if the maharanobis distance is larger than this
                                //!< value.
    double pose_stddev_x_ = 0.05;    //!< @brief  standard deviation for pose position x [m]
    double pose_stddev_y_ = 0.05;    //!< @brief  standard deviation for pose position y [m]
    double pose_stddev_yaw_ = 0.035;  //!< @brief  standard deviation for pose position yaw [rad]
    bool use_pose_with_covariance_ = false;  //!< @brief  use covariance in pose_with_covariance message
    bool use_twist_with_covariance_ = false; //!< @brief  use covariance in twist_with_covariance message

    /* twist */
    double twist_additional_delay_ = 0.0;  //!< @brief  compensated delay time = (twist.header.stamp - now) + additional_delay
                                    //!< [s]
    double twist_rate_ = 10.0;              //!< @brief  rate [s], used for covariance calculation
    double twist_gate_dist_ = 10000.0;  //!< @brief  measurement is ignored if the maharanobis distance is larger than this value.
    double twist_stddev_vx_ = 0.2;  //!< @brief  standard deviation for linear vx
    double twist_stddev_wz_ = 0.03;  //!< @brief  standard deviation for angular wx

    /* process noise variance for discrete model */
    double proc_cov_yaw_d_;       //!< @brief  discrete yaw process noise
    double proc_cov_yaw_bias_d_;  //!< @brief  discrete yaw bias process noise
    double proc_cov_vx_d_;        //!< @brief  discrete process noise in d_vx=0
    double proc_cov_wz_d_;        //!< @brief  discrete process noise in d_wz=0

    enum IDX
    {
        X = 0,
        Y = 1,
        YAW = 2,
        YAWB = 3,
        VX = 4,
        WZ = 5,
    };

      /* for model prediction */
    std::shared_ptr<geometry_msgs::msg::TwistStamped> current_twist_ptr_;  //!< @brief current measured twist
    std::shared_ptr<geometry_msgs::msg::PoseStamped> current_pose_ptr_;    //!< @brief current measured pose
    geometry_msgs::msg::PoseStamped current_ekf_pose_;                     //!< @brief current estimated pose
    geometry_msgs::msg::TwistStamped current_ekf_twist_;                   //!< @brief current estimated twist
    
    //ros1 definition for current_pose_covariance_ and current_twist_covariance_ was as boost::array.
    //Since the difference between the two are minimal. It is changed here to std::array inorder to prevent type cast errors
    std::array<double, 36ul> current_pose_covariance_;
    std::array<double, 36ul> current_twist_covariance_;

    rclcpp::Clock clk_; //!< @brief the node clock

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_br_shared_ptr_ = nullptr;


    /**
     * @brief computes update & prediction of EKF for each ekf_dt_[s] time
     */
    void timerCallback();

    /**
     * @brief publish tf
     */
    void broadcastTF();

    /**
     * @brief set pose measurement
     */
    void callbackPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

      /**
     * @brief set twist measurement
     */
    void callbackTwist(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

    /**
     * @brief set poseWithCovariance measurement
     */
    void callbackPoseWithCovariance(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);


    /**
     * @brief set twistWithCovariance measurement
     */
    void callbackTwistWithCovariance(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg);

    /**
     * @brief set initial_pose to current EKF pose
     */
    void callbackInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

    /**
     * @brief initialization of EKF
     */
    void initEKF();

    /**
     * @brief compute EKF prediction
     */
    void predictKinematicsModel();

    /**
     * @brief compute EKF update with pose measurement
     * @param pose measurement value
     */
    void measurementUpdatePose(const geometry_msgs::msg::PoseStamped& pose);

    /**
     * @brief compute EKF update with pose measurement
     * @param twist measurement value
     */
    void measurementUpdateTwist(const geometry_msgs::msg::TwistStamped& twist);


    /**
     * @brief check whether a measurement value falls within the mahalanobis distance threshold
     * @param dist_max mahalanobis distance threshold
     * @param estimated current estimated state
     * @param measured measured state
     * @param estimated_cov current estimation covariance
     * @return whether it falls within the mahalanobis distance threshold
     */
    bool mahalanobisGate(const double& dist_max, const Eigen::MatrixXd& estimated, const Eigen::MatrixXd& measured,
                        const Eigen::MatrixXd& estimated_cov);


    /**
     * @brief get transform from frame_id
     */
    bool getTransformFromTF(std::string parent_frame, std::string child_frame,
                            geometry_msgs::msg::TransformStamped& transform);


    /**
     * @brief normalize yaw angle
     * @param yaw yaw angle
     * @return normalized yaw
     */
    double normalizeYaw(const double& yaw);

    /**
     * @brief set current EKF estimation result to current_ekf_pose_ & current_ekf_twist_
     */
    void setCurrentResult();           

    /**
     * @brief publish current EKF estimation result
     */
    void publishEstimateResult();  

    /**
     * @brief for debug
     */
    void showCurrentX();       

    
    public:
    
    explicit EKFLocalizer(const rclcpp::NodeOptions &);
    
    carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State &);
};

} //namespace ekf_localizer