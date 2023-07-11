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

#include "ekf_localizer/ekf_localizer.hpp"

// clang-format off
#define PRINT_MAT(X) std::cout << #X << ":\n" << X << std::endl << std::endl
#define DEBUG_INFO(...) { if (show_debug_info_) { RCLCPP_INFO(rclcpp::get_logger("ekf_localizer"), __VA_ARGS__); } }
#define DEBUG_PRINT_MAT(X) { if (show_debug_info_) { std::cout << #X << ": " << X << std::endl; } }


namespace ekf_localizer{

    EKFLocalizer::EKFLocalizer(const rclcpp::NodeOptions &options) : carma_ros2_utils::CarmaLifecycleNode(options), dim_x_(6 /* x, y, yaw, yaw_bias, vx, wz */)
    {
        show_debug_info_ = declare_parameter<bool>("show_debug_info", show_debug_info_);
        ekf_rate_ = declare_parameter<double>("predict_frequency", ekf_rate_);
        enable_yaw_bias_estimation_ = declare_parameter<bool>("enable_yaw_bias_estimation", enable_yaw_bias_estimation_);
        extend_state_step_ = declare_parameter<int>("extend_state_step",extend_state_step_);
        pose_frame_id_ = declare_parameter<std::string>("pose_frame_id", pose_frame_id_);
        child_frame_id_ = declare_parameter<std::string>("child_frame_id", child_frame_id_);

        /* pose measurement */
        pose_additional_delay_ = declare_parameter<double>("pose_additional_delay", pose_additional_delay_);
        pose_measure_uncertainty_time_ = declare_parameter<double>("pose_measure_uncertainty_time", pose_measure_uncertainty_time_);
        pose_rate_ = declare_parameter<double>("pose_rate", pose_rate_);
        pose_gate_dist_ = declare_parameter<double>("pose_gate_dist", pose_gate_dist_);
        pose_stddev_x_ = declare_parameter<double>("pose_stddev_x", pose_stddev_x_);
        pose_stddev_y_ = declare_parameter<double>("pose_stddev_y", pose_stddev_y_);
        pose_stddev_yaw_ = declare_parameter<double>("pose_stddev_yaw", pose_stddev_yaw_);
        use_pose_with_covariance_ = declare_parameter<bool>("use_pose_with_covariance", use_pose_with_covariance_);

        /* twist measurement */
        twist_additional_delay_ = declare_parameter<double>("twist_additional_delay", twist_additional_delay_);
        twist_rate_ = declare_parameter<double>("twist_rate", twist_rate_);
        twist_gate_dist_ = declare_parameter<double>("twist_gate_dist", twist_gate_dist_);
        twist_stddev_vx_ = declare_parameter<double>("twist_stddev_vx", twist_stddev_vx_);
        twist_stddev_wz_ = declare_parameter<double>("twist_stddev_wz",twist_stddev_wz_);
        use_twist_with_covariance_ = declare_parameter<bool>("use_twist_with_covariance", use_twist_with_covariance_);


        /* process noise */
        proc_stddev_yaw_c_ = declare_parameter<double>("proc_stddev_yaw_c", proc_stddev_yaw_c_);
        proc_stddev_yaw_bias_c_ = declare_parameter<double>("proc_stddev_yaw_bias_c", proc_stddev_yaw_bias_c_);
        proc_stddev_vx_c_ = declare_parameter<double>("proc_stddev_vx_c", proc_stddev_vx_c_);
        proc_stddev_wz_c_ = declare_parameter<double>("proc_stddev_wz_c", proc_stddev_wz_c_);
        
    }

    carma_ros2_utils::CallbackReturn EKFLocalizer::handle_on_configure(const rclcpp_lifecycle::State &prev_state)
    {
        get_parameter<bool>("show_debug_info", show_debug_info_);
        get_parameter<double>("predict_frequency", ekf_rate_);
        ekf_dt_ = 1.0 / std::max(ekf_rate_, 0.1);
        get_parameter<bool>("enable_yaw_bias_estimation", enable_yaw_bias_estimation_);
        get_parameter<int>("extend_state_step",extend_state_step_);
        get_parameter<std::string>("pose_frame_id", pose_frame_id_);
        get_parameter<std::string>("child_frame_id", child_frame_id_);

        /* pose measurement */
        get_parameter<double>("pose_additional_delay", pose_additional_delay_);
        get_parameter<double>("pose_measure_uncertainty_time", pose_measure_uncertainty_time_);
        get_parameter<double>("pose_rate", pose_rate_);
        get_parameter<double>("pose_gate_dist", pose_gate_dist_);
        get_parameter<double>("pose_stddev_x", pose_stddev_x_);
        get_parameter<double>("pose_stddev_y", pose_stddev_y_);
        get_parameter<double>("pose_stddev_yaw", pose_stddev_yaw_);
        get_parameter<bool>("use_pose_with_covariance", use_pose_with_covariance_);

        /* twist measurement */
        get_parameter<double>("twist_additional_delay", twist_additional_delay_);
        get_parameter<double>("twist_rate", twist_rate_);
        get_parameter<double>("twist_gate_dist", twist_gate_dist_);
        get_parameter<double>("twist_stddev_vx", twist_stddev_vx_);
        get_parameter<double>("twist_stddev_wz",twist_stddev_wz_);
        get_parameter<bool>("use_twist_with_covariance", use_twist_with_covariance_);


        /* process noise */
        get_parameter<double>("proc_stddev_yaw_c", proc_stddev_yaw_c_);
        get_parameter<double>("proc_stddev_yaw_bias_c", proc_stddev_yaw_bias_c_);
        get_parameter<double>("proc_stddev_vx_c", proc_stddev_vx_c_);
        get_parameter<double>("proc_stddev_wz_c", proc_stddev_wz_c_);


        if (!enable_yaw_bias_estimation_)
        {
            proc_stddev_yaw_bias_c_ = 0.0;
        }

        /* convert to continuous to discrete */
        proc_cov_vx_d_ = std::pow(proc_stddev_vx_c_, 2.0) * ekf_dt_;
        proc_cov_wz_d_ = std::pow(proc_stddev_wz_c_, 2.0) * ekf_dt_;
        proc_cov_yaw_d_ = std::pow(proc_stddev_yaw_c_, 2.0) * ekf_dt_;
        proc_cov_yaw_bias_d_ = std::pow(proc_stddev_yaw_bias_c_, 2.0) * ekf_dt_;

        /* initialize ros system */
        timer_control_ = create_wall_timer(std::chrono::milliseconds(int(ekf_dt_*1000)), std::bind(&EKFLocalizer::timerCallback, this));
        pub_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>("ekf_pose", 1);
        pub_pose_cov_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("ekf_pose_with_covariance", 1);
        pub_twist_ = create_publisher<geometry_msgs::msg::TwistStamped>("ekf_twist", 1);
        pub_twist_cov_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("ekf_twist_with_covariance", 1);
        pub_yaw_bias_ = create_publisher<std_msgs::msg::Float64>("estimated_yaw_bias", 1);
        sub_initialpose_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 1, std::bind(&EKFLocalizer::callbackInitialPose, this, std::placeholders::_1));
        sub_pose_with_cov_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("in_pose_with_covariance", 1, std::bind(&EKFLocalizer::callbackPoseWithCovariance, this, std::placeholders::_1)); 
        sub_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>("in_pose", 1, std::bind(&EKFLocalizer::callbackPose, this, std::placeholders::_1));
        sub_twist_with_cov_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>("in_twist_with_covariance", 1, std::bind(&EKFLocalizer::callbackTwistWithCovariance, this, std::placeholders::_1));
        sub_twist_ = create_subscription<geometry_msgs::msg::TwistStamped>("in_twist", 1, std::bind(&EKFLocalizer::callbackTwist, this, std::placeholders::_1));
        
        dim_x_ex_ = dim_x_ * extend_state_step_;
        initEKF();


        pub_debug_ = create_publisher<std_msgs::msg::Float64MultiArray>("debug", 1);
        pub_measured_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>("debug/measured_pose", 1);

        clk_ = *this->get_clock();
        return CallbackReturn::SUCCESS;
    }

    void EKFLocalizer::timerCallback()
    {
        DEBUG_INFO("========================= timer called =========================");
        /* predict model in EKF */
        auto start = std::chrono::system_clock::now();
        DEBUG_INFO("------------------------- start prediction -------------------------");
        predictKinematicsModel();
        double elapsed =
            std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now() - start).count();
        DEBUG_INFO("[EKF] predictKinematicsModel calculation time = %f [ms]", elapsed * 1.0e-6);
        DEBUG_INFO("------------------------- end prediction -------------------------\n");
        /* pose measurement update */
        if (current_pose_ptr_ != nullptr)
        {
            DEBUG_INFO("------------------------- start Pose -------------------------");
            start = std::chrono::system_clock::now();
            measurementUpdatePose(*current_pose_ptr_);
            elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now() - start).count();
            DEBUG_INFO("[EKF] measurementUpdatePose calculation time = %f [ms]", elapsed * 1.0e-6);
            DEBUG_INFO("------------------------- end Pose -------------------------\n");
        }

        /* twist measurement update */
        if (current_twist_ptr_ != nullptr)
        {
            DEBUG_INFO("------------------------- start twist -------------------------");
            start = std::chrono::system_clock::now();
            measurementUpdateTwist(*current_twist_ptr_);
            elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now() - start).count();
            DEBUG_INFO("[EKF] measurementUpdateTwist calculation time = %f [ms]", elapsed * 1.0e-6);
            DEBUG_INFO("------------------------- end twist -------------------------\n");
        }

        /* set current pose, twist */
        setCurrentResult();

        /* publish ekf result */
        publishEstimateResult();
        broadcastTF();
    }

    void EKFLocalizer::showCurrentX()
    {
        if (show_debug_info_)
        {
            Eigen::MatrixXd X(dim_x_, 1);
            ekf_.getLatestX(X);
            DEBUG_PRINT_MAT(X.transpose());
        }
    }

    /*
    * setCurrentResult
    */
    void EKFLocalizer::setCurrentResult()
    {
    current_ekf_pose_.header.frame_id = pose_frame_id_;
    current_ekf_pose_.header.stamp = this->now();
    current_ekf_pose_.pose.position.x = ekf_.getXelement(IDX::X);
    current_ekf_pose_.pose.position.y = ekf_.getXelement(IDX::Y);

    tf2::Quaternion q_tf;
    double roll, pitch, yaw;
    if (current_pose_ptr_ != nullptr)
    {
        current_ekf_pose_.pose.position.z = current_pose_ptr_->pose.position.z;
        tf2::fromMsg(current_pose_ptr_->pose.orientation, q_tf); /* use Pose pitch and roll */
        tf2::Matrix3x3(q_tf).getRPY(roll, pitch, yaw);
    }
    else
    {
        current_ekf_pose_.pose.position.z = 0.0;
        roll = 0;
        pitch = 0;
    }
    yaw = ekf_.getXelement(IDX::YAW) + ekf_.getXelement(IDX::YAWB);
    q_tf.setRPY(roll, pitch, yaw);
    tf2::convert(q_tf, current_ekf_pose_.pose.orientation);

    current_ekf_twist_.header.frame_id = child_frame_id_;
    current_ekf_twist_.header.stamp = current_ekf_pose_.header.stamp; // Twist time stamp should exactly match pose timestamp since they were computed in the same EKF step
    current_ekf_twist_.twist.linear.x = ekf_.getXelement(IDX::VX);
    current_ekf_twist_.twist.angular.z = ekf_.getXelement(IDX::WZ);
    }


    /*
    * broadcastTF
    */
    void EKFLocalizer::broadcastTF()
    {
        tf2_ros::TransformBroadcaster tf_br_(shared_from_this());
        if (current_ekf_pose_.header.frame_id == "")
            return;

        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp = current_ekf_pose_.header.stamp; // Transform stamp should exactly match the same of the data it is set from
        transformStamped.header.frame_id = current_ekf_pose_.header.frame_id;
        transformStamped.child_frame_id = child_frame_id_;
        transformStamped.transform.translation.x = current_ekf_pose_.pose.position.x;
        transformStamped.transform.translation.y = current_ekf_pose_.pose.position.y;
        transformStamped.transform.translation.z = current_ekf_pose_.pose.position.z;

        transformStamped.transform.rotation.x = current_ekf_pose_.pose.orientation.x;
        transformStamped.transform.rotation.y = current_ekf_pose_.pose.orientation.y;
        transformStamped.transform.rotation.z = current_ekf_pose_.pose.orientation.z;
        transformStamped.transform.rotation.w = current_ekf_pose_.pose.orientation.w;

        tf_br_.sendTransform(transformStamped);
    }

    /*
    * getTransformFromTF
    */
    bool EKFLocalizer::getTransformFromTF(std::string parent_frame, std::string child_frame,
                                        geometry_msgs::msg::TransformStamped& transform)
    {
        tf2_ros::Buffer tf_buffer(get_clock());
        tf2_ros::TransformListener tf_listener(tf_buffer);

        rclcpp::sleep_for(std::chrono::milliseconds(100));
        if (parent_frame.front() == '/')
            parent_frame.erase(0, 1);
        if (child_frame.front() == '/')
            child_frame.erase(0, 1);

        for (int i = 0; i < 50; ++i)
        {
            try
            {
                transform = tf_buffer.lookupTransform(parent_frame, child_frame, rclcpp::Time(0,0));
                return true;
            }
            catch (tf2::TransformException& ex)
            {

                RCLCPP_WARN(get_logger(), ex.what());
                rclcpp::sleep_for(std::chrono::milliseconds(100));
            }
        }
        return false;
    }

    /*
    * callbackInitialPose
    */
    void EKFLocalizer::callbackInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr initialpose)
    {
        geometry_msgs::msg::TransformStamped transform;
        if (!getTransformFromTF(pose_frame_id_, initialpose->header.frame_id, transform))
        {
            RCLCPP_ERROR(get_logger(), "[EKF] TF transform failed. parent = %s, child = %s", pose_frame_id_.c_str(),
                    initialpose->header.frame_id.c_str());
        };

        Eigen::MatrixXd X(dim_x_, 1);
        Eigen::MatrixXd P = Eigen::MatrixXd::Zero(dim_x_, dim_x_);

        X(IDX::X) = initialpose->pose.pose.position.x + transform.transform.translation.x;
        X(IDX::Y) = initialpose->pose.pose.position.y + transform.transform.translation.y;
        X(IDX::YAW) = tf2::getYaw(initialpose->pose.pose.orientation) + tf2::getYaw(transform.transform.rotation);
        X(IDX::YAWB) = 0.0;
        X(IDX::VX) = 0.0;
        X(IDX::WZ) = 0.0;

        P(IDX::X, IDX::X) = initialpose->pose.covariance[0];
        P(IDX::Y, IDX::Y) = initialpose->pose.covariance[6 + 1];
        P(IDX::YAW, IDX::YAW) = initialpose->pose.covariance[6 * 5 + 5];
        P(IDX::YAWB, IDX::YAWB) = 0.0001;
        P(IDX::VX, IDX::VX) = 0.01;
        P(IDX::WZ, IDX::WZ) = 0.01;

        ekf_.init(X, P, extend_state_step_);
    }

    /*
    * callbackPose
    */
    void EKFLocalizer::callbackPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (!use_pose_with_covariance_)
        {
            current_pose_ptr_ = msg;
        }
    }

    /*
    * callbackPoseWithCovariance
    */
    void EKFLocalizer::callbackPoseWithCovariance(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        if (use_pose_with_covariance_)
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = msg->header;
            pose.pose = msg->pose.pose;
            current_pose_ptr_ = std::make_shared<geometry_msgs::msg::PoseStamped>(pose);
            current_pose_covariance_ = msg->pose.covariance;
        }
    
    }

    /*
    * callbackTwist
    */
    void EKFLocalizer::callbackTwist(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
        if (!use_twist_with_covariance_)
        {
            current_twist_ptr_ = std::make_shared<geometry_msgs::msg::TwistStamped>(*msg);
        }
    }

    /*
    * callbackTwistWithCovariance
    */
    void EKFLocalizer::callbackTwistWithCovariance(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
    {
        if (use_twist_with_covariance_)
        {
            geometry_msgs::msg::TwistStamped twist;
            twist.header = msg->header;
            twist.twist = msg->twist.twist;
            current_twist_ptr_ = std::make_shared<geometry_msgs::msg::TwistStamped>(twist);
            current_twist_covariance_ = msg->twist.covariance;
        }
    }


    /*
    * initEKF
    */
    void EKFLocalizer::initEKF()
    {
        Eigen::MatrixXd X = Eigen::MatrixXd::Zero(dim_x_, 1);
        Eigen::MatrixXd P = Eigen::MatrixXd::Identity(dim_x_, dim_x_) * 1.0E15;  // for x & y
        P(IDX::YAW, IDX::YAW) = 50.0;                                            // for yaw
        P(IDX::YAWB, IDX::YAWB) = proc_cov_yaw_bias_d_;                          // for yaw bias
        P(IDX::VX, IDX::VX) = 1000.0;                                            // for vx
        P(IDX::WZ, IDX::WZ) = 50.0;                                              // for wz
        ekf_.init(X, P, extend_state_step_);
    }


    /*
    * predictKinematicsModel
    */
    void EKFLocalizer::predictKinematicsModel()
    {
        /*  == Nonlinear model ==
        *
        * x_{k+1}   = x_k + vx_k * cos(yaw_k + b_k) * dt
        * y_{k+1}   = y_k + vx_k * sin(yaw_k + b_k) * dt
        * yaw_{k+1} = yaw_k + (wz_k) * dt
        * b_{k+1}   = b_k
        * vx_{k+1}  = vz_k
        * wz_{k+1}  = wz_k
        *
        * (b_k : yaw_bias_k)
        */

        /*  == Linearized model ==
        *
        * A = [ 1, 0, -vx*sin(yaw+b)*dt, -vx*sin(yaw+b)*dt, cos(yaw+b)*dt,  0]
        *     [ 0, 1,  vx*cos(yaw+b)*dt,  vx*cos(yaw+b)*dt, sin(yaw+b)*dt,  0]
        *     [ 0, 0,                 1,                 0,             0, dt]
        *     [ 0, 0,                 0,                 1,             0,  0]
        *     [ 0, 0,                 0,                 0,             1,  0]
        *     [ 0, 0,                 0,                 0,             0,  1]
        */
        Eigen::MatrixXd X_curr(dim_x_, 1);  // curent state
        Eigen::MatrixXd X_next(dim_x_, 1);  // predicted state
        ekf_.getLatestX(X_curr);
        
        DEBUG_PRINT_MAT(X_curr.transpose());
        
        Eigen::MatrixXd P_curr;
        ekf_.getLatestP(P_curr);

        const int d_dim_x = dim_x_ex_ - dim_x_;
        const double yaw = X_curr(IDX::YAW);
        const double yaw_bias = X_curr(IDX::YAWB);
        const double vx = X_curr(IDX::VX);
        const double wz = X_curr(IDX::WZ);
        const double dt = ekf_dt_;
        /* Update for latest state */
        X_next(IDX::X) = X_curr(IDX::X) + vx * cos(yaw + yaw_bias) * dt;  // dx = v * cos(yaw)
        X_next(IDX::Y) = X_curr(IDX::Y) + vx * sin(yaw + yaw_bias) * dt;  // dy = v * sin(yaw)
        X_next(IDX::YAW) = X_curr(IDX::YAW) + (wz)*dt;                    // dyaw = omega + omega_bias
        X_next(IDX::YAWB) = yaw_bias;
        X_next(IDX::VX) = vx;
        X_next(IDX::WZ) = wz;

        X_next(IDX::YAW) = std::atan2(std::sin(X_next(IDX::YAW)), std::cos(X_next(IDX::YAW)));
        /* Set A matrix for latest state */
        Eigen::MatrixXd A = Eigen::MatrixXd::Identity(dim_x_, dim_x_);
        A(IDX::X, IDX::YAW) = -vx * sin(yaw + yaw_bias) * dt;
        A(IDX::X, IDX::YAWB) = -vx * sin(yaw + yaw_bias) * dt;
        A(IDX::X, IDX::VX) = cos(yaw + yaw_bias) * dt;
        A(IDX::Y, IDX::YAW) = vx * cos(yaw + yaw_bias) * dt;
        A(IDX::Y, IDX::YAWB) = vx * cos(yaw + yaw_bias) * dt;
        A(IDX::Y, IDX::VX) = sin(yaw + yaw_bias) * dt;
        A(IDX::YAW, IDX::WZ) = dt;
        const double dvx = std::sqrt(P_curr(IDX::VX, IDX::VX));
        const double dyaw = std::sqrt(P_curr(IDX::YAW, IDX::YAW));

        Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(dim_x_, dim_x_);

        if (dvx < 10.0 && dyaw < 1.0)
        {
            // auto covariance calculate for x, y assuming vx & yaw estimation covariance is small

            /* Set covariance matrix Q for process noise. Calc Q by velocity and yaw angle covariance :
            dx = Ax + Jp*w -> Q = Jp*w_cov*Jp'          */
            Eigen::MatrixXd Jp = Eigen::MatrixXd::Zero(2, 2);  // coeff of deviation of vx & yaw
            Jp << cos(yaw), -vx * sin(yaw), sin(yaw), vx * cos(yaw);
            Eigen::MatrixXd Q_vx_yaw = Eigen::MatrixXd::Zero(2, 2);  // cov of vx and yaw

            Q_vx_yaw(0, 0) = P_curr(IDX::VX, IDX::VX) * dt;        // covariance of vx - vx
            Q_vx_yaw(1, 1) = P_curr(IDX::YAW, IDX::YAW) * dt;      // covariance of yaw - yaw
            Q_vx_yaw(0, 1) = P_curr(IDX::VX, IDX::YAW) * dt;       // covariance of vx - yaw
            Q_vx_yaw(1, 0) = P_curr(IDX::YAW, IDX::VX) * dt;       // covariance of yaw - vx
            Q.block(0, 0, 2, 2) = Jp * Q_vx_yaw * Jp.transpose();  // for pos_x & pos_y
        }
        else
        {
            // vx & vy is not converged yet, set constant value.
            Q(IDX::X, IDX::X) = 0.05;
            Q(IDX::Y, IDX::Y) = 0.05;
        }

        Q(IDX::YAW, IDX::YAW) = proc_cov_yaw_d_;         // for yaw
        Q(IDX::YAWB, IDX::YAWB) = proc_cov_yaw_bias_d_;  // for yaw bias
        Q(IDX::VX, IDX::VX) = proc_cov_vx_d_;            // for vx
        Q(IDX::WZ, IDX::WZ) = proc_cov_wz_d_;            // for wz

        ekf_.predictWithDelay(X_next, A, Q);

        // debug
        Eigen::MatrixXd X_result(dim_x_, 1);
        ekf_.getLatestX(X_result);
        DEBUG_PRINT_MAT(X_result.transpose());
        DEBUG_PRINT_MAT((X_result - X_curr).transpose());
    }

    /*
    * measurementUpdatePose
    */
    void EKFLocalizer::measurementUpdatePose(const geometry_msgs::msg::PoseStamped& pose)
    {
        if (pose.header.frame_id != pose_frame_id_)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), clk_, 2000, "pose frame_id is %s, but pose_frame is set as %s. They must be same.",
                                    pose.header.frame_id.c_str(), pose_frame_id_.c_str());
        }
        Eigen::MatrixXd X_curr(dim_x_, 1);  // curent state
        ekf_.getLatestX(X_curr);
        DEBUG_PRINT_MAT(X_curr.transpose());

        constexpr int dim_y = 3;  // pos_x, pos_y, yaw, depending on Pose output
        const rclcpp::Time t_curr = this->now();

        /* Calculate delay step */
        double delay_time = (t_curr - pose.header.stamp).seconds() + pose_additional_delay_;
        if (delay_time < 0.0)
        {
            delay_time = 0.0;
            RCLCPP_WARN_THROTTLE(get_logger(), clk_, 1000, "Pose time stamp is inappropriate, set delay to 0[s]. delay = %f", delay_time);
        }
        int delay_step = std::roundf(delay_time / ekf_dt_);
        if (delay_step > extend_state_step_ - 1)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), clk_, 1000,
                                    "Pose delay exceeds the compensation limit, ignored. delay: %f[s], limit = "
                                    "extend_state_step * ekf_dt : %f [s]",
                                    delay_time, extend_state_step_ * ekf_dt_);
            return;
        }
        DEBUG_INFO("delay_time: %f [s]", delay_time);

        /* Set yaw */
        const double yaw_curr = ekf_.getXelement((unsigned int)(delay_step * dim_x_ + IDX::YAW));
        double yaw = tf2::getYaw(pose.pose.orientation);
        const double ekf_yaw = ekf_.getXelement(delay_step * dim_x_ + IDX::YAW);
        const double yaw_error = normalizeYaw(yaw - ekf_yaw);  // normalize the error not to exceed 2 pi
        yaw = yaw_error + ekf_yaw;

        /* Set measurement matrix */
        Eigen::MatrixXd y(dim_y, 1);
        y << pose.pose.position.x, pose.pose.position.y, yaw;

        if (isnan(y.array()).any() || isinf(y.array()).any())
        {
            RCLCPP_WARN(get_logger(), "[EKF] pose measurement matrix includes NaN of Inf. ignore update. check pose message.");
            return;
        }

        /* Gate */
        Eigen::MatrixXd y_ekf(dim_y, 1);
        y_ekf << ekf_.getXelement(delay_step * dim_x_ + IDX::X), ekf_.getXelement(delay_step * dim_x_ + IDX::Y), ekf_yaw;
        Eigen::MatrixXd P_curr, P_y;
        ekf_.getLatestP(P_curr);
        P_y = P_curr.block(0, 0, dim_y, dim_y);
        if (!mahalanobisGate(pose_gate_dist_, y_ekf, y, P_y))
        {
            RCLCPP_WARN_THROTTLE(get_logger(), clk_ , 2000,"[EKF] Pose measurement update, mahalanobis distance is over limit. ignore "
                                        "measurement data.");
            return;
        }

        DEBUG_PRINT_MAT(y.transpose());
        DEBUG_PRINT_MAT(y_ekf.transpose());
        DEBUG_PRINT_MAT((y - y_ekf).transpose());

        /* Set measurement matrix */
        Eigen::MatrixXd C = Eigen::MatrixXd::Zero(dim_y, dim_x_);
        C(0, IDX::X) = 1.0;    // for pos x
        C(1, IDX::Y) = 1.0;    // for pos y
        C(2, IDX::YAW) = 1.0;  // for yaw

        /* Set measurement noise covariancs */
        Eigen::MatrixXd R = Eigen::MatrixXd::Zero(dim_y, dim_y);
        if (use_pose_with_covariance_)
        {
            R(0, 0) = current_pose_covariance_.at(0);   // x - x
            R(0, 1) = current_pose_covariance_.at(1);   // x - y
            R(0, 2) = current_pose_covariance_.at(5);   // x - yaw
            R(1, 0) = current_pose_covariance_.at(6);   // y - x
            R(1, 1) = current_pose_covariance_.at(7);   // y - y
            R(1, 2) = current_pose_covariance_.at(11);  // y - yaw
            R(2, 0) = current_pose_covariance_.at(30);  // yaw - x
            R(2, 1) = current_pose_covariance_.at(31);  // yaw - y
            R(2, 2) = current_pose_covariance_.at(35);  // yaw - yaw
        }
        else
        {
            const double ekf_yaw = ekf_.getXelement(IDX::YAW);
            const double vx = ekf_.getXelement(IDX::VX);
            const double wz = ekf_.getXelement(IDX::WZ);
            const double cov_pos_x = std::pow(pose_measure_uncertainty_time_ * vx * cos(ekf_yaw), 2.0);
            const double cov_pos_y = std::pow(pose_measure_uncertainty_time_ * vx * sin(ekf_yaw), 2.0);
            const double cov_yaw = std::pow(pose_measure_uncertainty_time_ * wz, 2.0);
            R(0, 0) = std::pow(pose_stddev_x_, 2) + cov_pos_x;  // pos_x
            R(1, 1) = std::pow(pose_stddev_y_, 2) + cov_pos_y;  // pos_y
            R(2, 2) = std::pow(pose_stddev_yaw_, 2) + cov_yaw;  // yaw
        }

        /* In order to avoid a large change at the time of updating, measuremeent update is performed by dividing at every
        * step. */
        R *= (ekf_rate_ / pose_rate_);

        ekf_.updateWithDelay(y, C, R, delay_step);

        // debug
        Eigen::MatrixXd X_result(dim_x_, 1);
        ekf_.getLatestX(X_result);
        DEBUG_PRINT_MAT(X_result.transpose());
        DEBUG_PRINT_MAT((X_result - X_curr).transpose());
    }

    /*
    * measurementUpdateTwist
    */
    void EKFLocalizer::measurementUpdateTwist(const geometry_msgs::msg::TwistStamped& twist)
    {
        if (twist.header.frame_id != child_frame_id_)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), clk_ , 2000,"twist frame_id must be %s", child_frame_id_);
        }

        Eigen::MatrixXd X_curr(dim_x_, 1);  // curent state
        ekf_.getLatestX(X_curr);
        DEBUG_PRINT_MAT(X_curr.transpose());

        constexpr int dim_y = 2;  // vx, wz
        const rclcpp::Time t_curr = this->now();

        /* Calculate delay step */
        double delay_time = (t_curr - twist.header.stamp).seconds() + twist_additional_delay_;
        if (delay_time < 0.0)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), clk_ , 1000, "Twist time stamp is inappropriate (delay = %f [s]), set delay to 0[s].",
                                    delay_time);
            delay_time = 0.0;
        }
        int delay_step = std::roundf(delay_time / ekf_dt_);
        if (delay_step > extend_state_step_ - 1)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), clk_ , 1000, 
                                    "Twist delay exceeds the compensation limit, ignored. delay: %f[s], limit = "
                                    "extend_state_step * ekf_dt : %f [s]",
                                    delay_time, extend_state_step_ * ekf_dt_);
            return;
        }
        DEBUG_INFO("delay_time: %f [s]", delay_time);

        /* Set measurement matrix */
        Eigen::MatrixXd y(dim_y, 1);
        y << twist.twist.linear.x, twist.twist.angular.z;

        if (isnan(y.array()).any() || isinf(y.array()).any())
        {
            RCLCPP_WARN(get_logger(), "[EKF] twist measurement matrix includes NaN of Inf. ignore update. check twist message.");
            return;
        }

        /* Gate */
        Eigen::MatrixXd y_ekf(dim_y, 1);
        y_ekf << ekf_.getXelement(delay_step * dim_x_ + IDX::VX), ekf_.getXelement(delay_step * dim_x_ + IDX::WZ);
        Eigen::MatrixXd P_curr, P_y;
        ekf_.getLatestP(P_curr);
        P_y = P_curr.block(4, 4, dim_y, dim_y);
        if (!mahalanobisGate(twist_gate_dist_, y_ekf, y, P_y))
        {
            RCLCPP_WARN_THROTTLE(get_logger(), clk_ , 2000 , "[EKF] Twist measurement update, mahalanobis distance is over limit. ignore "
                                        "measurement data.");
            return;
        }

        DEBUG_PRINT_MAT(y.transpose());
        DEBUG_PRINT_MAT(y_ekf.transpose());
        DEBUG_PRINT_MAT((y - y_ekf).transpose());

        /* Set measurement matrix */
        Eigen::MatrixXd C = Eigen::MatrixXd::Zero(dim_y, dim_x_);
        C(0, IDX::VX) = 1.0;  // for vx
        C(1, IDX::WZ) = 1.0;  // for wz

        /* Set measurement noise covariancs */
        Eigen::MatrixXd R = Eigen::MatrixXd::Zero(dim_y, dim_y);
        if (use_twist_with_covariance_)
        {
            R(0, 0) = current_twist_covariance_.at(0);   // vx - vx
            R(0, 1) = current_twist_covariance_.at(5);   // vx - wz
            R(1, 0) = current_twist_covariance_.at(30);  // wz - vx
            R(1, 1) = current_twist_covariance_.at(35);  // wz - wz
        }
        else
        {
            R(0, 0) = twist_stddev_vx_ * twist_stddev_vx_ * ekf_dt_;  // for vx
            R(1, 1) = twist_stddev_wz_ * twist_stddev_wz_ * ekf_dt_;  // for wz
        }

        /* In order to avoid a large change by update, measurement update is performed by dividing at every step. */
        R *= (ekf_rate_ / twist_rate_);

        ekf_.updateWithDelay(y, C, R, delay_step);

        // debug
        Eigen::MatrixXd X_result(dim_x_, 1);
        ekf_.getLatestX(X_result);
        DEBUG_PRINT_MAT(X_result.transpose());
        DEBUG_PRINT_MAT((X_result - X_curr).transpose());
    }

    /*
    * mahalanobisGate
    */
    bool EKFLocalizer::mahalanobisGate(const double& dist_max, const Eigen::MatrixXd& x, const Eigen::MatrixXd& obj_x,
                                    const Eigen::MatrixXd& cov)
    {
        Eigen::MatrixXd mahalanobis_squared = (x - obj_x).transpose() * cov.inverse() * (x - obj_x);
        DEBUG_INFO("measurement update: mahalanobis = %f, gate limit = %f", std::sqrt(mahalanobis_squared(0)), dist_max);
        if (mahalanobis_squared(0) > dist_max * dist_max)
        {
            return false;
        }

        return true;
    }


    /*
    * publishEstimateResult
    */
    void EKFLocalizer::publishEstimateResult()
    {
        rclcpp::Time current_time = this->now();
        Eigen::MatrixXd X(dim_x_, 1);
        Eigen::MatrixXd P(dim_x_, dim_x_);
        ekf_.getLatestX(X);
        ekf_.getLatestP(P);

        /* publish latest pose */
        pub_pose_->publish(current_ekf_pose_);

        /* publish latest pose with covariance */
        geometry_msgs::msg::PoseWithCovarianceStamped pose_cov;
        pose_cov.header.stamp = current_time;
        pose_cov.header.frame_id = current_ekf_pose_.header.frame_id;
        pose_cov.pose.pose = current_ekf_pose_.pose;
        pose_cov.pose.covariance[0] = P(IDX::X, IDX::X);
        pose_cov.pose.covariance[1] = P(IDX::X, IDX::Y);
        pose_cov.pose.covariance[5] = P(IDX::X, IDX::YAW);
        pose_cov.pose.covariance[6] = P(IDX::Y, IDX::X);
        pose_cov.pose.covariance[7] = P(IDX::Y, IDX::Y);
        pose_cov.pose.covariance[11] = P(IDX::Y, IDX::YAW);
        pose_cov.pose.covariance[30] = P(IDX::YAW, IDX::X);
        pose_cov.pose.covariance[31] = P(IDX::YAW, IDX::Y);
        pose_cov.pose.covariance[35] = P(IDX::YAW, IDX::YAW);
        pub_pose_cov_->publish(pose_cov);

        /* publish latest twist */
        pub_twist_->publish(current_ekf_twist_);

        /* publish latest twist with covariance */
        geometry_msgs::msg::TwistWithCovarianceStamped twist_cov;
        twist_cov.header.stamp = current_time;
        twist_cov.header.frame_id = current_ekf_twist_.header.frame_id;
        twist_cov.twist.twist = current_ekf_twist_.twist;
        twist_cov.twist.covariance[0] = P(IDX::VX, IDX::VX);
        twist_cov.twist.covariance[5] = P(IDX::VX, IDX::WZ);
        twist_cov.twist.covariance[30] = P(IDX::WZ, IDX::VX);
        twist_cov.twist.covariance[35] = P(IDX::WZ, IDX::WZ);
        pub_twist_cov_->publish(twist_cov);


        /* publish yaw bias */
        std_msgs::msg::Float64 yawb;
        yawb.data = X(IDX::YAWB);
        pub_yaw_bias_->publish(yawb);

        /* debug measured pose */
        if (current_pose_ptr_ != nullptr)
        {
            geometry_msgs::msg::PoseStamped p;
            p = *current_pose_ptr_;
            p.header.stamp = current_time;
            pub_measured_pose_->publish(p);
        }

        /* debug publish */
        double RAD2DEG = 180.0 / 3.141592;
        double pose_yaw = 0.0;
        if (current_pose_ptr_ != nullptr)
            pose_yaw = tf2::getYaw(current_pose_ptr_->pose.orientation) * RAD2DEG;

        std_msgs::msg::Float64MultiArray msg;
        msg.data.push_back(X(IDX::YAW) * RAD2DEG);   // [0] ekf yaw angle
        msg.data.push_back(pose_yaw);                // [1] measurement yaw angle
        msg.data.push_back(X(IDX::YAWB) * RAD2DEG);  // [2] yaw bias
        pub_debug_->publish(msg);
    }

    double EKFLocalizer::normalizeYaw(const double& yaw)
    {
        return std::atan2(std::sin(yaw), std::cos(yaw));
    }

} //namespace ekf_localizer


#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(ekf_localizer::EKFLocalizer)