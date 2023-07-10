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

#include <gtest/gtest.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <mutex>
#include "ekf_localizer/ekf_localizer.hpp"


class EKFLocalizerTestSuite : public rclcpp::Node
{
    private:
        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_twist_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
        rclcpp::TimerBase::SharedPtr timer_;

    public:

    EKFLocalizerTestSuite(std::string node_name) : Node (node_name)
    {
        sub_twist_ = create_subscription<geometry_msgs::msg::TwistStamped>("ekf_twist", 1, std::bind(&EKFLocalizerTestSuite::callbackTwist, this, std::placeholders::_1));
        sub_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>("ekf_pose", 1, std::bind(&EKFLocalizerTestSuite::callbackPose, this, std::placeholders::_1));
        timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&EKFLocalizerTestSuite::timerCallback, this));
    }

    void timerCallback()
    {
        /* !!! this should be defined before sendTransform() !!! */
        static tf2_ros::TransformBroadcaster br(shared_from_this());
        geometry_msgs::msg::TransformStamped sended;

        rclcpp::Time current_time = rclcpp::Time(0,0);

        sended.header.stamp = current_time;
        sended.header.frame_id = frame_id_a_;
        sended.child_frame_id = frame_id_b_;
        sended.transform.translation.x = -7.11;
        sended.transform.translation.y = 0.0;
        sended.transform.translation.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, 0.5);
        sended.transform.rotation.x = q.x();
        sended.transform.rotation.y = q.y();
        sended.transform.rotation.z = q.z();
        sended.transform.rotation.w = q.w();

        br.sendTransform(sended);
    }

    void callbackPose(const geometry_msgs::msg::PoseStamped::SharedPtr pose)
    {
        current_pose_ptr_ = std::make_shared<geometry_msgs::msg::PoseStamped>(*pose);
    }

    void callbackTwist(const geometry_msgs::msg::TwistStamped::SharedPtr twist)
    {
        current_twist_ptr_ = std::make_shared<geometry_msgs::msg::TwistStamped>(*twist);
    }

    void resetCurrentPoseAndTwist()
    {
        current_pose_ptr_ = nullptr;
        current_twist_ptr_ = nullptr;
    }

    std::string frame_id_a_ = "world";
    std::string frame_id_b_ = "base_link";
    std::shared_ptr<geometry_msgs::msg::PoseStamped> current_pose_ptr_;
    std::shared_ptr<geometry_msgs::msg::TwistStamped> current_twist_ptr_;

};

template<class MessageT>
class TestPublisher: public rclcpp::Node
{
    private:
    // Template publisher for testing
    typename rclcpp::Publisher<MessageT>::SharedPtr topic_pub_;
    mutable std::mutex msg_mutex_;

    public: 

    TestPublisher(std::string node_name, std::string topic_name, rclcpp::QoS qos = rclcpp::SystemDefaultsQoS()) : Node(node_name)
    {
        topic_pub_ = create_publisher<MessageT>(topic_name, qos);

    }

    void publish(const MessageT& msg){
        std::lock_guard<std::mutex> lock(msg_mutex_);
        topic_pub_->publish(msg);
    }
};


TEST(EKFLocalizerTestSuite, measurementUpdatePose)
{
    auto test_node = std::make_shared<EKFLocalizerTestSuite>("pose_test_node");
    auto worker_node = std::make_shared<ekf_localizer::EKFLocalizer>(rclcpp::NodeOptions());
    worker_node->configure(); //Call configure state transition
    worker_node->activate();  //Call activate state transition to get not read for runtime
    // Create publisher node
    auto pub_pose = std::make_shared<TestPublisher<geometry_msgs::msg::PoseStamped>>("pose_publisher_node", "in_pose");

    geometry_msgs::msg::PoseStamped in_pose;
    in_pose.header.frame_id = "world";
    in_pose.pose.position.x = 1.0;
    in_pose.pose.position.y = 2.0;
    in_pose.pose.position.z = 3.0;
    in_pose.pose.orientation.x = 0.0;
    in_pose.pose.orientation.y = 0.0;
    in_pose.pose.orientation.z = 0.0;
    in_pose.pose.orientation.w = 1.0;

    /* test for valid value */
    const double pos_x = 12.3;
    in_pose.pose.position.x = pos_x;  // for valid value

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(test_node->get_node_base_interface());
    executor.add_node(pub_pose->get_node_base_interface());
    executor.add_node(worker_node->get_node_base_interface());
    

    for (int i = 0; i < 20; ++i)
    {
        in_pose.header.stamp = rclcpp::Time(0,0);
        pub_pose->publish(in_pose);
        executor.spin_once();
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    ASSERT_FALSE(test_node->current_pose_ptr_ == nullptr);
    ASSERT_FALSE(test_node->current_twist_ptr_ == nullptr);


    double ekf_x = test_node->current_pose_ptr_->pose.position.x;
    bool is_succeeded = !(std::isnan(ekf_x) || std::isinf(ekf_x));
    ASSERT_EQ(true, is_succeeded) << "ekf result includes invalid value.";
    ASSERT_TRUE(std::fabs(ekf_x - pos_x) < 0.1) << "ekf pos x: " << ekf_x << " should be close to " << pos_x;

    /* test for invalid value */
    in_pose.pose.position.x = NAN;  // check for invalid values
    for (int i = 0; i < 10; ++i)
    {
        in_pose.header.stamp = rclcpp::Time(0,0);
        pub_pose->publish(in_pose);
        executor.spin_once();
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
    is_succeeded = !(std::isnan(ekf_x) || std::isinf(ekf_x));
    ASSERT_EQ(true, is_succeeded) << "ekf result includes invalid value.";

    test_node->resetCurrentPoseAndTwist();
}

TEST(EKFLocalizerTestSuite, measurementUpdateTwist)
{
    auto test_node = std::make_shared<EKFLocalizerTestSuite>("twist_test_node");
    auto worker_node = std::make_shared<ekf_localizer::EKFLocalizer>(rclcpp::NodeOptions());
    worker_node->configure(); //Call configure state transition
    worker_node->activate();  //Call activate state transition to get not read for runtime
    // Create publisher node
    auto pub_twist = std::make_shared<TestPublisher<geometry_msgs::msg::TwistStamped>>("twist_publisher_node", "in_twist");

    geometry_msgs::msg::TwistStamped in_twist;
    in_twist.header.frame_id = "base_link";

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(test_node->get_node_base_interface());
    executor.add_node(pub_twist->get_node_base_interface());
    executor.add_node(worker_node->get_node_base_interface());

    /* test for valid value */
    const double vx = 12.3;
    in_twist.twist.linear.x = vx;  // for vaild value
    for (int i = 0; i < 20; ++i)
    {
        in_twist.header.stamp = rclcpp::Time(0,0);
        pub_twist->publish(in_twist);
        executor.spin_once();
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    ASSERT_FALSE(test_node->current_pose_ptr_ == nullptr);
    ASSERT_FALSE(test_node->current_twist_ptr_ == nullptr);

    double ekf_vx = test_node->current_twist_ptr_->twist.linear.x;
    bool is_succeeded = !(std::isnan(ekf_vx) || std::isinf(ekf_vx));
    ASSERT_EQ(true, is_succeeded) << "ekf result includes invalid value.";
    ASSERT_TRUE(std::fabs(ekf_vx - vx) < 0.1) << "ekf vel x: " << ekf_vx << ", should be close to " << vx;

    /* test for invalid value */
    in_twist.twist.linear.x = NAN;  // check for invalid values

    for (int i = 0; i < 10; ++i)
    {
        in_twist.header.stamp = rclcpp::Time(0,0);
        pub_twist->publish(in_twist);
        executor.spin_once();
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    ekf_vx = test_node->current_twist_ptr_->twist.linear.x;
    is_succeeded = !(std::isnan(ekf_vx) || std::isinf(ekf_vx));
    ASSERT_EQ(true, is_succeeded) << "ekf result includes invalid value.";

    test_node->resetCurrentPoseAndTwist();
}

TEST(EKFLocalizerTestSuite, measurementUpdatePoseWithCovariance)
{
    std::vector<rclcpp::Parameter> initial_parameters =  {rclcpp::Parameter("use_pose_with_covariance", true)};
    rclcpp::NodeOptions node_options;
    node_options.parameter_overrides(initial_parameters);

    auto test_node = std::make_shared<EKFLocalizerTestSuite>("pose_with_cov_test_node");
    auto worker_node = std::make_shared<ekf_localizer::EKFLocalizer>(node_options);
    worker_node->configure(); //Call configure state transition
    worker_node->activate();  //Call activate state transition to get not read for runtime
    // Create publisher node
    auto pub_pose_cov = std::make_shared<TestPublisher<geometry_msgs::msg::PoseWithCovarianceStamped>>("pose_with_covariance_pub_node", "in_pose_with_covariance");
    geometry_msgs::msg::PoseWithCovarianceStamped in_pose;
    in_pose.header.frame_id = "world";
    in_pose.pose.pose.position.x = 1.0;
    in_pose.pose.pose.position.y = 2.0;
    in_pose.pose.pose.position.z = 3.0;
    in_pose.pose.pose.orientation.x = 0.0;
    in_pose.pose.pose.orientation.y = 0.0;
    in_pose.pose.pose.orientation.z = 0.0;
    in_pose.pose.pose.orientation.w = 1.0;
    for (int i = 0; i < 36; ++i)
    {
        in_pose.pose.covariance[i] = 0.1;
    }

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(test_node->get_node_base_interface());
    executor.add_node(pub_pose_cov->get_node_base_interface());
    executor.add_node(worker_node->get_node_base_interface());

    /* test for valid value */
    const double pos_x = 99.3;
    in_pose.pose.pose.position.x = pos_x;  // for vaild value

    for (int i = 0; i < 20; ++i)
    {
        in_pose.header.stamp = rclcpp::Time(0,0);
        pub_pose_cov->publish(in_pose);
        executor.spin_once();
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    ASSERT_FALSE(test_node->current_pose_ptr_ == nullptr);
    ASSERT_FALSE(test_node->current_twist_ptr_ == nullptr);


    double ekf_x = test_node->current_pose_ptr_->pose.position.x;
    bool is_succeeded = !(std::isnan(ekf_x) || std::isinf(ekf_x));
    ASSERT_EQ(true, is_succeeded) << "ekf result includes invalid value.";
    ASSERT_TRUE(std::fabs(ekf_x - pos_x) < 0.1) << "ekf pos x: " << ekf_x << " should be close to " << pos_x;


      /* test for invalid value */
    in_pose.pose.pose.position.x = NAN;  // check for invalid values
    for (int i = 0; i < 10; ++i)
    {
        in_pose.header.stamp = rclcpp::Time(0,0);
        pub_pose_cov->publish(in_pose);
        executor.spin_once();
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
    is_succeeded = !(std::isnan(ekf_x) || std::isinf(ekf_x));
    ASSERT_EQ(true, is_succeeded) << "ekf result includes invalid value.";

    test_node->resetCurrentPoseAndTwist();
    
}

TEST(EKFLocalizerTestSuite, measurementUpdateTwistWithCovariance)
{
    std::vector<rclcpp::Parameter> initial_parameters =  {rclcpp::Parameter("use_twist_with_covariance", true)};
    rclcpp::NodeOptions node_options;
    node_options.parameter_overrides(initial_parameters);

    auto test_node = std::make_shared<EKFLocalizerTestSuite>("twist_with_cov_test_node");
    auto worker_node = std::make_shared<ekf_localizer::EKFLocalizer>(node_options);
    
    // Create publisher node
    auto pub_twist_cov = std::make_shared<TestPublisher<geometry_msgs::msg::TwistWithCovarianceStamped>>("twist_covariance_publisher_node", "in_twist_with_covariance");
    
    geometry_msgs::msg::TwistWithCovarianceStamped in_twist;
    in_twist.header.frame_id = "base_link";

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(test_node->get_node_base_interface());
    executor.add_node(pub_twist_cov->get_node_base_interface());
    executor.add_node(worker_node->get_node_base_interface());

    worker_node->configure(); //Call configure state transition
    worker_node->activate();  //Call activate state transition to get not read for runtime

    /* test for valid value */
    const double vx = 12.3;
    in_twist.twist.twist.linear.x = vx;  // for vaild value
    for (int i = 0; i < 36; ++i)
    {
        in_twist.twist.covariance[i] = 0.1;
    }
    for (int i = 0; i < 10; ++i)
    {
        in_twist.header.stamp = rclcpp::Time(0,0);
        pub_twist_cov->publish(in_twist);
        executor.spin_once();
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
    
    ASSERT_FALSE(test_node->current_pose_ptr_ == nullptr);
    ASSERT_FALSE(test_node->current_twist_ptr_ == nullptr);

    double ekf_vx = test_node->current_twist_ptr_->twist.linear.x;
    bool is_succeeded = !(std::isnan(ekf_vx) || std::isinf(ekf_vx));
    ASSERT_EQ(true, is_succeeded) << "ekf result includes invalid value.";
    ASSERT_TRUE((ekf_vx - vx) < 0.1) << "vel x should be close to " << vx;

    /* test for invalid value */
    in_twist.twist.twist.linear.x = NAN;  // check for invalid values
    for (int i = 0; i < 10; ++i)
    {
        in_twist.header.stamp = rclcpp::Time(0,0);
        pub_twist_cov->publish(in_twist);
        executor.spin_once();
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    ekf_vx = test_node->current_twist_ptr_->twist.linear.x;
    is_succeeded = !(std::isnan(ekf_vx) || std::isinf(ekf_vx));
    ASSERT_EQ(true, is_succeeded) << "ekf result includes invalid value.";
}

int main(int argc, char ** argv)
{
    ::testing::InitGoogleTest(&argc, argv);

    //Initialize ROS
    rclcpp::init(argc, argv);

    bool success = RUN_ALL_TESTS();

    //shutdown ROS
    rclcpp::shutdown();

    return success;
} 