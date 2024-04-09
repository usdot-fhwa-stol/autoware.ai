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

#include <condition_variable>
#include <queue>
#include <thread>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <carma_ros2_utils/carma_lifecycle_node.hpp>

#include <autoware_msgs/msg/lane.hpp>
#include <autoware_msgs/msg/lane_array.hpp>

#include "tf2_ros/buffer.h"

#include <get_file.hpp>

namespace points_map_loader
{

    struct Area {
        std::string path;
        double x_min;
        double y_min;
        double z_min;
        double x_max;
        double y_max;
        double z_max;
    };

    constexpr int DEFAULT_UPDATE_RATE = 1000; // ms
    constexpr double MARGIN_UNIT = 100; // meter
    constexpr int ROUNDING_UNIT = 1000; // meter
    const std::string AREALIST_FILENAME = "arealist.txt";
    const std::string TEMPORARY_DIRNAME = "/tmp/";

    typedef std::vector<Area> AreaList;
    typedef std::vector<std::vector<std::string>> Tbl;

    int fallback_rate;
    double margin;
    bool can_download;

    rclcpp::Time gnss_time;
    rclcpp::Time current_time;

    std_msgs::msg::Bool stat_msg;
    AreaList all_areas;
    AreaList downloaded_areas;
    std::mutex downloaded_areas_mtx;
    std::vector<std::string> cached_arealist_paths;

    class PointsMapLoader : public carma_ros2_utils::CarmaLifecycleNode {
    private:
    
        std::queue<geometry_msgs::msg::Point> queue_; // takes priority over look_ahead_queue_
        std::queue<geometry_msgs::msg::Point> look_ahead_queue_;
        std::mutex mtx_;
        std::condition_variable cv_;

        // Publishers
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_pub;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stat_pub;

        // Subscribers
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr gnss_sub;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_sub;
        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_sub;
        rclcpp::Subscription<autoware_msgs::msg::LaneArray>::SharedPtr waypoints_sub;
        
        GetFile gf;

        // Parameters
        int update_rate = DEFAULT_UPDATE_RATE;
        std::string area = "";
        std::string load_type = "";
        std::string path_area_list = "";
        std::vector<std::string> pcd_path = {};
        std::string host_name = HTTP_HOSTNAME;
        int port = HTTP_PORT;
        std::string user = HTTP_USER;
        std::string password = HTTP_PASSWORD;


        void enqueue(const geometry_msgs::msg::Point& p);
        void enqueue_look_ahead(const geometry_msgs::msg::Point& p);
        void clear();
        void clear_look_ahead();
        geometry_msgs::msg::Point dequeue();

        void publish_pcd(sensor_msgs::msg::PointCloud2 pcd, const int* errp);

        Tbl read_csv(const std::string& path);
        void write_csv(const std::string& path, const Tbl& tbl);
        AreaList read_arealist(const std::string& path);
        void write_arealist(const std::string& path, const AreaList& areas);
        bool is_downloaded(const std::string& path);
        bool is_in_area(double x, double y, const Area& area, double m);
        std::string create_location(int x, int y);
        void cache_arealist(const Area& area, AreaList& areas);
        int download(GetFile gf, const std::string& tmp, const std::string& loc, const std::string& filename);
        
        void download_map();

        sensor_msgs::msg::PointCloud2 create_pcd(const geometry_msgs::msg::Point& p);
        sensor_msgs::msg::PointCloud2 create_pcd(const std::vector<std::string>& pcd_paths, int* ret_err);
        
        void publish_gnss_pcd(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void publish_dragged_pcd(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
        void publish_current_pcd(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void request_lookahead_download(const autoware_msgs::msg::LaneArray::SharedPtr msg);

        rclcpp::TimerBase::SharedPtr timer_;
        void timer_callback();

        void run();

    public:
        explicit PointsMapLoader(const rclcpp::NodeOptions &);

        carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State &prev_state);
        carma_ros2_utils::CallbackReturn handle_on_activate(const rclcpp_lifecycle::State &prev_state);


    };
    
    void print_usage()
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("map_file_ros2"), "Usage:");
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("map_file_ros2"), "ros2 launch map_file_ros2 points_map_loader.launch.py --load_type:=noupdate --pcd_path_parameter:=[PCD]...");
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("map_file_ros2"), "ros2 launch map_file_ros2 points_map_loader.launch.py --path_area_list:={1x1|3x3|5x5|7x7|9x9} --pcd_path_parameter:=[PCD]...");
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("map_file_ros2"), "ros2 launch map_file_ros2 points_map_loader.launch --path_area_list:{1x1|3x3|5x5|7x7|9x9} --load_type:=download");
    }
}