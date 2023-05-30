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

#include <points_map_loader.hpp>
// #include <lifecycle_msgs/msg/state.hpp>


namespace points_map_loader {


    PointsMapLoader::PointsMapLoader(const rclcpp::NodeOptions &options) : carma_ros2_utils::CarmaLifecycleNode(options)
    {
        // Initialize parameters
        update_rate = declare_parameter<int>("update_rate", update_rate);

        area = declare_parameter<std::string>("area", area);
        load_type = declare_parameter<std::string>("load_type", load_type);
        path_area_list = declare_parameter<std::string>("path_area_list", path_area_list);
        // parameter as lists
        declare_parameter("pcd_path_parameter", pcd_path);

        host_name = declare_parameter<std::string>("host_name", host_name);
        port = declare_parameter<int>("port", port);
        user = declare_parameter<std::string>("user", user);
        password = declare_parameter<std::string>("password", password);

    }

    carma_ros2_utils::CallbackReturn PointsMapLoader::handle_on_configure(const rclcpp_lifecycle::State &prev_state)
    {
        // Get Parameters
        get_parameter<int>("update_rate", update_rate);
        get_parameter<std::string>("area", area);
        get_parameter<std::string>("load_type", load_type);
        get_parameter<std::string>("path_area_list", path_area_list);
        
        // path pcd - list of strings
        rclcpp::Parameter pcd_path_parameter = get_parameter("pcd_path_parameter");
        pcd_path = pcd_path_parameter.as_string_array();
        

        // Create publishers
        // NOTE: Currently, intra-process comms must be disabled for the following two publishers that are transient_local: https://github.com/ros2/rclcpp/issues/1753
        rclcpp::PublisherOptions intra_proc_disabled; 
        intra_proc_disabled.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable; // Disable intra-process comms for this PublisherOptions object
        // Create a publisher that will send all previously published messages to late-joining subscribers ONLY If the subscriber is transient_local too
        auto pub_qos_transient_local = rclcpp::QoS(rclcpp::KeepAll()); // A publisher with this QoS will store the "Last" message that it has sent on the topic
        pub_qos_transient_local.transient_local();

        pcd_pub = create_publisher<sensor_msgs::msg::PointCloud2>("points_map", pub_qos_transient_local, intra_proc_disabled); //Make latched
        stat_pub = create_publisher<std_msgs::msg::Bool>("pmap_stat", pub_qos_transient_local, intra_proc_disabled); //Make latched

        return CallbackReturn::SUCCESS;
    }

    carma_ros2_utils::CallbackReturn PointsMapLoader::handle_on_activate(const rclcpp_lifecycle::State &prev_state)
    {
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&PointsMapLoader::timer_callback, this));
        return CallbackReturn::SUCCESS;
    }

    void PointsMapLoader::timer_callback()
    {
        if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE){
            run();   
            timer_->cancel();
        }
    }

    void PointsMapLoader::run(){
        if (load_type == "noupdate")
		        margin = -1;
            else if (area == "1x1")
                margin = 0;
            else if (area == "3x3")
                margin = MARGIN_UNIT * 1;
            else if (area == "5x5")
                margin = MARGIN_UNIT * 2;
            else if (area == "7x7")
                margin = MARGIN_UNIT * 3;
            else if (area == "9x9")
                margin = MARGIN_UNIT * 4;
            else {
                print_usage();
                RCLCPP_ERROR_STREAM(get_logger(), "Invalid area argument");
                
            }

            std::string arealist_path;
            std::vector<std::string> pcd_paths;
            if (margin < 0) {
                can_download = false;
                // If area = no_update get pcd paths
                pcd_paths.insert(pcd_paths.end(), pcd_path.begin(), pcd_path.end());
            } else {
                if (load_type == "download")
                {
                    can_download = true;
                    
                    get_parameter<std::string>("host_name", host_name);
                    get_parameter<int>("port", port);
                    get_parameter<std::string>("user", user);
                    get_parameter<std::string>("password", password);

                    gf = GetFile(host_name, port, user, password);
                }
                else{
                    can_download = false;
                    arealist_path += path_area_list;
                    pcd_paths.insert(pcd_paths.end(), pcd_path.begin(), pcd_path.end());
                }

            } 
            
            stat_msg.data = false;
            stat_pub->publish(stat_msg);
            sensor_msgs::msg::PointCloud2 pcd;

            if (margin < 0) {
                int err = 0;
                pcd = create_pcd(pcd_paths, &err);
                RCLCPP_INFO_STREAM(get_logger(), "Publishing pcd");
                publish_pcd(pcd, &err);
            } else{
                fallback_rate = update_rate * 2; // XXX better way?

                // Create subscribers
                gnss_sub = create_subscription<geometry_msgs::msg::PoseStamped>("gnss_pose", 1000, std::bind(&PointsMapLoader::publish_gnss_pcd, this, std::placeholders::_1));
                current_sub = create_subscription<geometry_msgs::msg::PoseStamped>("current_pose", 1000, std::bind(&PointsMapLoader::publish_current_pcd, this, std::placeholders::_1));
                initial_sub = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 1, std::bind(&PointsMapLoader::publish_dragged_pcd, this, std::placeholders::_1));

                if (can_download){
                    waypoints_sub = create_subscription<autoware_msgs::msg::LaneArray>("traffic_waypoints_array", 1, std::bind(&PointsMapLoader::request_lookahead_download, this, std::placeholders::_1));
                    try {
                        std::thread downloader([this]{this->download_map();});
                        downloader.detach();
                    } catch (std::exception &ex) {
                            RCLCPP_ERROR_STREAM(get_logger(), "failed to create thread from " << ex.what());
                    }
                } else{
                    AreaList areas = read_arealist(arealist_path);
                    for (const Area& area : areas) {
                        // Check if the user entered pcd paths in addition to the arealist.txt file
                        if (pcd_paths.size() > 0) {
                            // Only load cells which the user specified
                            for (const std::string& path : pcd_paths) {
                                if (path == area.path)
                                    cache_arealist(area, downloaded_areas);
                            }
                        } else {
                            // The user did not specify any cells to load all the cells contained in the arealist.txt file
                            cache_arealist(area, downloaded_areas);
                        }
                    }
                }

                gnss_time = current_time = this->now();
            }
    }

    void PointsMapLoader::enqueue(const geometry_msgs::msg::Point& p)
    {
        std::unique_lock<std::mutex> lock(mtx_);
        queue_.push(p);
        cv_.notify_all();
    }

    void PointsMapLoader::enqueue_look_ahead(const geometry_msgs::msg::Point& p)
    {
        std::unique_lock<std::mutex> lock(mtx_);
        look_ahead_queue_.push(p);
        cv_.notify_all();
    }

    void PointsMapLoader::clear()
    {
        std::unique_lock<std::mutex> lock(mtx_);
        while (!queue_.empty())
            queue_.pop();
    }

    void PointsMapLoader::clear_look_ahead()
    {
        std::unique_lock<std::mutex> lock(mtx_);
        while (!look_ahead_queue_.empty())
            look_ahead_queue_.pop();
    }

    geometry_msgs::msg::Point PointsMapLoader::dequeue()
    {
        std::unique_lock<std::mutex> lock(mtx_);
        while (queue_.empty() && look_ahead_queue_.empty())
            cv_.wait(lock);
        if (!queue_.empty()) {
            geometry_msgs::msg::Point p = queue_.front();
            queue_.pop();
            return p;
        } else {
            geometry_msgs::msg::Point p = look_ahead_queue_.front();
            look_ahead_queue_.pop();
            return p;
        }
    }


    Tbl PointsMapLoader::read_csv(const std::string& path)
    {
        std::ifstream ifs(path.c_str());
        std::string line;
        Tbl ret;
        while (std::getline(ifs, line)) {
            std::istringstream iss(line);
            std::string col;
            std::vector<std::string> cols;
            while (std::getline(iss, col, ','))
                cols.push_back(col);
            ret.push_back(cols);
        }
        return ret;
    }

    void PointsMapLoader::write_csv(const std::string& path, const Tbl& tbl)
    {
        std::ofstream ofs(path.c_str());
        for (const std::vector<std::string>& cols : tbl) {
            std::string line;
            for (size_t i = 0; i < cols.size(); ++i) {
                if (i == 0)
                    line += cols[i];
                else
                    line += "," + cols[i];
            }
            ofs << line << std::endl;
        }
    }

    AreaList PointsMapLoader::read_arealist(const std::string& path)
    {
        Tbl tbl = read_csv(path);
        AreaList ret;
        for (const std::vector<std::string>& cols : tbl) {
            Area area;
            area.path = cols[0];
            area.x_min = std::stod(cols[1]);
            area.y_min = std::stod(cols[2]);
            area.z_min = std::stod(cols[3]);
            area.x_max = std::stod(cols[4]);
            area.y_max = std::stod(cols[5]);
            area.z_max = std::stod(cols[6]);
            ret.push_back(area);
        }
        return ret;
    }


    void PointsMapLoader::write_arealist(const std::string& path, const AreaList& areas)
    {
        Tbl tbl;
        for (const Area& area : areas) {
            std::vector<std::string> cols;
            cols.push_back(area.path);
            cols.push_back(std::to_string(area.x_min));
            cols.push_back(std::to_string(area.y_min));
            cols.push_back(std::to_string(area.z_min));
            cols.push_back(std::to_string(area.x_max));
            cols.push_back(std::to_string(area.y_max));
            cols.push_back(std::to_string(area.z_max));
            tbl.push_back(cols);
        }
        write_csv(path, tbl);
    }

    bool PointsMapLoader::is_downloaded(const std::string& path)
    {
        struct stat st;
        return (stat(path.c_str(), &st) == 0);
    }

    bool PointsMapLoader::is_in_area(double x, double y, const Area& area, double m)
    {
        return ((area.x_min - m) <= x && x <= (area.x_max + m) && (area.y_min - m) <= y && y <= (area.y_max + m));
    }

    std::string PointsMapLoader::create_location(int x, int y)
    {
        x -= x % ROUNDING_UNIT;
        y -= y % ROUNDING_UNIT;
        return ("data/map/" + std::to_string(y) + "/" + std::to_string(x) + "/pointcloud/");
    }

    void PointsMapLoader::cache_arealist(const Area& area, AreaList& areas)
    {
        for (const Area& a : areas) {
            if (a.path == area.path)
                return;
        }
        areas.push_back(area);
    }

    int PointsMapLoader::download(GetFile gf, const std::string& tmp, const std::string& loc, const std::string& filename)
    {
        std::string pathname;
        pathname += tmp;
        std::istringstream iss(loc);
        std::string col;
        while (std::getline(iss, col, '/')) {
            pathname += col + "/";
            mkdir(pathname.c_str(), 0755);
        }

        return gf.GetHTTPFile(loc + filename);
    }

    void PointsMapLoader::download_map()
    {
        while (true) {
            geometry_msgs::msg::Point p = dequeue();

            int x = static_cast<int>(p.x);
            int y = static_cast<int>(p.y);
            int x_min = static_cast<int>(p.x - margin);
            int y_min = static_cast<int>(p.y - margin);
            int x_max = static_cast<int>(p.x + margin);
            int y_max = static_cast<int>(p.y + margin);

            std::vector<std::string> locs;
            locs.push_back(create_location(x, y));
            locs.push_back(create_location(x_min, y_min));
            locs.push_back(create_location(x_min, y_max));
            locs.push_back(create_location(x_max, y_min));
            locs.push_back(create_location(x_max, y_max));
            for (const std::string& loc : locs) { // XXX better way?
                std::string arealist_path = TEMPORARY_DIRNAME + loc + AREALIST_FILENAME;

                bool cached = false;
                for (const std::string& path : cached_arealist_paths) {
                    if (path == arealist_path) {
                        cached = true;
                        break;
                    }
                }
                if (cached)
                    continue;

                AreaList areas;
                if (is_downloaded(arealist_path))
                    areas = read_arealist(arealist_path);
                else {
                    if (download(gf, TEMPORARY_DIRNAME, loc, AREALIST_FILENAME) != 0)
                        continue;
                    areas = read_arealist(arealist_path);
                    for (Area& area : areas)
                        area.path = TEMPORARY_DIRNAME + loc + basename(area.path.c_str());
                    write_arealist(arealist_path, areas);
                }
                for (const Area& area : areas)
                    cache_arealist(area, all_areas);
                cached_arealist_paths.push_back(arealist_path);
            }

            for (const Area& area : all_areas) {
                if (is_in_area(p.x, p.y, area, margin)) {
                    int x_area = static_cast<int>(area.x_max - MARGIN_UNIT);
                    int y_area = static_cast<int>(area.y_max - MARGIN_UNIT);
                    std::string loc = create_location(x_area, y_area);
                    if (is_downloaded(area.path) ||
                        download(gf, TEMPORARY_DIRNAME, loc, basename(area.path.c_str())) == 0) {
                        std::unique_lock<std::mutex> lock(downloaded_areas_mtx);
                        cache_arealist(area, downloaded_areas);
                    }
                }
            }
        }
    }

    
    sensor_msgs::msg::PointCloud2 PointsMapLoader::create_pcd(const geometry_msgs::msg::Point& p)
    {
        sensor_msgs::msg::PointCloud2 pcd, part;
        std::unique_lock<std::mutex> lock(downloaded_areas_mtx);
        for (const Area& area : downloaded_areas) {
            if (is_in_area(p.x, p.y, area, margin)) {
                if (pcd.width == 0)
                    pcl::io::loadPCDFile(area.path.c_str(), pcd);
                else {
                    pcl::io::loadPCDFile(area.path.c_str(), part);
                    pcd.width += part.width;
                    pcd.row_step += part.row_step;
                    pcd.data.insert(pcd.data.end(), part.data.begin(), part.data.end());
                }
            }
        }

        return pcd;
    }

    sensor_msgs::msg::PointCloud2 PointsMapLoader::create_pcd(const std::vector<std::string>& pcd_paths, int* ret_err = NULL)
    {
        sensor_msgs::msg::PointCloud2 pcd, part;
        for (const std::string& path : pcd_paths) {
            // Following outputs are used for progress bar of Runtime Manager.
            if (pcd.width == 0) {
                if (pcl::io::loadPCDFile(path.c_str(), pcd) == -1) {
                    std::cerr << "load failed " << path << std::endl;
                    if (ret_err) *ret_err = 1;
                }
            } else {
                if (pcl::io::loadPCDFile(path.c_str(), part) == -1) {
                    std::cerr << "load failed " << path << std::endl;
                    if (ret_err) *ret_err = 1;
                }
                pcd.width += part.width;
                pcd.row_step += part.row_step;
                pcd.data.insert(pcd.data.end(), part.data.begin(), part.data.end());
            }
            std::cerr << "load " << path << std::endl;
            if (!rclcpp::ok()) break;
        }

        return pcd;
    }

    void PointsMapLoader::publish_pcd(sensor_msgs::msg::PointCloud2 pcd, const int* errp = NULL)
    {
        RCLCPP_INFO_STREAM(get_logger(),"Entering Publish pcd");
        if (pcd.width != 0) {
            RCLCPP_INFO_STREAM(get_logger(),"Entering pcd width!= 0");
            pcd.header.frame_id = "map";
            pcd_pub->publish(pcd);

            if (errp == NULL || *errp == 0) {
                RCLCPP_INFO_STREAM(get_logger(),"Err == Null");
                stat_msg.data = true;
                stat_pub->publish(stat_msg);
            }
        }
    }

    void PointsMapLoader::publish_gnss_pcd(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        rclcpp::Time now = this->now();
        if (((now - current_time).seconds() * 1000) < fallback_rate)
            return;
        if (((now - gnss_time).seconds() * 1000) < update_rate)
            return;
        gnss_time = now;

        if (can_download)
            enqueue(msg->pose.position);

        publish_pcd(create_pcd(msg->pose.position));
    }

    void PointsMapLoader::publish_current_pcd(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
       	rclcpp::Time now = this->now();
        if (((now - current_time).seconds() * 1000) < update_rate)
            return;
        current_time = now;

        if (can_download)
            enqueue(msg->pose.position);

        publish_pcd(create_pcd(msg->pose.position)); 
    }

    void PointsMapLoader::publish_dragged_pcd(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        tf2_ros::Buffer buffer(this->get_clock());
        tf2_ros::TransformListener listener(buffer);
        geometry_msgs::msg::TransformStamped tf_geom;
        tf2::Transform transform;
        try {
            rclcpp::Time zero = this->now();
            // listener.waitForTransform("map", msg.header.frame_id, zero, );
            RCLCPP_WARN_STREAM(get_logger(), "Reaching before lookup Transform");
            tf_geom = buffer.lookupTransform("map", msg->header.frame_id, zero, rclcpp::Duration(10,0));
            RCLCPP_WARN_STREAM(get_logger(), "Reaching After lookup Transform");
        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR_STREAM(get_logger(),"failed to create transform from " << ex.what());
        }

        // ros1 implementation called getOrigin from returned transform
        // ros2 implementation fills getOrigin in geom msg: https://github.com/ros2/geometry2/blob/d1dc38b8a0e706fbd67a800a241fee950fce39f4/tf2/src/buffer_core.cpp#L636
        geometry_msgs::msg::Point p;
        p.x = msg->pose.pose.position.x + tf_geom.transform.translation.x;
        p.y = msg->pose.pose.position.y + tf_geom.transform.translation.y;

        if (can_download)
            enqueue(p);

        publish_pcd(create_pcd(p));
    }

    void PointsMapLoader::request_lookahead_download(const autoware_msgs::msg::LaneArray::SharedPtr msg)
    {
        clear_look_ahead();

        for (const autoware_msgs::msg::Lane& l : msg->lanes) {
            size_t end = l.waypoints.size() - 1;
            double distance = 0;
            double threshold = (MARGIN_UNIT / 2) + margin; // XXX better way?
            for (size_t i = 0; i <= end; ++i) {
                if (i == 0 || i == end) {
                    geometry_msgs::msg::Point p;
                    p.x = l.waypoints[i].pose.pose.position.x;
                    p.y = l.waypoints[i].pose.pose.position.y;
                    enqueue_look_ahead(p);
                } else {
                    geometry_msgs::msg::Point p1, p2;
                    p1.x = l.waypoints[i].pose.pose.position.x;
                    p1.y = l.waypoints[i].pose.pose.position.y;
                    p2.x = l.waypoints[i - 1].pose.pose.position.x;
                    p2.y = l.waypoints[i - 1].pose.pose.position.y;
                    distance += hypot(p2.x - p1.x, p2.y - p1.y);
                    if (distance > threshold) {
                        enqueue_look_ahead(p1);
                        distance = 0;
                    }
                }
            }
        }
    }
    

}  // namespace


#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(points_map_loader::PointsMapLoader)