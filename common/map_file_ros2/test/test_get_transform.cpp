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


#include <gtest/gtest.h>
#include <map_param_loader.hpp>


TEST(GetTransformTest, testECEFPoints)
{
    //Example frames
    std::string map_frame = "+proj=tmerc +lat_0=38.95197911150576 +lon_0=-77.14835128349988 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m +no_defs";

    rclcpp::NodeOptions options;
    auto worker_node = std::make_shared<map_param_loader::MapParamLoader>(options);
    worker_node->configure(); //Call configure state transition
    worker_node->activate();  //Call activate state transition to get not read for runtime

    //Points are calculated using cs2cs command line function of proj library.
    tf2::Transform tf = worker_node->getTransform(map_frame);
    tf2::Vector3 origin = tf(tf2::Vector3{0,0,0});
    ASSERT_FLOAT_EQ (origin[0],1104726.07);
    ASSERT_FLOAT_EQ (origin[1],-4842261.44);
    ASSERT_FLOAT_EQ (origin[2],3988172.62);

    tf2::Vector3 e1 = tf(tf2::Vector3{1,0,0});
    ASSERT_FLOAT_EQ (e1[0],1104727.04);
    ASSERT_FLOAT_EQ (e1[1],-4842261.22);
    ASSERT_FLOAT_EQ (e1[2],3988172.62);

    tf2::Vector3 random = tf(tf2::Vector3{4,3,5});
    ASSERT_FLOAT_EQ (random[0],1104730.41);
    ASSERT_FLOAT_EQ (random[1],-4842262.50);
    ASSERT_FLOAT_EQ (random[2],3988178.10);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);

    //Initialize ROS
    rclcpp::init(argc, argv);

    bool success = RUN_ALL_TESTS();

    //shutdown ROS
    rclcpp::shutdown();

    return success;
}

