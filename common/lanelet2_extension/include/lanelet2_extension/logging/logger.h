
#ifndef LANELET2_EXTENSION_LOGGER_H
#define LANELET2_EXTENSION_LOGGER_H

/*
 * Copyright (C) 2020-2021 LEIDOS.
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

#define LOGGER_TYPE_ROS_LOGGER 0
#define LOGGER_TYPE_STD_LOGGER 1

// If the logger type is not set then set it to the default

#if LANELET2_EXTENSION_LOGGER_TYPE == LOGGER_TYPE_STD_LOGGER // Logger type of 1 indicates usage of std::cout 
  
    #include <iostream> // Include std::cout and cerr

    #define LOG_DEBUG_STREAM(args) std::cout << args << std::endl;
    #define LOG_INFO_STREAM(args) std::cout << args << std::endl;
    #define LOG_WARN_STREAM(args) std::cout << args << std::endl;
    #define LOG_ERROR_STREAM(args) std::cerr << args << std::endl;
    #define LOG_FATAL_STREAM(args) std::cerr << args << std::endl;

#else // LOGGER_TYPE_ROS_LOGGER By default this library will use ros logging

    #if LANELET2_EXTENSION_ROS_VERSION == 2 // ROS2

        #include <rclcpp/logging.hpp> // Include rosconsole macros
        #include <rclcpp/logger.hpp>

        // NOTE: If logging from this package seems slow, consider storing the logger as a static object
        #define LOG_DEBUG_STREAM(args) RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lanelet2_extension"), args)
        #define LOG_INFO_STREAM(args) RCLCPP_INFO_STREAM(rclcpp::get_logger("lanelet2_extension"), args)
        #define LOG_WARN_STREAM(args) RCLCPP_WARN_STREAM(rclcpp::get_logger("lanelet2_extension"), args)
        #define LOG_ERROR_STREAM(args) RCLCPP_ERROR_STREAM(rclcpp::get_logger("lanelet2_extension"), args)
        #define LOG_FATAL_STREAM(args) RCLCPP_FATAL_STREAM(rclcpp::get_logger("lanelet2_extension"), args)

    #else // ROS1

        #include <ros/console.h> // Include rosconsole macros

        #define LOG_DEBUG_STREAM(args) ROS_DEBUG_STREAM(args)
        #define LOG_INFO_STREAM(args) ROS_INFO_STREAM(args)
        #define LOG_WARN_STREAM(args) ROS_WARN_STREAM(args)
        #define LOG_ERROR_STREAM(args) ROS_ERROR_STREAM(args)
        #define LOG_FATAL_STREAM(args) ROS_FATAL_STREAM(args)
    #endif

#endif


#endif  // LANELET2_EXTENSION_LOGGER_H