cmake_minimum_required(VERSION 3.5)
project(twist_gate)

find_package(carma_cmake_common REQUIRED)
carma_check_ros_version(2)
carma_package()

## Find dependencies using ament auto
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Name build targets
set(node_exec twist_gate_node_exec)
set(node_lib twist_gate_node)

# Includes
include_directories(
  include
)

# Build
ament_auto_add_library(${node_lib} SHARED
        src/twist_gate.cpp
)

ament_auto_add_executable(${node_exec} 
        src/twist_gate_node.cpp 
)

# Register component
rclcpp_components_register_nodes(${node_lib} "TwistGate")

# All locally created targets will need to be manually linked
# ament auto will handle linking of external dependencies
target_link_libraries(${node_exec}
        ${node_lib}
)

# Testing
if(BUILD_TESTING)  

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies() # This populates the ${${PROJECT_NAME}_FOUND_TEST_DEPENDS} variable

  ament_add_gtest(test_twist_gate test/src/test_twist_gate.cpp)

  ament_target_dependencies(test_twist_gate ${${PROJECT_NAME}_FOUND_TEST_DEPENDS})

  target_link_libraries(test_twist_gate ${node_lib})

endif()

# Install
ament_auto_package()