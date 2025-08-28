cmake_minimum_required(VERSION 3.8)
project(firefighting_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include directories
include_directories(include)

# Create executables for each node
add_executable(teleop_control_node src/teleop_control_node.cpp)
ament_target_dependencies(teleop_control_node rclcpp std_msgs geometry_msgs ackermann_msgs)

add_executable(ugv_movement_node src/ugv_movement_node.cpp)
ament_target_dependencies(ugv_movement_node rclcpp std_msgs geometry_msgs ackermann_msgs nav_msgs tf2 tf2_ros)

add_executable(environment_mapping_node src/environment_mapping_node.cpp)
ament_target_dependencies(environment_mapping_node rclcpp std_msgs sensor_msgs nav_msgs tf2 tf2_ros)

add_executable(autonomous_navigation_node src/autonomous_navigation_node.cpp)
ament_target_dependencies(autonomous_navigation_node rclcpp std_msgs geometry_msgs nav_msgs tf2 tf2_ros)



# Install executables
install(TARGETS
  teleop_control_node
  ugv_movement_node
  environment_mapping_node
  autonomous_navigation_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  config
  models
  worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install Python modules
install(DIRECTORY src
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
