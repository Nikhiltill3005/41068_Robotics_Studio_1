cmake_minimum_required(VERSION 3.8)
project(drone_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Create executables
add_executable(drone_teleop src/drone_teleop.cpp)
add_executable(husky_joy_teleop src/husky_joy_teleop.cpp)

# Specify libraries to link a library or executable target against
ament_target_dependencies(drone_teleop
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
)

ament_target_dependencies(husky_joy_teleop
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
)

# Install targets
install(TARGETS
  drone_teleop
  husky_joy_teleop
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

