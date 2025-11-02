cmake_minimum_required(VERSION 3.8)
project(41068_ignition_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(ros_ign_interfaces REQUIRED)

install(
  DIRECTORY config launch models urdf worlds urdf_drone scripts
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/fire_detection_node.py
  scripts/odom_tf_publisher.py
  scripts/positioning_data.py
  scripts/navigation_gui.py
  scripts/cmd_vel_relay.py
  scripts/autonomous_firefighter_node.py
  scripts/steamdeck_gui.py
  scripts/nav_goal_gui.py
  scripts/test_dynamic_nav.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
