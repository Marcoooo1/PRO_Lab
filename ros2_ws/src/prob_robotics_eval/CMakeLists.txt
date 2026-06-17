cmake_minimum_required(VERSION 3.8)
project(prob_robotics_eval)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --------------------------------------------------------------------------
# Dependencies
# --------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# --------------------------------------------------------------------------
# auto_driver
# --------------------------------------------------------------------------
add_executable(auto_driver src/auto_driver.cpp)
ament_target_dependencies(auto_driver
  rclcpp
  geometry_msgs
)

# --------------------------------------------------------------------------
# trajectory_logger
# --------------------------------------------------------------------------
add_executable(trajectory_logger src/trajectory_logger.cpp)
ament_target_dependencies(trajectory_logger
  rclcpp
  nav_msgs
  geometry_msgs
)

# --------------------------------------------------------------------------
# Install
# --------------------------------------------------------------------------
install(TARGETS
  auto_driver
  trajectory_logger
  DESTINATION lib/${PROJECT_NAME}
)

# evaluate.py bleibt als Offline-Script (kein ROS-Node)
install(PROGRAMS
  scripts/evaluate.py
  DESTINATION lib/${PROJECT_NAME}
)

# --------------------------------------------------------------------------
# Tests
# --------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
