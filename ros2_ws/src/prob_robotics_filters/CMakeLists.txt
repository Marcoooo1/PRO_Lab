cmake_minimum_required(VERSION 3.8)
project(prob_robotics_filters)

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
find_package(visualization_msgs REQUIRED)

include_directories(include)

# --------------------------------------------------------------------------
# kf_node
# --------------------------------------------------------------------------
add_executable(kf_node src/kf_node.cpp)
ament_target_dependencies(kf_node rclcpp nav_msgs geometry_msgs)

# --------------------------------------------------------------------------
# ekf_node
# --------------------------------------------------------------------------
add_executable(ekf_node src/ekf_node.cpp)
ament_target_dependencies(ekf_node rclcpp nav_msgs geometry_msgs)

# --------------------------------------------------------------------------
# pf_node
# --------------------------------------------------------------------------
add_executable(pf_node src/pf_node.cpp)
ament_target_dependencies(pf_node rclcpp nav_msgs geometry_msgs)

# --------------------------------------------------------------------------
# landmark_detector
# --------------------------------------------------------------------------
add_executable(landmark_detector src/landmark_detector.cpp)
ament_target_dependencies(landmark_detector
  rclcpp
  nav_msgs
  geometry_msgs
  visualization_msgs
)

# --------------------------------------------------------------------------
# Install
# --------------------------------------------------------------------------
install(TARGETS
  kf_node
  ekf_node
  pf_node
  landmark_detector
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
