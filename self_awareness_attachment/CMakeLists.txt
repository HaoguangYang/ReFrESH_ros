cmake_minimum_required(VERSION 3.16)
project(self_awareness_attachment)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# setup targets
# generate ros2 node self awareness attachment
ament_auto_add_library(${PROJECT_NAME}_lib SHARED
  src/refresh_self_awareness_attachment.cpp
)
rclcpp_components_register_node(${PROJECT_NAME}_lib
  PLUGIN "ReFRESH::ROS_NodeSelfAwarenessImpl"
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_auto_add_library(modular_redundancy_lib SHARED
  src/modular_redundancy.cpp
)
rclcpp_components_register_node(modular_redundancy_lib
  PLUGIN "ReFRESH::ROS_ModularRedundancyNode"
  EXECUTABLE modular_redundancy_node
)

ament_auto_add_library(${PROJECT_NAME}_plugins SHARED
  src/msg_quality_attr_plugins/age_attr_generic.cpp
  src/msg_quality_attr_plugins/age_attr_builtin_interfaces_msg_time.cpp
  src/msg_quality_attr_plugins/age_attr_sensor_msgs_msg_imu.cpp
  src/msg_quality_attr_plugins/age_attr_sensor_msgs_msg_nav_sat_fix.cpp
  src/msg_quality_attr_plugins/age_attr_std_msgs_msg_header.cpp
  src/msg_quality_attr_plugins/age_attr_nav_msgs_msg_odometry.cpp
  src/msg_quality_attr_plugins/stdev_attr_generic.cpp
  src/msg_quality_attr_plugins/stdev_attr_geometry_msgs_msg_pose_with_covariance.cpp
  src/msg_quality_attr_plugins/stdev_attr_geometry_msgs_msg_pose_with_covariance_stamped.cpp
  src/msg_quality_attr_plugins/stdev_attr_sensor_msgs_msg_imu.cpp
  src/msg_quality_attr_plugins/stdev_attr_nav_msgs_msg_odometry.cpp
  src/msg_quality_attr_plugins/field_value_generic.cpp
)
pluginlib_export_plugin_description_file(self_awareness_attachment plugins.xml)

# TESTS
if(AMENT_ENABLE_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# INSTALL
# Ament package generation and installing. Export information to downstream packages
ament_auto_package(
  CONFIG_EXTRAS "${PROJECT_NAME}-extras.cmake"
)
