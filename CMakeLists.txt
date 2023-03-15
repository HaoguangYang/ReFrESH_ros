cmake_minimum_required(VERSION 3.16)
project(refresh_ros2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# setup targets
# generate library
set(TARGET_LIB_NAME ${PROJECT_NAME}_lib)
ament_auto_add_library(${TARGET_LIB_NAME}
  src/bt_refresh_control_node.cpp
  src/bt_refresh_module_node.cpp
  src/bt_refresh_ros_action_node.cpp
  include/refresh_ros2/bt_action_node.hpp
  include/refresh_ros2/bt_service_node.hpp
  include/refresh_ros2/bt_subscriber_node.hpp
  include/refresh_ros2/bt_refresh_control_node.hpp
  include/refresh_ros2/bt_refresh_module_node.hpp
  include/refresh_ros2/bt_refresh_ros_action_node.hpp
)

# generate bt task engine node
ament_auto_add_library(${PROJECT_NAME}_bt_task_engine SHARED
  src/bt_task_engine.cpp
)
rclcpp_components_register_node(${PROJECT_NAME}_bt_task_engine
  PLUGIN "ReFRESH::BehaviorTreeTaskEngineNode"
  EXECUTABLE ${PROJECT_NAME}_bt_task_engine_node
)

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

  ament_auto_add_executable(sleep_server test/sleep_server.cpp)
  
  ament_auto_add_executable(test_client test/test_client.cpp)
  
  #ament_auto_add_executable(test_server test/test_server.cpp)
endif()

# INSTALL
# Ament package generation and installing. Export information to downstream packages
ament_auto_package(
  CONFIG_EXTRAS "${PROJECT_NAME}-extras.cmake"
)
