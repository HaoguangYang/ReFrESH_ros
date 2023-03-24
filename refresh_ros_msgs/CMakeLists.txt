project(refresh_ros_msgs)

find_package(ros_environment REQUIRED)

set(ROS_VERSION $ENV{ROS_VERSION})

file(GLOB ${PROJECT_NAME}_msg_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} msg/*)
file(GLOB ${PROJECT_NAME}_srv_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} srv/*)
file(GLOB ${PROJECT_NAME}_action_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} action/*)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(${ROS_VERSION} EQUAL 1)
  cmake_minimum_required(VERSION 3.5.1) # version on Ubuntu Trusty

  # Default to C++14
  if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
  endif()

  find_package(catkin REQUIRED
    COMPONENTS
    message_generation
    std_msgs
    actionlib_msgs
    actionlib
  )

  set(TEMP_LIST "")
  # replace outstanding Time message type definition
  # remove msg/, srv/, action/ from relative path
  foreach(MSG_FILE ${${PROJECT_NAME}_msg_files})
    execute_process(
      COMMAND /bin/bash -c "sed -i 's/builtin_interfaces\\/Time/time/g' ${MSG_FILE}"
      WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    )
    string(REPLACE "msg/" "" STRIPPED_MSG ${MSG_FILE})
    list(APPEND TEMP_LIST "${STRIPPED_MSG}")
  endforeach()
  add_message_files(
    FILES ${TEMP_LIST}
  )

  set(TEMP_LIST "")
  foreach(SRV_FILE ${${PROJECT_NAME}_srv_files})
    execute_process(
      COMMAND /bin/bash -c "sed -i 's/builtin_interfaces\\/Time/time/g' ${SRV_FILE}"
      WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    )
    string(REPLACE "srv/" "" STRIPPED_SRV ${SRV_FILE})
    list(APPEND TEMP_LIST "${STRIPPED_SRV}")
  endforeach()
  add_service_files(
    FILES ${TEMP_LIST}
  )

  set(TEMP_LIST "")
  foreach(ACT_FILE ${${PROJECT_NAME}_action_files})
    execute_process(
      COMMAND /bin/bash -c "sed -i 's/builtin_interfaces\\/Time/time/g' ${ACT_FILE}"
      WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    )
    string(REPLACE "action/" "" STRIPPED_ACT ${ACT_FILE})
    list(APPEND TEMP_LIST "${STRIPPED_ACT}")
  endforeach()
  add_action_files(
    FILES ${TEMP_LIST}
  )

  generate_messages(DEPENDENCIES std_msgs actionlib_msgs)

  catkin_package(
    CATKIN_DEPENDS message_runtime
  )

elseif(${ROS_VERSION} EQUAL 2)
  cmake_minimum_required(VERSION 3.16)

  if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
  endif()

  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
  endif()

  find_package(ament_cmake_auto REQUIRED)
  ament_auto_find_build_dependencies()

  # replace outstanding Time message type definition
  foreach(MSG_FILE ${${PROJECT_NAME}_msg_files})
    execute_process(
      COMMAND /bin/bash -c "sed -i 's/time /builtin_interfaces\\/Time /g' ${MSG_FILE}"
      WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    )
  endforeach()

  foreach(SRV_FILE ${${PROJECT_NAME}_srv_files})
    execute_process(
      COMMAND /bin/bash -c "sed -i 's/time /builtin_interfaces\\/Time /g' ${SRV_FILE}"
      WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    )
  endforeach()

  foreach(ACT_FILE ${${PROJECT_NAME}_action_files})
    execute_process(
      COMMAND /bin/bash -c "sed -i 's/time /builtin_interfaces\\/Time /g' ${ACT_FILE}"
      WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    )
  endforeach()

  # message generation
  find_package(rosidl_default_generators REQUIRED)
  rosidl_generate_interfaces(
    ${PROJECT_NAME}
    ${${PROJECT_NAME}_msg_files}
    ${${PROJECT_NAME}_srv_files}
    ${${PROJECT_NAME}_action_files}
    DEPENDENCIES ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS}
    ADD_LINTER_TESTS
  )

  # TESTS
  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
  endif()

  # INSTALL
  ament_auto_package()
endif()
