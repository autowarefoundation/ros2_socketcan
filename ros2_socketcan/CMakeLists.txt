cmake_minimum_required(VERSION 3.5)
project(ros2_socketcan)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/socket_can_common.cpp
  src/socket_can_id.cpp
  src/socket_can_receiver.cpp
  src/socket_can_sender.cpp)

ament_auto_add_library(socket_can_receiver_node SHARED
  src/socket_can_receiver_node.cpp
)

target_link_libraries(socket_can_receiver_node
  ${PROJECT_NAME}
)

rclcpp_components_register_node(socket_can_receiver_node
  PLUGIN "drivers::socketcan::SocketCanReceiverNode"
  EXECUTABLE socket_can_receiver_node_exe
)

ament_auto_add_library(socket_can_sender_node SHARED
  src/socket_can_sender_node.cpp
)

target_link_libraries(socket_can_sender_node
  ${PROJECT_NAME}
)

rclcpp_components_register_node(socket_can_sender_node
  PLUGIN "drivers::socketcan::SocketCanSenderNode"
  EXECUTABLE socket_can_sender_node_exe
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # TODO(c.ho) Make this into a pytest
  ament_add_gtest(${PROJECT_NAME}_test
    test/gtest_main.cpp
    test/receiver.cpp
    test/sanity_checks.cpp)
  target_include_directories(${PROJECT_NAME}_test PUBLIC include)
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
)
