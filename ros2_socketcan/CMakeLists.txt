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
endif()

# ##################################################################
# ADDED: Install and Export commands to make this a valid dependency
# ##################################################################

# 1. Install all targets (libraries and executables)
install(TARGETS
    ${PROJECT_NAME}
    socket_can_receiver_node
    socket_can_sender_node
    socket_can_receiver_node_exe
    socket_can_sender_node_exe
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  COMPONENT ${PROJECT_NAME}
)

# 2. Install the public headers
install(
  DIRECTORY ros2_socketcan/include/
  DESTINATION include
)

# 3. Export include directories for downstream packages
ament_export_include_directories(
  ros2_socketcan/include
)

# 4. Export libraries for downstream packages
ament_export_libraries(
  ${PROJECT_NAME}
)

# This command must come AFTER all other install/export commands
ament_auto_package(INSTALL_TO_SHARE
  launch
)