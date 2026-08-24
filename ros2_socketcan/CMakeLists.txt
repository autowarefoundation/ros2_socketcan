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

# if ENABLE_AGNOCAST environment variable is set to 1, enable agnocast components
# and define a compile definition to conditionally compile code that depends on agnocast
set(ENABLE_AGNOCAST_DEFAULT "$ENV{ENABLE_AGNOCAST}")
if(ENABLE_AGNOCAST_DEFAULT STREQUAL "1")
  set(ENABLE_AGNOCAST_DEFAULT "ON")
else()
  # default to OFF when the environment variable is unset or not equal to "1"
  set(ENABLE_AGNOCAST_DEFAULT "OFF")
endif()

option(ENABLE_AGNOCAST "Enable Agnocast components" ${ENABLE_AGNOCAST_DEFAULT})

if(ENABLE_AGNOCAST)
  find_package(agnocast_components REQUIRED)
  find_package(agnocast_cie_thread_configurator REQUIRED)
endif()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/socket_can_common.cpp
  src/socket_can_id.cpp
  src/socket_can_receiver.cpp
  src/socket_can_sender.cpp)

ament_auto_add_library(socket_can_receiver_node SHARED
  src/socket_can_receiver_node.cpp
)

if(ENABLE_AGNOCAST)
  target_compile_definitions(socket_can_receiver_node PRIVATE USE_AGNOCAST_ENABLED)
  ament_target_dependencies(socket_can_receiver_node
    agnocast_cie_thread_configurator
  )
endif()

target_link_libraries(socket_can_receiver_node
  ${PROJECT_NAME}
)

if(ENABLE_AGNOCAST)
  agnocast_components_register_node(socket_can_receiver_node
    PLUGIN "drivers::socketcan::SocketCanReceiverNode"
    EXECUTABLE socket_can_receiver_node_exe
    EXECUTOR CallbackIsolatedAgnocastExecutor
  )
else()
  rclcpp_components_register_node(socket_can_receiver_node
    PLUGIN "drivers::socketcan::SocketCanReceiverNode"
    EXECUTABLE socket_can_receiver_node_exe
  )
endif()

ament_auto_add_library(socket_can_sender_node SHARED
  src/socket_can_sender_node.cpp
)

target_link_libraries(socket_can_sender_node
  ${PROJECT_NAME}
)

if(ENABLE_AGNOCAST)
  agnocast_components_register_node(socket_can_sender_node
    PLUGIN "drivers::socketcan::SocketCanSenderNode"
    EXECUTABLE socket_can_sender_node_exe
    EXECUTOR CallbackIsolatedAgnocastExecutor
  )
else()
  rclcpp_components_register_node(socket_can_sender_node
    PLUGIN "drivers::socketcan::SocketCanSenderNode"
    EXECUTABLE socket_can_sender_node_exe
  )
endif()

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

  ament_add_gtest(${PROJECT_NAME}_node_destruction_test
    test/gtest_main.cpp
    test/node_destruction.cpp)
  target_compile_features(${PROJECT_NAME}_node_destruction_test PRIVATE cxx_std_17)
  target_link_libraries(${PROJECT_NAME}_node_destruction_test socket_can_receiver_node)
endif()

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
    launch
)
