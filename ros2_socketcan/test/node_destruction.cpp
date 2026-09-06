// Copyright 2026 the Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>
#include <unistd.h>

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_socketcan/socket_can_receiver.hpp"
#include "ros2_socketcan/socket_can_receiver_node.hpp"

using namespace std::chrono_literals;

// The receiver thread must not outlive the node object, and a lifecycle
// cleanup must end it while the node still exists. Each scenario runs in a
// death test child, so that an abort or a hang in one scenario leaves the
// other verdicts intact, and the parent still writes the gtest result file.

namespace
{
constexpr auto kInterface = "vcan0";
constexpr unsigned int kDeadlineSeconds = 10U;

// The child prints this line right before it exits with 0. The parent
// requires it, so that a child that never reaches the scenario cannot pass.
constexpr auto kSuccessMessage = "node destroyed, receiver thread gone";

// SocketCanReceiverNode::on_configure() names the receiver thread rx:<interface>.
std::string receiver_thread_name()
{
  return std::string("rx:") + kInterface;
}

bool vcan0_available()
{
  try {
    drivers::socketcan::SocketCanReceiver receiver{kInterface};
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

int threads_named(const std::string & name)
{
  int count = 0;
  for (const auto & task : std::filesystem::directory_iterator("/proc/self/task")) {
    std::ifstream comm(task.path() / "comm");
    std::string line;
    std::getline(comm, line);
    if (line == name) {
      ++count;
    }
  }
  return count;
}

bool wait_for_thread_count(const std::string & name, int expected)
{
  const auto deadline = std::chrono::steady_clock::now() + 1s;
  while (threads_named(name) != expected) {
    if (std::chrono::steady_clock::now() > deadline) {
      return false;
    }
    std::this_thread::sleep_for(10ms);
  }
  return true;
}

// Ends the child with a message and a non-zero exit code that the parent reports.
void require(bool condition, const char * message, int exit_code)
{
  if (!condition) {
    std::cerr << message << "\n";
    std::exit(exit_code);
  }
}

std::unique_ptr<drivers::socketcan::SocketCanReceiverNode> make_node(bool enable_can_fd)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"interface", kInterface}, {"enable_can_fd", enable_can_fd}});
  return std::make_unique<drivers::socketcan::SocketCanReceiverNode>(std::move(options));
}

void configure_and_activate(drivers::socketcan::SocketCanReceiverNode & node)
{
  require(
    node.configure().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    "configure failed", 2);
  require(
    node.activate().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    "activate failed", 2);
}

void exit_after_success()
{
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  std::cerr << kSuccessMessage << "\n";
  std::exit(0);
}

// Destroys a configured and activated node, after rclcpp::shutdown() or with
// the context up. Exit code 0 means that the destructor returned, the receiver
// thread is gone, and the context is in the state the scenario expects.
void destroy_node(bool shutdown_before_destruction, bool enable_can_fd)
{
  alarm(kDeadlineSeconds);
  rclcpp::init(0, nullptr);

  auto node = make_node(enable_can_fd);
  configure_and_activate(*node);
  require(wait_for_thread_count(receiver_thread_name(), 1), "receiver thread not found", 2);

  if (shutdown_before_destruction) {
    rclcpp::shutdown();
  }
  node.reset();

  require(
    wait_for_thread_count(receiver_thread_name(), 0),
    "receiver thread still runs after node destruction", 3);
  require(
    shutdown_before_destruction || rclcpp::ok(),
    "node destruction shut the context down", 4);
  exit_after_success();
}

// Runs a cleanup transition, configures the node again, and then destroys it
// with the context up. The receiver thread must end on cleanup and must run
// again after the second configure.
void cleanup_and_reconfigure_node()
{
  alarm(kDeadlineSeconds);
  rclcpp::init(0, nullptr);

  auto node = make_node(false);
  configure_and_activate(*node);
  require(wait_for_thread_count(receiver_thread_name(), 1), "receiver thread not found", 2);

  require(
    node->deactivate().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    "deactivate failed", 2);
  require(
    node->cleanup().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
    "cleanup failed", 2);
  require(
    wait_for_thread_count(receiver_thread_name(), 0),
    "receiver thread still runs after cleanup", 3);

  configure_and_activate(*node);
  require(
    wait_for_thread_count(receiver_thread_name(), 1),
    "receiver thread not found after the second configure", 5);
  std::this_thread::sleep_for(200ms);
  require(
    threads_named(receiver_thread_name()) == 1,
    "receiver thread ended after the second configure", 5);

  node.reset();
  require(
    wait_for_thread_count(receiver_thread_name(), 0),
    "receiver thread still runs after node destruction", 3);
  require(rclcpp::ok(), "node destruction shut the context down", 4);
  exit_after_success();
}

class SocketCanReceiverNodeDestruction : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!vcan0_available()) {
      if (std::getenv("ROS2_SOCKETCAN_REQUIRE_VCAN0") != nullptr) {
        FAIL() << "vcan0 is required by ROS2_SOCKETCAN_REQUIRE_VCAN0 but not available";
      }
      GTEST_SKIP() << "vcan0 is not available";
    }
    ::testing::GTEST_FLAG(death_test_style) = "threadsafe";
  }
};
}  // namespace

// On plain shutdown, for example SIGINT, the context goes down first. Then the
// node goes out of scope without a lifecycle transition. The destructor of a
// joinable std::thread calls std::terminate(), so this destruction must join
// the thread.
TEST_F(SocketCanReceiverNodeDestruction, completes_after_shutdown)
{
  EXPECT_EXIT(destroy_node(true, false), ::testing::ExitedWithCode(0), kSuccessMessage);
}

TEST_F(SocketCanReceiverNodeDestruction, completes_after_shutdown_with_can_fd)
{
  EXPECT_EXIT(destroy_node(true, true), ::testing::ExitedWithCode(0), kSuccessMessage);
}

// On a component unload, the context is still up and the node is destroyed
// from the ACTIVE state without a transition. A join with no stop signal then
// blocks forever, so destruction must stop the thread and must leave the
// context up. A destructor that detaches the thread instead leaves it to run
// on the destroyed node, and the child crashes on that use.
TEST_F(SocketCanReceiverNodeDestruction, completes_while_context_is_up)
{
  EXPECT_EXIT(destroy_node(false, false), ::testing::ExitedWithCode(0), kSuccessMessage);
}

TEST_F(SocketCanReceiverNodeDestruction, completes_while_context_is_up_with_can_fd)
{
  EXPECT_EXIT(destroy_node(false, true), ::testing::ExitedWithCode(0), kSuccessMessage);
}

// A cleanup transition runs on_cleanup() while the context is up, so it needs
// the same stop signal. A second configure must start a new thread, so the
// stop signal must not stay set.
TEST_F(SocketCanReceiverNodeDestruction, completes_after_cleanup_and_reconfigure)
{
  EXPECT_EXIT(cleanup_and_reconfigure_node(), ::testing::ExitedWithCode(0), kSuccessMessage);
}
