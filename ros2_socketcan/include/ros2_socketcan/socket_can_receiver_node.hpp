// Copyright 2021 the Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#ifndef ROS2_SOCKETCAN__SOCKET_CAN_RECEIVER_NODE_HPP_
#define ROS2_SOCKETCAN__SOCKET_CAN_RECEIVER_NODE_HPP_

#include <memory>
#include <thread>
#include <string>
#include <vector>

#include "ros2_socketcan/visibility_control.hpp"
#include "ros2_socketcan/socket_can_receiver.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"
#include "can_msgs/msg/frame.hpp"
#include "ros2_socketcan_msgs/msg/fd_frame.hpp"
#include "lifecycle_msgs/msg/state.hpp"

namespace lc = rclcpp_lifecycle;
using LNI = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;

namespace drivers
{
namespace socketcan
{
/// \brief SocketCanReceiverNode class which can pass messages
/// from CAN hardware or virtual channels
class SOCKETCAN_PUBLIC SocketCanReceiverNode final
  : public lc::LifecycleNode
{
public:
  /// \brief Default constructor
  explicit SocketCanReceiverNode(rclcpp::NodeOptions options);

  /// \brief Callback from transition to "configuring" state.
  /// \param[in] state The current state that the node is in.
  LNI::CallbackReturn on_configure(const lc::State & state) override;

  /// \brief Callback from transition to "activating" state.
  /// \param[in] state The current state that the node is in.
  LNI::CallbackReturn on_activate(const lc::State & state) override;

  /// \brief Callback from transition to "deactivating" state.
  /// \param[in] state The current state that the node is in.
  LNI::CallbackReturn on_deactivate(const lc::State & state) override;

  /// \brief Callback from transition to "unconfigured" state.
  /// \param[in] state The current state that the node is in.
  LNI::CallbackReturn on_cleanup(const lc::State & state) override;

  /// \brief Callback from transition to "shutdown" state.
  /// \param[in] state The current state that the node is in.
  LNI::CallbackReturn on_shutdown(const lc::State & state) override;

  /// \brief Callback for reading from hardware interface on timer tick.
  void receive();

private:
  std::string interface_;
  std::shared_ptr<lc::LifecyclePublisher<can_msgs::msg::Frame>> frames_pub_;
  std::shared_ptr<lc::LifecyclePublisher<ros2_socketcan_msgs::msg::FdFrame>> fd_frames_pub_;
  std::unique_ptr<SocketCanReceiver> receiver_;
  std::unique_ptr<std::thread> receiver_thread_;
  std::chrono::nanoseconds interval_ns_;
  bool enable_fd_;
  bool use_bus_time_;
  bool enable_loopback_;
};
}  // namespace socketcan
}  // namespace drivers

#endif  // ROS2_SOCKETCAN__SOCKET_CAN_RECEIVER_NODE_HPP_
