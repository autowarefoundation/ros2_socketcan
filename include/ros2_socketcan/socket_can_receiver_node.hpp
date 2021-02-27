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

#ifndef ROS2_SOCKETCAN__SOCKET_CAN_RECEIVER_NODE_HPP_
#define ROS2_SOCKETCAN__SOCKET_CAN_RECEIVER_NODE_HPP_

#include <ros2_socketcan/socket_can_receiver.hpp>
#include <ros2_socketcan/visibility_control.hpp>

#include <rclcpp/rclcpp.hpp>

#include <array>
#include <memory>

namespace drivers
{
namespace socketcan
{
class SOCKETCAN_PUBLIC SocketCanReceiverNode : public rclcpp::Node
{
public:
  explicit SocketCanReceiverNode(const rclcpp::NodeOptions & options);

private:
  std::array<uint8_t, 8> m_data_buffer;
  std::unique_ptr<SocketCanReceiver> m_receiver;
};
}  // namespace socketcan
}  // namespace drivers

#endif  // ROS2_SOCKETCAN__SOCKET_CAN_RECEIVER_NODE_HPP_
