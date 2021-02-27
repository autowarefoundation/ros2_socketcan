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

#include <ros2_socketcan/socket_can_receiver_node.hpp>

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>

namespace drivers
{
namespace socketcan
{

SocketCanReceiverNode::SocketCanReceiverNode(const rclcpp::NodeOptions & options)
: Node("socket_can_receiver", options),
  m_data_buffer{}
{
  m_receiver = std::make_unique<SocketCanReceiver>(
    declare_parameter(
      "interface_name").get<std::string>());
  m_read_interval = std::chrono::nanoseconds(
    declare_parameter("polling_interval_ns").get<int64_t>());

  m_can_pub = create_publisher<can_msgs::msg::Frame>("can_received", rclcpp::QoS{50});

  m_receiver_thread = std::make_unique<std::thread>(&SocketCanReceiverNode::receive, this);
}

SocketCanReceiverNode::~SocketCanReceiverNode()
{
  if (m_receiver_thread->joinable()) {
    m_receiver_thread->join();
  }
}

void SocketCanReceiverNode::receive()
{
  CanId can_id;

  while (rclcpp::ok()) {
    can_id = m_receiver->receive(&m_data_buffer[0], m_read_interval);
    can_msgs::msg::Frame can_frame;
    can_frame.id = can_id.identifier();
    can_frame.is_rtr = (can_id.frame_type() == FrameType::REMOTE);
    can_frame.is_extended = can_id.is_extended();
    can_frame.is_error = (can_id.frame_type() == FrameType::ERROR);
    can_frame.dlc = can_id.length();
    std::copy(m_data_buffer.begin(), m_data_buffer.end(), can_frame.data.begin());

    m_can_pub->publish(can_frame);
  }
}

}  // namespace socketcan
}  // namespace drivers

#include <rclcpp_components/register_node_macro.hpp> //NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(drivers::socketcan::SocketCanReceiverNode)
