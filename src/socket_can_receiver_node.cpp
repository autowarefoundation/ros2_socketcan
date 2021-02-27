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
}

}  // namespace socketcan
}  // namespace drivers

#include <rclcpp_components/register_node_macro.hpp> //NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(drivers::socketcan::SocketCanReceiverNode)
