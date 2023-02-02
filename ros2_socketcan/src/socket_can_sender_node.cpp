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

#include "ros2_socketcan/socket_can_sender_node.hpp"
#include "ros2_socketcan/socket_can_common.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <utility>

namespace lc = rclcpp_lifecycle;
using LNI = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;
using lifecycle_msgs::msg::State;

namespace drivers
{
namespace socketcan
{
SocketCanSenderNode::SocketCanSenderNode(rclcpp::NodeOptions options)
: lc::LifecycleNode("socket_can_sender_node", options)
{
  interface_ = this->declare_parameter("interface", "can0");
  enable_fd_ = this->declare_parameter("enable_can_fd", false);
  double timeout_sec = this->declare_parameter("timeout_sec", 0.01);
  timeout_ns_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(timeout_sec));

  RCLCPP_INFO(this->get_logger(), "interface: %s", interface_.c_str());
  RCLCPP_INFO(this->get_logger(), "can fd enabled: %s", enable_fd_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "timeout(s): %f", timeout_sec);
}

LNI::CallbackReturn SocketCanSenderNode::on_configure(const lc::State & state)
{
  (void)state;

  try {
    sender_ = std::make_unique<SocketCanSender>(interface_, enable_fd_);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      this->get_logger(), "Error opening CAN sender: %s - %s",
      interface_.c_str(), ex.what());
    return LNI::CallbackReturn::FAILURE;
  }

  RCLCPP_DEBUG(this->get_logger(), "Sender successfully configured.");

  if (!enable_fd_) {
    frames_sub_ = this->create_subscription<can_msgs::msg::Frame>(
      "to_can_bus", 500, std::bind(&SocketCanSenderNode::on_frame, this, std::placeholders::_1));
  } else {
    fd_frames_sub_ = this->create_subscription<ros2_socketcan_msgs::msg::FdFrame>(
      "to_can_bus_fd", 500, std::bind(
        &SocketCanSenderNode::on_fd_frame, this,
        std::placeholders::_1));
  }

  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn SocketCanSenderNode::on_activate(const lc::State & state)
{
  (void)state;
  RCLCPP_DEBUG(this->get_logger(), "Sender activated.");
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn SocketCanSenderNode::on_deactivate(const lc::State & state)
{
  (void)state;
  RCLCPP_DEBUG(this->get_logger(), "Sender deactivated.");
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn SocketCanSenderNode::on_cleanup(const lc::State & state)
{
  (void)state;

  if (!enable_fd_) {
    frames_sub_.reset();
  } else {
    fd_frames_sub_.reset();
  }

  RCLCPP_DEBUG(this->get_logger(), "Sender cleaned up.");
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn SocketCanSenderNode::on_shutdown(const lc::State & state)
{
  (void)state;
  RCLCPP_DEBUG(this->get_logger(), "Sender shutting down.");
  return LNI::CallbackReturn::SUCCESS;
}

void SocketCanSenderNode::on_frame(const can_msgs::msg::Frame::SharedPtr msg)
{
  if (this->get_current_state().id() == State::PRIMARY_STATE_ACTIVE) {
    FrameType type;
    if (msg->is_rtr) {
      type = FrameType::REMOTE;
    } else if (msg->is_error) {
      type = FrameType::ERROR;
    } else {
      type = FrameType::DATA;
    }

    CanId send_id = msg->is_extended ? CanId(msg->id, 0, type, ExtendedFrame) :
      CanId(msg->id, 0, type, StandardFrame);
    try {
      sender_->send(msg->data.data(), msg->dlc, send_id, timeout_ns_);
    } catch (const std::exception & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Error sending CAN message: %s - %s",
        interface_.c_str(), ex.what());
      return;
    }
  }
}

void SocketCanSenderNode::on_fd_frame(const ros2_socketcan_msgs::msg::FdFrame::SharedPtr msg)
{
  if (this->get_current_state().id() == State::PRIMARY_STATE_ACTIVE) {
    FrameType type;
    if (msg->is_error) {
      type = FrameType::ERROR;
    } else {
      type = FrameType::DATA;
    }

    CanId send_id = msg->is_extended ? CanId(msg->id, 0, type, ExtendedFrame) :
      CanId(msg->id, 0, type, StandardFrame);
    try {
      sender_->send_fd(msg->data.data<void>(), msg->len, send_id, timeout_ns_);
    } catch (const std::exception & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Error sending CAN message: %s - %s",
        interface_.c_str(), ex.what());
      return;
    }
  }
}

}  // namespace socketcan
}  // namespace drivers

RCLCPP_COMPONENTS_REGISTER_NODE(drivers::socketcan::SocketCanSenderNode)
