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

#include <gtest/gtest.h>

#include <chrono>
#include <memory>

#include "ros2_socketcan/socket_can_receiver.hpp"
#include "ros2_socketcan/socket_can_sender.hpp"

using drivers::socketcan::SocketCanReceiver;
using drivers::socketcan::SocketCanSender;
using drivers::socketcan::CanId;
using drivers::socketcan::StandardFrame;
using drivers::socketcan::ExtendedFrame;
using drivers::socketcan::FrameType;

// Requires elevated kernel permissions normal containers can't provide
class DISABLED_receiver : public ::testing::Test
{
protected:
  void SetUp()
  {
    constexpr auto test_interface = "vcan0";
    receiver_ = std::make_unique<SocketCanReceiver>(test_interface);
    sender_ = std::make_unique<SocketCanSender>(test_interface);
  }

  std::unique_ptr<SocketCanReceiver> receiver_{};
  std::unique_ptr<SocketCanSender> sender_{};
  std::chrono::milliseconds send_timeout_{1LL};
  std::chrono::milliseconds receive_timeout_{10LL};
};  // class receiver

TEST_F(DISABLED_receiver, basic_typed)
{
  constexpr uint32_t send_msg = 0x5A'5A'5A'5AU;
  const CanId send_id{};
  sender_->send(send_msg, send_id, send_timeout_);
  {
    uint32_t receive_msg{};
    CanId receive_id{};
    EXPECT_NO_THROW(receive_id = receiver_->receive(receive_msg, receive_timeout_));
    EXPECT_EQ(receive_msg, send_msg);
    EXPECT_EQ(receive_id.length(), sizeof(send_msg));
    EXPECT_EQ(send_id.get(), receive_id.get());
  }
}

TEST_F(DISABLED_receiver, ping_pong)
{
  for (uint64_t idx = 0U; idx < 100U; ++idx) {
    CanId send_id{};
    {
      send_id.identifier(static_cast<CanId::IdT>(idx));
      // Switch between standard and extended
      if (idx % 2U == 0U) {
        (void)send_id.extended();
      } else {
        (void)send_id.standard();
      }
      // Switch between remote and data; error frame is not picked up by socketCan?
      if (idx % 3U == 0U) {
        (void)send_id.data_frame();
      } else {
        (void)send_id.remote_frame();
      }
    }
    EXPECT_NO_THROW(sender_->send(idx, send_id, send_timeout_)) << idx;
    {
      decltype(idx) receive_msg{};
      CanId receive_id{};
      receive_id = receiver_->receive(receive_msg, receive_timeout_);
      EXPECT_EQ(receive_msg, idx);
      EXPECT_EQ(receive_id.length(), sizeof(idx));
      EXPECT_EQ(send_id.get(), receive_id.get());
    }
  }
}

TEST_F(DISABLED_receiver, can_filters)
{
  constexpr uint32_t send_msg = 0x5A'5A'5A'5AU;

  // pass only ids: 0x100, 0x250, 0x555
  receiver_->SetCanFilters({{0x100, 0x7FF}, {0x250, 0x7FF}, {0x555, 0x7FF}});
  CanId send_id{};
  send_id.standard();

  for (uint32_t idx = 0x50U; idx < 0x100U; idx++) {
    send_id.identifier(static_cast<CanId::IdT>(idx));
    sender_->send(send_msg, send_id, send_timeout_);
  }

  send_id.identifier(static_cast<CanId::IdT>(0x100U));
  sender_->send(send_msg, send_id, send_timeout_);

  for (uint32_t idx = 0x200U; idx < 0x250U; idx++) {
    send_id.identifier(static_cast<CanId::IdT>(idx));
    sender_->send(send_msg, send_id, send_timeout_);
  }

  send_id.identifier(static_cast<CanId::IdT>(0x250U));
  sender_->send(send_msg, send_id, send_timeout_);

  for (uint32_t idx = 0x500U; idx < 0x550U; idx++) {
    send_id.identifier(static_cast<CanId::IdT>(idx));
    sender_->send(send_msg, send_id, send_timeout_);
  }

  send_id.identifier(static_cast<CanId::IdT>(0x555U));
  sender_->send(send_msg, send_id, send_timeout_);

  uint32_t receive_msg{};
  CanId receive_id{};
  receive_id = receiver_->receive(receive_msg, receive_timeout_);
  EXPECT_EQ(0x100U, receive_id.get());
  receive_id = receiver_->receive(receive_msg, receive_timeout_);
  EXPECT_EQ(0x250U, receive_id.get());
  receive_id = receiver_->receive(receive_msg, receive_timeout_);
  EXPECT_EQ(0x555U, receive_id.get());

  // pass only even ids
  receiver_->SetCanFilters({{0x0, 0x1}});
  send_id.extended();
  for (uint32_t idx = 0x1000U; idx < 0x1050U; idx++) {
    send_id.identifier(static_cast<CanId::IdT>(idx));
    sender_->send(send_msg, send_id, send_timeout_);
    if (idx % 2 == 0) {
      receive_id = receiver_->receive(receive_msg, receive_timeout_);
      EXPECT_EQ(send_id.get(), receive_id.get());
    }
  }

  // pass none ids
  receiver_->SetCanFilters({});
  send_id.standard();
  for (uint32_t idx = 0x300U; idx < 0x330U; idx++) {
    send_id.identifier(static_cast<CanId::IdT>(idx));
    sender_->send(send_msg, send_id, send_timeout_);
  }
  EXPECT_THROW(receive_id = receiver_->receive(receive_msg, receive_timeout_), std::runtime_error);

  // pass all frames
  receiver_->SetCanFilters({{0x0, 0x0}});
  send_id.standard();
  for (uint32_t idx = 0x300U; idx < 0x330U; idx++) {
    send_id.identifier(static_cast<CanId::IdT>(idx));
    sender_->send(send_msg, send_id, send_timeout_);
    receive_id = receiver_->receive(receive_msg, receive_timeout_);
    EXPECT_EQ(send_id.get(), receive_id.get());
  }
}
