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
#include <linux/can/error.h>

#include <chrono>
#include <memory>
#include <string>

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

TEST_F(DISABLED_receiver, can_filters_parser)
{
  typedef SocketCanReceiver::CanFilterList CanFilterList;

  auto filter_list = CanFilterList("101:7FF,333:ab,404:1,92345678:DFFFFFFF");
  ASSERT_EQ(filter_list.filters.size(), 4U);
  EXPECT_EQ(filter_list.filters[0].can_id, 0x101U);
  EXPECT_EQ(filter_list.filters[0].can_mask, 0x7FFU);
  EXPECT_EQ(filter_list.filters[1].can_id, 0x333U);
  EXPECT_EQ(filter_list.filters[1].can_mask, 0xABU);
  EXPECT_EQ(filter_list.filters[2].can_id, 0x404U);
  EXPECT_EQ(filter_list.filters[2].can_mask, 0x1U);
  EXPECT_EQ(filter_list.filters[3].can_id, 0x92345678U);
  EXPECT_EQ(filter_list.filters[3].can_mask, 0xDFFFFFFFU);
  EXPECT_EQ(filter_list.error_mask, 0x0U);
  EXPECT_FALSE(filter_list.join_filters);

  filter_list = CanFilterList("");
  EXPECT_TRUE(filter_list.filters.empty());
  EXPECT_EQ(filter_list.error_mask, 0x0U);
  EXPECT_FALSE(filter_list.join_filters);
  filter_list = CanFilterList("#12345");
  EXPECT_TRUE(filter_list.filters.empty());
  EXPECT_EQ(filter_list.error_mask, 0x12345U);
  EXPECT_FALSE(filter_list.join_filters);

  filter_list = CanFilterList("j");
  EXPECT_TRUE(filter_list.filters.empty());
  EXPECT_EQ(filter_list.error_mask, 0x0U);
  EXPECT_TRUE(filter_list.join_filters);


  filter_list = CanFilterList("0~0,#FFFFFFFF");
  ASSERT_EQ(filter_list.filters.size(), 1U);
  EXPECT_EQ(filter_list.filters[0].can_id, 0x0U | CAN_INV_FILTER);
  EXPECT_EQ(filter_list.filters[0].can_mask, 0x0U);
  EXPECT_EQ(filter_list.error_mask, 0xFFFFFFFFU);
  EXPECT_FALSE(filter_list.join_filters);

  filter_list = CanFilterList("1:2,3~4,5:6,7~8,9:A,j");
  ASSERT_EQ(filter_list.filters.size(), 5U);
  EXPECT_EQ(filter_list.filters[0].can_id, 0x1U);
  EXPECT_EQ(filter_list.filters[0].can_mask, 0x2U);
  EXPECT_EQ(filter_list.filters[1].can_id, 0x3U | CAN_INV_FILTER);
  EXPECT_EQ(filter_list.filters[1].can_mask, 0x4U);
  EXPECT_EQ(filter_list.filters[2].can_id, 0x5U);
  EXPECT_EQ(filter_list.filters[2].can_mask, 0x6U);
  EXPECT_EQ(filter_list.filters[3].can_id, 0x7U | CAN_INV_FILTER);
  EXPECT_EQ(filter_list.filters[3].can_mask, 0x8U);
  EXPECT_EQ(filter_list.filters[4].can_id, 0x9U);
  EXPECT_EQ(filter_list.filters[4].can_mask, 0xAU);
  EXPECT_EQ(filter_list.error_mask, 0x0U);
  EXPECT_TRUE(filter_list.join_filters);

  filter_list = CanFilterList("ABC:DEF,123:C00007FF,J,#5");
  ASSERT_EQ(filter_list.filters.size(), 2U);
  EXPECT_EQ(filter_list.filters[0].can_id, 0xABCU);
  EXPECT_EQ(filter_list.filters[0].can_mask, 0xDEFU);
  EXPECT_EQ(filter_list.filters[1].can_id, 0x123U);
  EXPECT_EQ(filter_list.filters[1].can_mask, 0xC00007FFU);
  EXPECT_EQ(filter_list.error_mask, 0x5U);
  EXPECT_TRUE(filter_list.join_filters);

  // whitespace trimming test
  filter_list = CanFilterList(
    "        ABC:DEF             , 123:C00007FF       ,         J   ,    #5    ");
  ASSERT_EQ(filter_list.filters.size(), 2U);
  EXPECT_EQ(filter_list.filters[0].can_id, 0xABCU);
  EXPECT_EQ(filter_list.filters[0].can_mask, 0xDEFU);
  EXPECT_EQ(filter_list.filters[1].can_id, 0x123U);
  EXPECT_EQ(filter_list.filters[1].can_mask, 0xC00007FFU);
  EXPECT_EQ(filter_list.error_mask, 0x5U);
  EXPECT_TRUE(filter_list.join_filters);

  // test incorrect input
  std::string str = "        ABC:DEF             , 123:C00007FF       ,         J   ,    #p5    ";
  EXPECT_THROW(CanFilterList::ParseFilters(str), std::runtime_error);
  str = "1:2,3~4,5:6,7~8,9:A,l";
  EXPECT_THROW(CanFilterList::ParseFilters(str), std::runtime_error);
  str = "1:2,3~4,5;6,7~8,9:A,j";
  EXPECT_THROW(CanFilterList::ParseFilters(str), std::runtime_error);
  str = "1:2,3 ~4,5:6,7~8,9:A,j";
  EXPECT_THROW(CanFilterList::ParseFilters(str), std::runtime_error);
  str = "not a correct string";
  EXPECT_THROW(CanFilterList::ParseFilters(str), std::runtime_error);
}

TEST_F(DISABLED_receiver, can_filters)
{
  constexpr uint32_t send_msg = 0x5A'5A'5A'5AU;
  SocketCanReceiver::CanFilterList filter_list;

  ////////////////////////////////////////////////////////////////////////////////
  // pass only ids: 0x100, 0x250, 0x555 of standard length
  filter_list.filters = {{0x100, 0xC00007FF}, {0x250, 0xC00007FF}, {0x555, 0xC00007FF}};
  receiver_->SetCanFilters(filter_list);
  CanId send_id{};

  // error frame should be blocked
  send_id.error_frame();
  send_id.identifier(static_cast<CanId::IdT>(0x100U));
  sender_->send(send_msg, send_id, send_timeout_);

  // RTR frame should be blocked
  send_id.remote_frame();
  send_id.standard();
  send_id.identifier(static_cast<CanId::IdT>(0x100U));
  sender_->send(send_msg, send_id, send_timeout_);

  // extended data frame should be blocked
  send_id.data_frame();
  send_id.extended();
  send_id.identifier(static_cast<CanId::IdT>(0x100U));
  sender_->send(send_msg, send_id, send_timeout_);

  // wrong ids - should be blocked
  send_id.data_frame();
  send_id.standard();
  for (uint32_t idx = 0x50U; idx < 0x100U; idx++) {
    send_id.identifier(static_cast<CanId::IdT>(idx));
    sender_->send(send_msg, send_id, send_timeout_);
  }

  // should pass
  send_id.identifier(static_cast<CanId::IdT>(0x100U));
  sender_->send(send_msg, send_id, send_timeout_);

  // wrong ids - should be blocked
  for (uint32_t idx = 0x200U; idx < 0x250U; idx++) {
    send_id.identifier(static_cast<CanId::IdT>(idx));
    sender_->send(send_msg, send_id, send_timeout_);
  }

  // should pass
  send_id.identifier(static_cast<CanId::IdT>(0x250U));
  sender_->send(send_msg, send_id, send_timeout_);

  // wrong ids - should be blocked
  for (uint32_t idx = 0x500U; idx < 0x550U; idx++) {
    send_id.identifier(static_cast<CanId::IdT>(idx));
    sender_->send(send_msg, send_id, send_timeout_);
  }

  // should pass
  send_id.identifier(static_cast<CanId::IdT>(0x555U));
  sender_->send(send_msg, send_id, send_timeout_);

  uint32_t receive_msg{};
  CanId receive_id{};
  receive_id = receiver_->receive(receive_msg, receive_timeout_);
  EXPECT_EQ(0x100U, receive_id.get());
  EXPECT_FALSE(receive_id.is_extended());
  EXPECT_EQ(receive_id.frame_type(), FrameType::DATA);
  receive_id = receiver_->receive(receive_msg, receive_timeout_);
  EXPECT_EQ(0x250U, receive_id.get());
  EXPECT_FALSE(receive_id.is_extended());
  EXPECT_EQ(receive_id.frame_type(), FrameType::DATA);
  receive_id = receiver_->receive(receive_msg, receive_timeout_);
  EXPECT_EQ(0x555U, receive_id.get());
  EXPECT_FALSE(receive_id.is_extended());
  EXPECT_EQ(receive_id.frame_type(), FrameType::DATA);

  ////////////////////////////////////////////////////////////////////////////////
  // pass only even ids
  filter_list.filters = {{0x0, 0x1}};
  receiver_->SetCanFilters(filter_list);
  send_id.extended();
  for (uint32_t idx = 0x1000U; idx < 0x1050U; idx++) {
    send_id.identifier(static_cast<CanId::IdT>(idx));
    sender_->send(send_msg, send_id, send_timeout_);
    if (idx % 2 == 0) {
      receive_id = receiver_->receive(receive_msg, receive_timeout_);
      EXPECT_EQ(send_id.get(), receive_id.get());
      EXPECT_EQ(send_id.frame_type(), FrameType::DATA);
    }
  }

  ////////////////////////////////////////////////////////////////////////////////
  // pass none ids
  filter_list.filters = {};
  receiver_->SetCanFilters(filter_list);
  send_id.standard();
  for (uint32_t idx = 0x300U; idx < 0x330U; idx++) {
    send_id.identifier(static_cast<CanId::IdT>(idx));
    sender_->send(send_msg, send_id, send_timeout_);
  }
  EXPECT_THROW(receive_id = receiver_->receive(receive_msg, receive_timeout_), std::runtime_error);

  ////////////////////////////////////////////////////////////////////////////////
  // pass all frames (including errors and remotes)
  filter_list.filters = {{0x0, 0x0}};
  filter_list.error_mask = 0xFFFFFFFFU;
  receiver_->SetCanFilters(filter_list);
  send_id.standard();
  for (uint32_t idx = 0x300U; idx < 0x330U; idx++) {
    send_id.identifier(static_cast<CanId::IdT>(idx));
    sender_->send(send_msg, send_id, send_timeout_);
    receive_id = receiver_->receive(receive_msg, receive_timeout_);
    EXPECT_EQ(send_id.get(), receive_id.get());
    EXPECT_EQ(send_id.frame_type(), FrameType::DATA);
  }
  send_id.error_frame();
  for (uint32_t idx = 0x200U; idx < 0x230U; idx++) {
    send_id.identifier(static_cast<CanId::IdT>(idx));
    sender_->send(send_msg, send_id, send_timeout_);
    receive_id = receiver_->receive(receive_msg, receive_timeout_);
    EXPECT_EQ(send_id.get(), receive_id.get());
    EXPECT_EQ(send_id.frame_type(), FrameType::ERROR);
  }
  send_id.remote_frame();
  for (uint32_t idx = 0x100U; idx < 0x130U; idx++) {
    send_id.identifier(static_cast<CanId::IdT>(idx));
    sender_->send(send_msg, send_id, send_timeout_);
    receive_id = receiver_->receive(receive_msg, receive_timeout_);
    EXPECT_EQ(send_id.get(), receive_id.get());
    EXPECT_EQ(send_id.frame_type(), FrameType::REMOTE);
  }


  ////////////////////////////////////////////////////////////////////////////////
  // JOIN FILTERS: pass only CAN_ERR_TX_TIMEOUT and CAN_ERR_BUSOFF error frames
  // filter_list.filters = {{0x0, 0x0 | CAN_INV_FILTER}};
  // filter_list.error_mask = (CAN_ERR_TX_TIMEOUT | CAN_ERR_BUSOFF);
  // receiver_->SetCanFilters(filter_list);
  receiver_->SetCanFilters(SocketCanReceiver::CanFilterList("0~0,#41"));  // same as above comment

  // should be blocked
  for (uint32_t idx = 0x300U; idx < 0x330U; idx++) {
    send_id.identifier(static_cast<CanId::IdT>(idx));
    sender_->send(send_msg, send_id, send_timeout_);
  }

  // should pass
  send_id.error_frame();
  send_id.identifier(static_cast<CanId::IdT>(CAN_ERR_TX_TIMEOUT));
  sender_->send(send_msg, send_id, send_timeout_);
  receive_id = receiver_->receive(receive_msg, receive_timeout_);
  EXPECT_EQ(send_id.get(), receive_id.get());
  EXPECT_EQ(send_id.frame_type(), FrameType::ERROR);

  // should be blocked
  send_id.identifier(static_cast<CanId::IdT>(CAN_ERR_ACK));
  sender_->send(send_msg, send_id, send_timeout_);

  // should pass
  send_id.error_frame();
  send_id.identifier(static_cast<CanId::IdT>(CAN_ERR_BUSOFF));
  sender_->send(send_msg, send_id, send_timeout_);
  receive_id = receiver_->receive(receive_msg, receive_timeout_);
  EXPECT_EQ(send_id.get(), receive_id.get());
  EXPECT_EQ(send_id.frame_type(), FrameType::ERROR);

  ////////////////////////////////////////////////////////////////////////////////
  // JOIN FILTERS: pass only even id data and remote frames from 0x400 to 0x499
  filter_list.filters = {{0x0, 0x1}, {0x400, 0x700}};
  filter_list.error_mask = 0;
  filter_list.join_filters = true;
  receiver_->SetCanFilters(filter_list);

  send_id.data_frame();
  send_id.standard();

  // should be blocked
  for (uint32_t idx = 0x300U; idx < 0x330U; idx++) {
    send_id.identifier(static_cast<CanId::IdT>(idx));
    sender_->send(send_msg, send_id, send_timeout_);
  }

  // only even should pass
  for (uint32_t idx = 0x400U; idx < 0x500U; idx++) {
    send_id.identifier(static_cast<CanId::IdT>(idx));
    sender_->send(send_msg, send_id, send_timeout_);
    if (idx % 2 == 0) {
      receive_id = receiver_->receive(receive_msg, receive_timeout_);
      EXPECT_EQ(send_id.get(), receive_id.get());
      EXPECT_EQ(send_id.frame_type(), FrameType::DATA);
    }
  }

  // only even should pass
  send_id.remote_frame();
  for (uint32_t idx = 0x400U; idx < 0x500U; idx++) {
    send_id.identifier(static_cast<CanId::IdT>(idx));
    sender_->send(send_msg, send_id, send_timeout_);
    if (idx % 2 == 0) {
      receive_id = receiver_->receive(receive_msg, receive_timeout_);
      EXPECT_EQ(send_id.get(), receive_id.get());
      EXPECT_EQ(send_id.frame_type(), FrameType::REMOTE);
    }
  }
}
