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

#include <unistd.h>
#include <fcntl.h>
#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <gtest/gtest.h>
#include <cstring>

#include <memory>
#include <string>

#include "ros2_socketcan/socket_can_sender.hpp"
#include "ros2_socketcan/socket_can_receiver.hpp"

using drivers::socketcan::SocketCanSender;
using drivers::socketcan::SocketCanReceiver;
using drivers::socketcan::CanId;
using drivers::socketcan::StandardFrame;
using drivers::socketcan::ExtendedFrame;
using drivers::socketcan::FrameType;
using drivers::socketcan::MAX_DATA_LENGTH;


// Exercise the CanId stuff
TEST(socket_can_basics, id_bad)
{
  // Bad frame type
  // had to re-write to use lambda to compile properly
  const auto construct_bad_frame = []() -> auto {
      constexpr CanId::IdT truncated_id = 0x6000'0000U;
      return CanId{truncated_id, 0};
    };
  EXPECT_THROW(construct_bad_frame(), std::domain_error);

  // Standard truncation
  const auto construct = [](const auto frame) -> auto {
      constexpr CanId::IdT truncated_id = 0xFFFF'FFFFU;
      return CanId{truncated_id, 0, FrameType::DATA, frame};
    };
  EXPECT_THROW(construct(StandardFrame), std::domain_error);
  EXPECT_THROW(construct(ExtendedFrame), std::domain_error);
}

TEST(socket_can_basics, id)
{
  // Default
  {
    CanId id{};
    EXPECT_EQ(id.get(), 0U);
    EXPECT_FALSE(id.is_extended());
    EXPECT_EQ(id.frame_type(), FrameType::DATA);
    // Set to extended
    id = id.extended();
    EXPECT_TRUE(id.is_extended());
    EXPECT_EQ(id.get(), 0x8000'0000U);
    // Change type to error
    id = id.error_frame();
    EXPECT_EQ(id.frame_type(), FrameType::ERROR);
    EXPECT_EQ(id.get(), 0xA000'0000U);
    // Change type to remote
    id = id.remote_frame();
    EXPECT_EQ(id.frame_type(), FrameType::REMOTE);
    EXPECT_EQ(id.get(), 0xC000'0000U);
    // Set to standard
    id = id.standard();
    EXPECT_FALSE(id.is_extended());
    EXPECT_EQ(id.get(), 0x4000'0000U);
    // Change type to data
    id = id.data_frame();
    EXPECT_EQ(id.frame_type(), FrameType::DATA);
    EXPECT_EQ(id.get(), 0U);
  }
}

// Sanity checks on constructor
TEST(socket_can_basics, bad_constructor)
{
  {
    const std::string long_name{"abcdefghijklmnopqrs"};
    ASSERT_GE(long_name.size(), 14U);
    EXPECT_THROW(SocketCanSender{long_name}, std::domain_error);
    EXPECT_THROW(SocketCanReceiver{long_name}, std::domain_error);
  }
  {
    constexpr auto nonexistent_interface = "foo";
    EXPECT_THROW(SocketCanSender{nonexistent_interface}, std::runtime_error);
    EXPECT_THROW(SocketCanReceiver{nonexistent_interface}, std::runtime_error);
  }
}

// Requires elevated kernel permissions normal containers can't provide
class DISABLED_sender_test : public ::testing::Test
{
public:
  using MsgT = uint64_t;
  static_assert(sizeof(MsgT) == 8U, "Data size is incorrect");

protected:
  void SetUp()
  {
    constexpr auto TEST_INTERFACE = "vcan0";
    sender_ = std::make_unique<SocketCanSender>(TEST_INTERFACE);
    // Set up file descriptor
    file_descriptor_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    fcntl(file_descriptor_, F_SETFL, O_NONBLOCK);
    struct sockaddr_can addr;
    struct ifreq ifr;

    strcpy(ifr.ifr_name, TEST_INTERFACE);  // NOLINT literally just copying bytes
    ioctl(file_descriptor_, SIOCGIFINDEX, &ifr);

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    bind(file_descriptor_, (struct sockaddr *)&addr, sizeof(addr));
  }

  void TearDown()
  {
    close(file_descriptor_);
  }

  uint32_t receive(
    MsgT & msg,
    const std::chrono::nanoseconds timeout = std::chrono::milliseconds{1LL})
  {
    if (timeout < decltype(timeout)::zero()) {
      throw std::domain_error{"Negative timeout"};
    }
    if (timeout >= std::chrono::seconds{1LL}) {
      throw std::domain_error{"Timeout >= 1s, not dealing with this"};
    }
    // Set up selector
    {
      struct timeval c_timeout;
      c_timeout.tv_sec = 0;
      c_timeout.tv_usec = std::chrono::duration_cast<std::chrono::milliseconds>(timeout).count();

      fd_set read_set;
      FD_ZERO(&read_set);
      FD_SET(file_descriptor_, &read_set);
      // Wait
      if (0 == select(file_descriptor_ + 1, &read_set, nullptr, nullptr, &c_timeout)) {
        throw std::runtime_error{"Timeout"};
      }
      if (!FD_ISSET(file_descriptor_, &read_set)) {
        throw std::runtime_error{"What?"};
      }
    }
    // Read
    struct can_frame frame;
    const auto nbytes = read(file_descriptor_, &frame, sizeof(frame));
    // Checks
    if (nbytes < 0) {
      throw std::runtime_error{"CAN raw socket read"};
      perror("can raw socket read");
    }
    if (static_cast<std::size_t>(nbytes) < sizeof(frame)) {
      throw std::runtime_error{"read: incomplete CAN frame"};
    }
    if (static_cast<std::size_t>(nbytes) != sizeof(frame)) {
      throw std::logic_error{"Message was wrong size"};
    }
    // Write
    (void)std::memcpy(&msg, frame.data, sizeof(msg));
    return frame.can_id;
  }

  std::unique_ptr<SocketCanSender> sender_;
  int file_descriptor_{};
};  // class sender_test

// Minimal usage
TEST_F(DISABLED_sender_test, basic_untyped)
{
  constexpr MsgT data = 0xA5A5A5A5A5A5A5A5U;
  // Use untyped interface
  {
    EXPECT_THROW(
      sender_->send(&data, MAX_DATA_LENGTH + 1U, std::chrono::milliseconds{1LL}),
      std::domain_error
    );
    sender_->send(&data, 8U, std::chrono::milliseconds{1LL});
    MsgT msg{};
    ASSERT_NE(msg, data);
    const auto id = receive(msg);
    EXPECT_EQ(data, msg);
    EXPECT_EQ(id, sender_->default_id().get());
  }
}

TEST_F(DISABLED_sender_test, basic_typed)
{
  constexpr MsgT data = 0xA5A5A5A5A5A5A5A5U;
  // Use typed interface
  {
    sender_->send(data, std::chrono::milliseconds{1LL});
    MsgT msg{};
    ASSERT_NE(msg, data);
    const auto id = receive(msg);
    EXPECT_EQ(data, msg);
    EXPECT_EQ(id, sender_->default_id().get());
  }
}

// Ensure there's no funny stateful stuff happening
TEST_F(DISABLED_sender_test, sequential)
{
  for (MsgT idx = 1UL; idx < 100UL; ++idx) {
    sender_->send(idx, std::chrono::milliseconds{1LL});
    MsgT msg{};
    ASSERT_NE(msg, idx);
    const auto id = receive(msg);
    EXPECT_EQ(idx, msg);
    EXPECT_EQ(id, sender_->default_id().get());
  }
}
