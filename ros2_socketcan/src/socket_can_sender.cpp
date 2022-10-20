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

#include "ros2_socketcan/socket_can_common.hpp"
#include "ros2_socketcan/socket_can_sender.hpp"

#include <unistd.h>  // for close()
#include <sys/select.h>
#include <sys/socket.h>
#include <linux/can.h>

#include <cstring>
#include <chrono>
#include <stdexcept>
#include <string>

namespace drivers
{
namespace socketcan
{

////////////////////////////////////////////////////////////////////////////////
SocketCanSender::SocketCanSender(const std::string & interface, const CanId & default_id)
: m_file_descriptor{bind_can_socket(interface)},
  m_default_id{default_id}
{
}

////////////////////////////////////////////////////////////////////////////////
SocketCanSender::~SocketCanSender() noexcept
{
  (void)close(m_file_descriptor);
  // I'm destructing--there's not much else I can do on an error
}

////////////////////////////////////////////////////////////////////////////////
CanId SocketCanSender::default_id() const noexcept
{
  return m_default_id;
}

////////////////////////////////////////////////////////////////////////////////
void SocketCanSender::send(
  const void * const data,
  const std::size_t length,
  const CanId id,
  const std::chrono::nanoseconds timeout) const
{
  if (length > MAX_DATA_LENGTH) {
    throw std::domain_error{"Size is too large to send via CAN"};
  }
  send_impl(data, length, id, timeout);
}

////////////////////////////////////////////////////////////////////////////////
void SocketCanSender::send(
  const void * const data,
  const std::size_t length,
  const std::chrono::nanoseconds timeout) const
{
  send(data, length, m_default_id, timeout);
}

////////////////////////////////////////////////////////////////////////////////
void SocketCanSender::wait(const std::chrono::nanoseconds timeout) const
{
  if (decltype(timeout)::zero() < timeout) {
    auto c_timeout = to_timeval(timeout);
    auto write_set = single_set(m_file_descriptor);
    // Wait
    if (0 == select(m_file_descriptor + 1, NULL, &write_set, NULL, &c_timeout)) {
      throw SocketCanTimeout{"CAN Send Timeout"};
    }
    //lint --e{9130, 9123, 9125, 1924, 9126} NOLINT
    if (!FD_ISSET(m_file_descriptor, &write_set)) {
      throw SocketCanTimeout{"CAN Send timeout"};
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
void SocketCanSender::send_impl(
  const void * const data,
  const std::size_t length,
  const CanId id,
  const std::chrono::nanoseconds timeout) const
{
  // Use select call on positive timeout
  wait(timeout);
  // Actually send the data
  constexpr int flags = 0;  // TODO(c.ho) not implemented
  struct can_frame data_frame;
  data_frame.can_id = id.get();
  // User facing functions do check
  data_frame.can_dlc = static_cast<decltype(data_frame.can_dlc)>(length);
  //lint -e{586} NOLINT data_frame is a stack variable; guaranteed not to overlap
  (void)std::memcpy(static_cast<void *>(&data_frame.data[0U]), data, length);
  const auto bytes_sent = ::send(m_file_descriptor, &data_frame, sizeof(data_frame), flags);
  if (0 > bytes_sent) {
    throw std::runtime_error{strerror(errno)};
  }
}

}  // namespace socketcan
}  // namespace drivers
