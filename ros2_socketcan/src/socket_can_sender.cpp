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

#define ERR_AGAIN_RETRY_CNT 10

namespace drivers
{
namespace socketcan
{

////////////////////////////////////////////////////////////////////////////////
SocketCanSender::SocketCanSender(
  const std::string & interface,
  const bool enable_fd,
  const CanId & default_id)
: m_enable_fd(enable_fd),
  m_file_descriptor{bind_can_socket(interface, m_enable_fd)},
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
void SocketCanSender::send_fd(
  const void * const data,
  const std::size_t length,
  const CanId id,
  const std::chrono::nanoseconds timeout) const
{
  if (length > MAX_FD_DATA_LENGTH) {
    throw std::domain_error{"Size is too large to send via CAN FD"};
  }
  send_fd_impl(data, length, id, timeout);
}

////////////////////////////////////////////////////////////////////////////////
void SocketCanSender::send_fd(
  const void * const data,
  const std::size_t length,
  const std::chrono::nanoseconds timeout) const
{
  send_fd(data, length, m_default_id, timeout);
}

////////////////////////////////////////////////////////////////////////////////
void SocketCanSender::wait(const std::chrono::nanoseconds timeout) const
{
  if (decltype(timeout)::zero() < timeout) {
    auto c_timeout = to_timeval(timeout);
    int retval;
    auto write_set = single_set(m_file_descriptor);
    retval = select(m_file_descriptor + 1, NULL, &write_set, NULL, &c_timeout);
    // Check return value of the select function
    if (0 == retval) {
      throw SocketCanTimeout{"CAN Send Timeout"};
    } else if (-1 == retval) {
      // Output errno
      throw std::runtime_error{strerror(errno)};
    } else {
      // Check file descriptor
      //lint --e{9130, 1924, 9123, 9125, 1924, 9126} NOLINT
      if (0 == FD_ISSET(m_file_descriptor, &write_set)) {
        throw SocketCanTimeout{"File descriptor not set"};
      }
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
  if (m_enable_fd) {
    throw std::runtime_error{"Tried to send standard frame from FD socket"};
  }

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

  int number = 0;
  ssize_t nbytes = 0;

  struct can_frame * p = &data_frame;
  ulong bytes_sent = 0;
  while (bytes_sent < sizeof(data_frame) && -1 != nbytes) {
    for (int i = 0; i < ERR_AGAIN_RETRY_CNT; i++) {
      nbytes = ::send(
        m_file_descriptor, (char *)p + bytes_sent, sizeof(data_frame) - bytes_sent,
        flags);
      if (-1 == nbytes) {
        number = errno;
        if (!(EAGAIN == (number) || EWOULDBLOCK == (number) || EINTR == (number))) {
          break;
        }
      } else {
        bytes_sent += nbytes;
        break;
      }
    }
  }
  // Checks
  if (-1 == nbytes) {
    throw std::runtime_error{strerror(errno)};
  }
}

////////////////////////////////////////////////////////////////////////////////
void SocketCanSender::send_fd_impl(
  const void * const data,
  const std::size_t length,
  const CanId id,
  const std::chrono::nanoseconds timeout) const
{
  if (!m_enable_fd) {
    throw std::runtime_error{"Tried to send FD frame from standard socket"};
  }

  // Use select call on positive timeout
  wait(timeout);
  // Actually send the data
  constexpr int flags = 0;  // TODO(c.ho) not implemented
  struct canfd_frame data_frame;
  data_frame.can_id = id.get();
  // User facing functions do check
  data_frame.len = static_cast<decltype(data_frame.len)>(length);
  //lint -e{586} NOLINT data_frame is a stack variable; guaranteed not to overlap
  (void)std::memcpy(static_cast<void *>(&data_frame.data[0U]), data, length);
  const auto bytes_sent = ::send(m_file_descriptor, &data_frame, sizeof(data_frame), flags);
  if (0 > bytes_sent) {
    throw std::runtime_error{strerror(errno)};
  }
}

}  // namespace socketcan
}  // namespace drivers
