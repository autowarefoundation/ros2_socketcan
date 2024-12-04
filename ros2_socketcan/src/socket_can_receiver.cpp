// Copyright 2019 the Autoware Foundation
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
#include "ros2_socketcan/socket_can_receiver.hpp"

#include <unistd.h>  // for close()
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/sockios.h>

#include <cstring>
#include <string>
#include <sstream>
#include <vector>
#include <cstdio>

namespace drivers
{
namespace socketcan
{

////////////////////////////////////////////////////////////////////////////////
SocketCanReceiver::SocketCanReceiver(
  const std::string & interface, const bool enable_fd,
  const bool enable_loopback)
: m_file_descriptor{bind_can_socket(interface, enable_fd, enable_loopback)},
  m_enable_fd(enable_fd)
{
}

////////////////////////////////////////////////////////////////////////////////
SocketCanReceiver::~SocketCanReceiver() noexcept
{
  // Can't do anything on error; in fact generally shouldn't on close() error
  (void)close(m_file_descriptor);
}

////////////////////////////////////////////////////////////////////////////////
SocketCanReceiver::CanFilterList::CanFilterList(const char * str)
{
  *this = ParseFilters(str);
}

////////////////////////////////////////////////////////////////////////////////
SocketCanReceiver::CanFilterList::CanFilterList(const std::string & str)
{
  *this = ParseFilters(str);
}

////////////////////////////////////////////////////////////////////////////////
SocketCanReceiver::CanFilterList SocketCanReceiver::CanFilterList::ParseFilters(
  const std::string & str)
{
  CanFilterList filter_list;
  filter_list.error_mask = 0;
  filter_list.join_filters = false;

  std::istringstream input(str);
  std::string fstr;

  while (getline(input, fstr, ',')) {
    // trim leading and trailing whitespaces
    fstr = fstr.substr(
      fstr.find_first_not_of(" \t"),
      fstr.find_last_not_of(" \t") - fstr.find_first_not_of(" \t") + 1);

    struct can_filter filter;
    if (std::sscanf(fstr.c_str(), "%x:%x", &filter.can_id, &filter.can_mask) == 2) {
      filter.can_mask &= ~CAN_ERR_FLAG;
      if (fstr.size() > 8 && fstr[8] == ':') {
        filter.can_id |= CAN_EFF_FLAG;
      }
      filter_list.filters.push_back(filter);
    } else if (std::sscanf(fstr.c_str(), "%x~%x", &filter.can_id, &filter.can_mask) == 2) {
      filter.can_id |= CAN_INV_FILTER;
      filter.can_mask &= ~CAN_ERR_FLAG;
      if (fstr.size() > 8 && fstr[8] == '~') {
        filter.can_id |= CAN_EFF_FLAG;
      }
      filter_list.filters.push_back(filter);
    } else if (fstr == "j" || fstr == "J") {
      filter_list.join_filters = true;
    } else if (std::sscanf(fstr.c_str(), "#%x", &filter_list.error_mask) != 1) {
      throw std::runtime_error("Error during filter parsing: " + fstr);
    }
  }
  return filter_list;
}

////////////////////////////////////////////////////////////////////////////////
void SocketCanReceiver::SetCanFilters(const CanFilterList & filters)
{
  set_can_filter(m_file_descriptor, filters.filters);
  set_can_err_filter(m_file_descriptor, filters.error_mask);
  set_can_filter_join(m_file_descriptor, filters.join_filters);
}

////////////////////////////////////////////////////////////////////////////////
void SocketCanReceiver::wait(const std::chrono::nanoseconds timeout) const
{
  if (decltype(timeout)::zero() < timeout) {
    auto c_timeout = to_timeval(timeout);
    auto read_set = single_set(m_file_descriptor);
    // Wait
    if (0 == select(m_file_descriptor + 1, &read_set, NULL, NULL, &c_timeout)) {
      throw SocketCanTimeout{"CAN Receive Timeout"};
    }
    //lint --e{9130, 1924, 9123, 9125, 1924, 9126} NOLINT
    if (!FD_ISSET(m_file_descriptor, &read_set)) {
      throw SocketCanTimeout{"CAN Receive timeout"};
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
CanId SocketCanReceiver::receive(void * const data, const std::chrono::nanoseconds timeout) const
{
  if (m_enable_fd) {
    throw std::runtime_error{"attempted to read standard frame from FD socket"};
  }

  wait(timeout);
  // Read
  struct can_frame frame;
  const auto nbytes = read(m_file_descriptor, &frame, sizeof(frame));
  // Checks
  if (nbytes < 0) {
    throw std::runtime_error{strerror(errno)};
  }
  if (static_cast<std::size_t>(nbytes) < sizeof(frame)) {
    throw std::runtime_error{"read: incomplete CAN frame"};
  }
  if (static_cast<std::size_t>(nbytes) != sizeof(frame)) {
    throw std::logic_error{"Message was wrong size"};
  }
  // Write
  const auto data_length = static_cast<CanId::LengthT>(frame.can_dlc);
  (void)std::memcpy(data, static_cast<void *>(&frame.data[0U]), data_length);

  // get bus timestamp
  struct timeval tv;
  ioctl(m_file_descriptor, SIOCGSTAMP, &tv);
  uint64_t bus_time = from_timeval(tv);

  return CanId{frame.can_id, bus_time, data_length};
}

////////////////////////////////////////////////////////////////////////////////
CanId SocketCanReceiver::receive_fd(void * const data, const std::chrono::nanoseconds timeout) const
{
  if (!m_enable_fd) {
    throw std::runtime_error{"attempted to read FD frame from standard socket"};
  }

  wait(timeout);
  // Read
  struct canfd_frame frame;
  const auto nbytes = read(m_file_descriptor, &frame, sizeof(frame));

  // Checks
  if (nbytes < 0) {
    throw std::runtime_error{strerror(errno)};
  }

  if (static_cast<std::size_t>(nbytes) < sizeof(frame.can_id) + sizeof(frame.len)) {
    throw std::runtime_error{"read: corrupted CAN frame"};
  }

  if (frame.len > CANFD_MAX_DLEN) {
    throw std::runtime_error{"read: frame length is larger than max allowed CAN FD payload length"};
  }

  const auto data_length = static_cast<CanId::LengthT>(frame.len);
  // some CAN FD frames are shorter than 64 bytes
  const auto expected_length = sizeof(frame) - sizeof(frame.data) + data_length;

  if (static_cast<std::size_t>(nbytes) < expected_length) {
    throw std::runtime_error{"read: incomplete CAN FD frame"};
  }
  // Write
  (void)std::memcpy(data, static_cast<void *>(&frame.data[0U]), data_length);

  // get bus timestamp
  struct timeval tv;
  ioctl(m_file_descriptor, SIOCGSTAMP, &tv);
  uint64_t bus_time = from_timeval(tv);

  return CanId{frame.can_id, bus_time, data_length};
}

}  // namespace socketcan
}  // namespace drivers
