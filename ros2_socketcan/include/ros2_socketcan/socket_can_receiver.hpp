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
/// \file
/// \brief This file defines a class a socket sender
#ifndef ROS2_SOCKETCAN__SOCKET_CAN_RECEIVER_HPP_
#define ROS2_SOCKETCAN__SOCKET_CAN_RECEIVER_HPP_

#include <linux/can.h>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "ros2_socketcan/visibility_control.hpp"
#include "ros2_socketcan/socket_can_id.hpp"

namespace drivers
{
namespace socketcan
{

/// Simple RAII wrapper around a raw CAN receiver
class SOCKETCAN_PUBLIC SocketCanReceiver
{
public:
  /// Constructor
  explicit SocketCanReceiver(
    const std::string & interface = "can0", const bool enable_fd = false,
    const bool enable_loopback = false);
  /// Destructor
  ~SocketCanReceiver() noexcept;

  /// Structure containing possible CAN filter options.
  struct CanFilterList
  {
    std::vector<struct can_filter> filters;
    can_err_mask_t error_mask = 0;
    bool join_filters = false;

    /// Default constructor
    CanFilterList() = default;


    /// \copydoc ParseFilters(const std::string & str)
    explicit CanFilterList(const char * str);


    /// \copydoc ParseFilters(const std::string & str)
    explicit CanFilterList(const std::string & str);

    /// Parse CAN filters string:\n
    /// Filters:\n
    /// Comma separated filters can be specified for each given CAN interface.\n
    /// <can_id>:<can_mask>\n
    ///         (matches when <received_can_id> & mask == can_id & mask)\n
    /// <can_id>~<can_mask>\n
    ///         (matches when <received_can_id> & mask != can_id & mask)\n
    /// #<error_mask>\n
    ///         (set error frame filter, see include/linux/can/error.h)\n
    /// [j|J]\n
    ///         (join the given CAN filters - logical AND semantic)\n
    ///
    /// CAN IDs, masks and data content are given and expected in hexadecimal values.
    /// When can_id and can_mask are both 8 digits, they are assumed to be 29 bit EFF.
    /// \see https://manpages.ubuntu.com/manpages/jammy/man1/candump.1.html
    /// \param[in] str Input to be parsed.
    /// \return Populated CanFilterList structure.
    /// \throw std::runtime_error if string couldn't be parsed.
    static CanFilterList ParseFilters(const std::string & str);
  };

  /// Set SocketCAN filters
  /// \param[in] filters List of filters to be applied.
  /// \throw std::runtime_error If filters couldn't be applied
  void SetCanFilters(const CanFilterList & filters);

  /// Receive CAN data
  /// \param[out] data A buffer to be written with data bytes. Must be at least 8 bytes in size
  /// \param[in] timeout Maximum duration to wait for data on the file descriptor. Negative
  ///                    durations are treated the same as zero timeout
  /// \return The CanId for the received can_frame, with length appropriately populated
  /// \throw SocketCanTimeout On timeout
  /// \throw std::runtime_error on other errors
  CanId receive(
    void * const data,
    const std::chrono::nanoseconds timeout = std::chrono::nanoseconds::zero()) const;
  /// Receive typed CAN data. Slightly less efficient than untyped interface; has extra copy and
  /// branches
  /// \tparam Type of data to receive, must be 8 bytes or smaller
  /// \param[out] data A buffer to be written with data bytes. Must be at least 8 bytes in size
  /// \param[in] timeout Maximum duration to wait for data on the file descriptor. Negative
  ///                    durations are treated the same as zero timeout
  /// \return The CanId for the received can_frame, with length appropriately populated
  /// \throw SocketCanTimeout On timeout
  /// \throw std::runtime_error If received data would not fit into provided type
  /// \throw std::runtime_error on other errors
  template<typename T, typename = std::enable_if_t<!std::is_pointer<T>::value>>
  CanId receive(
    T & data,
    const std::chrono::nanoseconds timeout = std::chrono::nanoseconds::zero()) const
  {
    static_assert(sizeof(data) <= MAX_DATA_LENGTH, "Data type too large for CAN");
    std::array<uint8_t, MAX_DATA_LENGTH> data_raw{};
    const auto ret = receive(&data_raw[0U], timeout);
    if (ret.length() != sizeof(data)) {
      throw std::runtime_error{"Received CAN data is of size incompatible with provided type!"};
    }
    (void)std::memcpy(&data, &data_raw[0U], ret.length());
    return ret;
  }

  /// Receive CAN FD data
  /// \param[out] data A buffer to be written with data bytes. Must be at least 64 bytes in size
  /// \param[in] timeout Maximum duration to wait for data on the file descriptor. Negative
  ///                    durations are treated the same as zero timeout
  /// \return The CanId for the received canfd_frame, with length appropriately populated
  /// \throw SocketCanTimeout On timeout
  /// \throw std::runtime_error on other errors
  CanId receive_fd(
    void * const data,
    const std::chrono::nanoseconds timeout = std::chrono::nanoseconds::zero()) const;
  /// Receive typed CAN FD data. Slightly less efficient than untyped interface; has extra copy and
  /// branches
  /// \tparam Type of data to receive, must be 64 bytes or smaller
  /// \param[out] data A buffer to be written with data bytes. Must be at least 64 bytes in size
  /// \param[in] timeout Maximum duration to wait for data on the file descriptor. Negative
  ///                    durations are treated the same as zero timeout
  /// \return The CanId for the received canfd_frame, with length appropriately populated
  /// \throw SocketCanTimeout On timeout
  /// \throw std::runtime_error If received data would not fit into provided type
  /// \throw std::runtime_error on other errors
  template<typename T, typename = std::enable_if_t<!std::is_pointer<T>::value>>
  CanId receive_fd(
    T & data,
    const std::chrono::nanoseconds timeout = std::chrono::nanoseconds::zero()) const
  {
    static_assert(sizeof(data) <= MAX_FD_DATA_LENGTH, "Data type too large for CAN FD");
    std::array<uint8_t, MAX_FD_DATA_LENGTH> data_raw{};
    const auto ret = receive_fd(&data_raw[0U], timeout);
    if (ret.length() != sizeof(data)) {
      throw std::runtime_error{"Received CAN FD data is of size incompatible with provided type!"};
    }
    (void)std::memcpy(&data, &data_raw[0U], ret.length());
    return ret;
  }

private:
  // Wait for file descriptor to be available to send data via select()
  SOCKETCAN_LOCAL void wait(const std::chrono::nanoseconds timeout) const;

  int32_t m_file_descriptor;
  bool m_enable_fd;
};  // class SocketCanReceiver

}  // namespace socketcan
}  // namespace drivers

#endif  // ROS2_SOCKETCAN__SOCKET_CAN_RECEIVER_HPP_
