#include "ros2_socketcan/socket_can_transceiver.hpp"
#include "ros2_socketcan/socket_can_common.hpp"
#include <cstring>
#include <linux/sockios.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace drivers {
namespace socketcan {

SocketCanTransceiver::SocketCanTransceiver(const std::string &interface)
    : m_file_descriptor{bind_can_socket(interface, false, true)} {}

void SocketCanTransceiver::send(const void *const data,
                                const std::size_t length, const CanId id,
                                const std::chrono::nanoseconds timeout) const {
  if (length > MAX_DATA_LENGTH) {
    throw std::domain_error{"Size is too large to send via CAN"};
  }
  send_impl(data, length, id, timeout);
}

SocketCanTransceiver::~SocketCanTransceiver() noexcept {
  // Can't do anything on error; in fact generally shouldn't on close() error
  (void)close(m_file_descriptor);
}

void SocketCanTransceiver::send_impl(
    const void *const data, const std::size_t length, const CanId id,
    const std::chrono::nanoseconds timeout) const {

  // Use select call on positive timeout
  wait_send(timeout);
  // Actually send the data
  constexpr int flags = 0; // TODO(c.ho) not implemented
  struct can_frame data_frame;
  data_frame.can_id = id.get();
  // User facing functions do check
  data_frame.can_dlc = static_cast<decltype(data_frame.can_dlc)>(length);
  // lint -e{586} NOLINT data_frame is a stack variable; guaranteed not to
  // overlap
  (void)std::memcpy(static_cast<void *>(&data_frame.data[0U]), data, length);
  const auto bytes_sent =
      ::send(m_file_descriptor, &data_frame, sizeof(data_frame), flags);
  if (0 > bytes_sent) {
    throw std::runtime_error{strerror(errno)};
  }
}

void SocketCanTransceiver::wait_send(
    const std::chrono::nanoseconds timeout) const {
  if (decltype(timeout)::zero() < timeout) {
    auto c_timeout = to_timeval(timeout);
    auto write_set = single_set(m_file_descriptor);
    // Wait
    if (0 ==
        select(m_file_descriptor + 1, NULL, &write_set, NULL, &c_timeout)) {
      throw SocketCanTimeout{"CAN Send Timeout"};
    }
    // lint --e{9130, 9123, 9125, 1924, 9126} NOLINT
    if (!FD_ISSET(m_file_descriptor, &write_set)) {
      throw SocketCanTimeout{"CAN Send timeout"};
    }
  }
}

CanId SocketCanTransceiver::receive(
    void *const data, const std::chrono::nanoseconds timeout) const {

  wait_receive(timeout);
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

void SocketCanTransceiver::wait_receive(
    const std::chrono::nanoseconds timeout) const {
  if (decltype(timeout)::zero() < timeout) {
    auto c_timeout = to_timeval(timeout);
    auto read_set = single_set(m_file_descriptor);
    // Wait
    if (0 == select(m_file_descriptor + 1, &read_set, NULL, NULL, &c_timeout)) {
      throw SocketCanTimeout{"CAN Receive Timeout"};
    }
    // lint --e{9130, 1924, 9123, 9125, 1924, 9126} NOLINT
    if (!FD_ISSET(m_file_descriptor, &read_set)) {
      throw SocketCanTimeout{"CAN Receive timeout"};
    }
  }
}

} // namespace socketcan
} // namespace drivers