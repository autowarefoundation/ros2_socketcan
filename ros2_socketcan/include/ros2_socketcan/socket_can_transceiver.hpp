#ifndef ROS2_SOCKETCAN__SOCKET_CAN_TRANSCEIVER_HPP_
#define ROS2_SOCKETCAN__SOCKET_CAN_TRANSCEIVER_HPP_

#include "ros2_socketcan/socket_can_id.hpp"
#include "ros2_socketcan/visibility_control.hpp"
#include <chrono>

namespace drivers {
namespace socketcan {

class SOCKETCAN_PUBLIC SocketCanTransceiver {
public:
  /// Constructor
  explicit SocketCanTransceiver(const std::string &interface = "can0");
  /// Destructor
  ~SocketCanTransceiver() noexcept;

  /// Send raw data with an explicit CAN id
  /// \param[in] data A pointer to the beginning of the data to send
  /// \param[in] timeout Maximum duration to wait for file descriptor to be free
  /// for write. Negative
  ///                    durations are treated the same as zero timeout
  /// \param[in] id The id field for the CAN frame
  /// \param[in] length The amount of data to send starting from the data
  /// pointer
  /// \throw std::domain_error If length is > 8
  /// \throw SocketCanTimeout On timeout
  /// \throw std::runtime_error on other errors
  void send(const void *const data, const std::size_t length, const CanId id,
            const std::chrono::nanoseconds timeout =
                std::chrono::nanoseconds::zero()) const;

  /// Receive CAN data
  /// \param[out] data A buffer to be written with data bytes. Must be at least
  /// 8 bytes in size
  /// \param[in] timeout Maximum duration to wait for data on the file
  /// descriptor. Negative
  ///                    durations are treated the same as zero timeout
  /// \return The CanId for the received can_frame, with length appropriately
  /// populated
  /// \throw SocketCanTimeout On timeout
  /// \throw std::runtime_error on other errors
  CanId receive(void *const data, const std::chrono::nanoseconds timeout =
                                      std::chrono::nanoseconds::zero()) const;

private:
  int32_t m_file_descriptor;

  void send_impl(const void *const data, const std::size_t length,
                 const CanId id, const std::chrono::nanoseconds timeout) const;
  // Wait for file descriptor to be available to send data via select()
  SOCKETCAN_LOCAL void wait_send(const std::chrono::nanoseconds timeout) const;
  SOCKETCAN_LOCAL void
  wait_receive(const std::chrono::nanoseconds timeout) const;
};
} // namespace socketcan
} // namespace drivers

#endif
