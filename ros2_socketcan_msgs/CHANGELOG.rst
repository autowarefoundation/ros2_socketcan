^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2_socketcan_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.3.0 (2024-07-16)
------------------
* Jazzy release

1.2.0 (2023-03-03)
------------------
* Add CAN FD Support (`#28 <https://github.com/autowarefoundation/ros2_socketcan/issues/28>`_)
  * Add FD support to ROS2 interface.
  * Add FD send/receive.
  * Missed some fields in Frame msg.
  * Make standard and FD mutually exclusive.
  * Enable runtime checks for standard vs FD.
  * Try to minimize API changes.
  * Fix receive_id/fd_receive_id mix-up.
  * Make new message FD-specific.
  * Use FdFrame message.
  * Remove unused functions.
  * Always resize fd frame buffer to 64 before receive.
  * Add enable_can_fd to socket_can_bridge.launch.xml.
  ---------
* Adding ros2_socketcan_msgs (`#26 <https://github.com/autowarefoundation/ros2_socketcan/issues/26>`_)
* Contributors: Joshua Whitley

* Add CAN FD Support (`#28 <https://github.com/autowarefoundation/ros2_socketcan/issues/28>`_)
  * Add FD support to ROS2 interface.
  * Add FD send/receive.
  * Missed some fields in Frame msg.
  * Make standard and FD mutually exclusive.
  * Enable runtime checks for standard vs FD.
  * Try to minimize API changes.
  * Fix receive_id/fd_receive_id mix-up.
  * Make new message FD-specific.
  * Use FdFrame message.
  * Remove unused functions.
  * Always resize fd frame buffer to 64 before receive.
  * Add enable_can_fd to socket_can_bridge.launch.xml.
  ---------
* Adding ros2_socketcan_msgs (`#26 <https://github.com/autowarefoundation/ros2_socketcan/issues/26>`_)
* Contributors: Joshua Whitley

1.1.0 (2022-02-03)
------------------

1.0.0 (2021-04-01)
------------------
