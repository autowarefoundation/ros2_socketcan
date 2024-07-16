^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2_socketcan
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.3.0 (2024-07-16)
------------------
* Jazzy release
* fix: add missing header (`#42 <https://github.com/autowarefoundation/ros2_socketcan/issues/42>`_)
* Allow remapping of the canbus topics (`#39 <https://github.com/autowarefoundation/ros2_socketcan/issues/39>`_)
* Contributors: Joshua Whitley, Tim Clephas

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
* SocketCAN filters (`#25 <https://github.com/autowarefoundation/ros2_socketcan/issues/25>`_)
  * SocketCAN filters
  Filters can be set using launch parameter.
  A list of pairs (can id and mask) are fetched and applied to
  socket filter.
  Added unit test for filters application and fuctionality.
  * Full support of SocketCAN filters
  SocketCAN filters can be now
  with string description used
  by candump utility.
  Added support for error masks
  and joined CAN filters.
  Instead of list of integers, receiver
  node now uses string parameter
  in order to receive filters. Filters will
  now be parsed and setup during
  configuration.
  Added unit test for parsing and
  updated filters unit test.
  * Reference to man-pages docs of filters syntax
  Added links referencing man-pages docs for candump,
  containing more information about socketcan filters
  syntax used. Links were added to doxygen documentation
  of filters parsing method and to launch argument
  description.
* Fix unit conversion bug in to_timeval() (`#24 <https://github.com/autowarefoundation/ros2_socketcan/issues/24>`_)
* Reorganize folders for adding ros2_socketcan_msgs (`#23 <https://github.com/autowarefoundation/ros2_socketcan/issues/23>`_)
  Reorganize folders to permit adding a msgs package.
* Contributors: Joshua Whitley, Marcel Dudek, ljuricic

1.1.0 (2022-02-03)
------------------
* Added bus time (`#12 <https://github.com/autowarefoundation/ros2_socketcan/issues/12>`_)
  * added the ability to get the bus time for the can packet, versus using ros time when received; packs bus time as part of the can id struct
  * cleanup; cast fix
  * chore: apply uncrustify
  * chore: fix include order for cpplint
  Co-authored-by: wep21 <border_goldenmarket@yahoo.co.jp>
* Merge pull request `#10 <https://github.com/autowarefoundation/ros2_socketcan/issues/10>`_ from wep21/ci-galactic
  Add galactic into action
* Add galactic into action
* Contributors: Andrew Saba, Daisuke Nishimatsu, Joshua Whitley

1.0.0 (2021-04-01)
------------------
* Initial release
* Initial port from Autoware.Auto
* Initial commit
* Contributors: Joshua Whitley, Kenji Miyake, wep21
