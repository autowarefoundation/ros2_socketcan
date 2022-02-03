^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2_socketcan
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
