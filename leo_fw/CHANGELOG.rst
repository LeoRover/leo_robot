^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package leo_fw
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.3 (2022-01-18)
------------------
* Fix some problems reported by catkin_lint
* Add tf_frame_prefix to firmware message converter (`#4 <https://github.com/LeoRover/leo_robot/issues/4>`_)
* Contributors: Bitterisland6, Błażej Sowa

2.0.2 (2022-01-05)
------------------
* Update leocore firmware to version 1.0.2
* Fix default selection in board prompt

2.0.1 (2021-12-28)
------------------
* Update leocore firmware to version 1.0.1
* Change the new board name to LeoCore (codename leocore)
* Make the flashing scripts work when run under ROS namespace

2.0.0 (2021-12-12)
------------------
* Add Leo Hat firmware binary
* Update package description
* Get rid of python2 dependencies
* Rename core2 firmware binary
* Allow explicitly specifying board type in flash script
* Add functions for determining board type and firmware version
* Use format strings when applicable
* Add initial Leo Hat support
* Add firmware_message_parser node

1.2.1 (2021-09-28)
------------------
* Reformat python code with black
* Make the code python 2-3 compatible

1.2.0 (2021-04-23)
------------------
* Bundle a new firmware binary release (v1.2.0)
* Add flash script to CMakeLists
* Add flash script for flashing custom firmware
* Add arguments to the flash_firmware function
* Update author and maintainer info

1.1.3 (2020-12-18)
------------------

1.1.2 (2020-11-25)
------------------

1.1.1 (2020-10-19)
------------------
* Add setuptools to buildtool dependencies
* Fix CMakeLists, install update script

1.1.0 (2020-10-16)
------------------
* Add leo_fw package

1.0.2 (2020-09-23)
------------------
