^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package leo_bringup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.1 (2020-10-19)
------------------

1.1.0 (2020-10-16)
------------------
* Update package description
* Increase cmake minimum version
* Change CMakeLists formatting
* Add xacro to dependencies, remove raspicam_node from dependencies

1.0.2 (2020-09-23)
------------------
* Changed camera frame name to camera_optical_frame

1.0.1 (2020-04-22)
------------------
* used relative topic names in leo_system node

1.0.0 (2020-03-24)
------------------
* fixed leo_system shutdown command
* added configuration files for camera and core2 firmware
* added motors_model argument to the launch file

0.6.0 (2020-02-12)
------------------
* changed camera frame id to camera_frame
* added upload_description argument to leo_bringup.launch file

0.5.0 (2019-10-02)
------------------
* change camera frame_id
* add robot state publisher, change rosserial baudrate, add camera calibration file

0.4.0 (2019-09-04)
------------------
* don't use husarion cloud

0.3.0 (2019-07-11)
------------------
* set unregister timeout in rosbridge_server

0.2.0 (2019-07-10)
------------------
* remap topics, change stream quality

0.1.0 (2019-06-06)
------------------
* add LICENSE
* add leo_system script
* add install targets to CMakeLists
* add raspicam_node
* package cleanup
* Initial commit
