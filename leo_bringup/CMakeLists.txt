cmake_minimum_required(VERSION 3.0.2)
project(leo_bringup)

find_package(catkin REQUIRED)

catkin_package()

install(
  DIRECTORY
    camera_info
    config
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(
  PROGRAMS scripts/leo_system
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
