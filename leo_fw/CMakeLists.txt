cmake_minimum_required(VERSION 3.0.2)
project(leo_fw)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

install(
  DIRECTORY firmware
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(
  PROGRAMS scripts/update
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)