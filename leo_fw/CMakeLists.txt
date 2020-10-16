cmake_minimum_required(VERSION 3.0.2)
project(leo_fw)

find_package(catkin REQUIRED)

catkin_package()

catkin_python_setup()

install(
  DIRECTORY firmware/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/firmware
)