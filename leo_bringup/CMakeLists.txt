cmake_minimum_required(VERSION 2.8.3)
project(leo_bringup)

find_package(catkin)

catkin_package()

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY camera_info/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/camera_info)

install(DIRECTORY config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

catkin_install_python(PROGRAMS
    scripts/leo_system
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
