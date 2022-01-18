cmake_minimum_required(VERSION 3.0.2)
project(leo_fw)

find_package(catkin REQUIRED COMPONENTS
  leo_msgs
  nav_msgs
  roscpp
  sensor_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    leo_msgs
    nav_msgs
    roscpp
    sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(firmware_message_converter
  src/firmware_message_converter.cpp
)

target_link_libraries(firmware_message_converter
  ${catkin_LIBRARIES}
)

install(
  DIRECTORY firmware
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(
  PROGRAMS 
    scripts/update
    scripts/flash
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS firmware_message_converter
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)