cmake_minimum_required(VERSION 3.0.2)
project(leo_fw)

find_package(catkin REQUIRED COMPONENTS
  leo_msgs
  nav_msgs
  roscpp
  sensor_msgs
)

find_package(yaml-cpp REQUIRED)

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
  ${YAML_CPP_INCLUDE_DIRS}
)


add_executable(firmware_message_converter
  src/firmware_message_converter.cpp
)

target_link_libraries(firmware_message_converter
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

install(
  DIRECTORY 
    firmware
    test_data
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(
  PROGRAMS 
    scripts/update
    scripts/flash
    scripts/test_hw
    scripts/calibrate_imu
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS firmware_message_converter
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)