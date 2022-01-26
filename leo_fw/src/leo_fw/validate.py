from typing import Optional
from enum import Enum

import rospy
import rosgraph
import rosnode
import rospkg

import yaml

from .utils import *
from .board import BoardType, determine_board, check_firmware_version
from leo_msgs.msg import Imu, WheelOdom, WheelStates

rospack = rospkg.RosPack()

class TestMode(Enum):
    GROUND_WHEEL = "ground_wheel"
    FLOAT_WHEEL = "float_wheel"
    NO_WHEEL = "no_wheel"

    def __str__(self):
        return self.value

class TestHW(Enum):
    HBRIDGE = "h_bridge"
    ENCODER = "encoder"
    TORQUE = "torque"
    IMU = "imu"
    BATTERY = "battery"
    ALL = "all"
    
    def __str__(self):
        return self.value

###YAML PARSER

def parse_yaml(file_path):
    with open(file_path, "r") as stream:
        try:
            return yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)

    return tuple()

###MOTOR ENCODER TEST

def check_encoder():
    path = rospack.get_path('leo_fw')
    encoder_valid = parse_yaml(path+"/validate/motor.yaml")
    print(encoder_valid)

###MOTOR TORQUE TEST

def check_torque():
    path = rospack.get_path('leo_fw')
    torque_valid = parse_yaml(path+"/validate/motor.yaml")
    print(torque_valid)

###IMU TEST

def check_imu():
    path = rospack.get_path('leo_fw')
    imu_valid = parse_yaml(path+"/validate/imu.yaml")
    print(imu_valid)
    write_flush("--> Checking IMU.. ")

    

###BATTERY TEST

def check_battery():
    path = rospack.get_path('leo_fw')
    batt_valid = parse_yaml(path+"/validate/battery.yaml")
    print(batt_valid)

#####################################################

def validate_hw(
    hardware: Optional[TestHW] = TestHW.ALL,
    mode: Optional[TestMode] = TestMode.NO_WHEEL,
    rosbag: bool = False,
):

    write_flush("--> Checking if ROS Master is online.. ")

    if rosgraph.is_master_online():
        print("YES")
        master_online = True
    else:
        print("NO")
        master_online = False
        print(
            "ROS Master is not running. "
            "Will not be able to validate hardware."
            "Try to restart leo.service or reboot system."
        )
        return

    #####################################################

    if master_online:
        write_flush("--> Checking if rosserial node is active.. ")

        if rospy.resolve_name("serial_node") in rosnode.get_node_names():
            print("YES")
            serial_node_active = True
        else:
            print("NO")
            serial_node_active = False
            print(
                "Rosserial node is not active. "
                "Will not be able to validate hardware."
                "Try to restart leo.service or reboot system."
            )
            return

    #####################################################

    if master_online and serial_node_active:
        write_flush("--> Trying to determine board type.. ")

        board_type = determine_board()

        if board_type is not None:
            print("SUCCESS")
        else:
            print("FAIL")


    #####################################################

    current_firmware_version = "<unknown>"

    if master_online and serial_node_active:
        write_flush("--> Trying to check the current firmware version.. ")

        current_firmware_version = check_firmware_version()

        if current_firmware_version != "<unknown>":
            print("SUCCESS")
        else:
            print("FAIL")

    #####################################################

    if current_firmware_version == "<unknown>" or board_type is None:
        print(
            "Can not determine firmware version or board type. "
            "Flash firmware and try to rerun the script"
        )
        return

    #####################################################

    if master_online and serial_node_active:
        write_flush("--> Initializing ROS node.. ")
        rospy.init_node("leo_core_validation", anonymous=True)
        print("DONE")

    #####################################################

    print(f"Firmware version: {current_firmware_version}")
    if board_type == BoardType.CORE2:
        print(f"Board type: Core2ROS")
    elif board_type == BoardType.LEOCORE:
        print(f"Board type: LeoCore")

    #####################################################

    if not query_yes_no("Run test?"):
        return

    #####################################################

    check_encoder()

    check_torque()

    check_imu()

    check_battery()
  
    #####################################################

