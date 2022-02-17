from asyncio.constants import ACCEPT_RETRY_DELAY
from errno import ELIBBAD
from pickle import TRUE
from posixpath import isabs
from tabnanny import check
from typing import Optional
from enum import Enum

import time

import rospy
import rosgraph
import rosnode
import rospkg

import yaml

from .utils import *
from .board import BoardType, determine_board, check_firmware_version
from leo_msgs.msg import Imu, WheelOdom, WheelStates
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

rospack = rospkg.RosPack()

###YAML PARSER


def parse_yaml(file_path):
    with open(file_path, "r") as stream:
        try:
            return yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)

    return {}


path = rospack.get_path("leo_fw")
motor_valid = parse_yaml(path + "/validate/motor.yaml")

imuData = Imu()
wheelData = WheelStates()
batteryData = Float32()

isNewImuData = 0
isNewWheelData = 0
isNewBatteryData = 0

isWheelLoaded = 1
wheelSpeedLimit = 0.05


class TestHW(Enum):
    ENCODER = "encoder"
    TORQUE = "torque"
    IMU = "imu"
    BATTERY = "battery"
    ALL = "all"

    def __str__(self):
        return self.value


class bcolors:
    OKGREEN = "\033[92m"
    WARNING = "\033[93m"
    FAIL = "\033[91m"
    ENDC = "\033[0m"


def batteryCallback(data):
    global isNewBatteryData
    global batteryData
    batteryData = data
    isNewBatteryData = 1


def imuCallback(data):
    global isNewImuData
    global imuData
    imuData = data
    isNewImuData = 1


def wheelCallback(data):
    global wheelData
    global isNewWheelData
    wheelData = data
    isNewWheelData = 1

    #####################################################


###Publisher

cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

cmd_pwmFL_pub = rospy.Publisher("firmware/wheel_FL/cmd_pwm_duty", Float32, queue_size=1)
cmd_pwmRL_pub = rospy.Publisher("firmware/wheel_RL/cmd_pwm_duty", Float32, queue_size=1)
cmd_pwmFR_pub = rospy.Publisher("firmware/wheel_FR/cmd_pwm_duty", Float32, queue_size=1)
cmd_pwmRR_pub = rospy.Publisher("firmware/wheel_RR/cmd_pwm_duty", Float32, queue_size=1)

cmd_velFL_pub = rospy.Publisher("firmware/wheel_FL/cmd_velocity", Float32, queue_size=1)
cmd_velRL_pub = rospy.Publisher("firmware/wheel_RL/cmd_velocity", Float32, queue_size=1)
cmd_velFR_pub = rospy.Publisher("firmware/wheel_FR/cmd_velocity", Float32, queue_size=1)
cmd_velRR_pub = rospy.Publisher("firmware/wheel_RR/cmd_velocity", Float32, queue_size=1)

###Subscriber

battery_sub = rospy.Subscriber("firmware/battery", Float32, batteryCallback)
wheel_sub = rospy.Subscriber("firmware/wheel_states", WheelStates, wheelCallback)
imu_sub = rospy.Subscriber("firmware/imu", Imu, imuCallback)

###WHEEL LOAD TEST


def check_motor_load():
    global isWheelLoaded
    global wheelData
    speed_limit = 1

    for x in range(30):

        cmd_pwmFL_pub.publish(Float32(x))
        cmd_pwmFR_pub.publish(Float32(x))
        cmd_pwmRL_pub.publish(Float32(-x))
        cmd_pwmRR_pub.publish(Float32(-x))

        if (
            wheelData.velocity[0] > speed_limit
            and wheelData.velocity[1] < -speed_limit
            and wheelData.velocity[2] > speed_limit
            and wheelData.velocity[3] < -speed_limit
        ):
            isWheelLoaded = 0
            break

        rospy.sleep(0.2)
    if isWheelLoaded:
        print(bcolors.OKGREEN + "LOAD" + bcolors.ENDC)
    else:
        print(bcolors.OKGREEN + "UNLOAD" + bcolors.ENDC)

    cmd_pwmFL_pub.publish(Float32(0))
    cmd_pwmFR_pub.publish(Float32(0))
    cmd_pwmRL_pub.publish(Float32(0))
    cmd_pwmRR_pub.publish(Float32(0))


###MOTOR ENCODER TEST


def check_encoder():
    global isWheelLoaded
    global wheelData

    is_error = 0
    is_error_tab = [0, 0, 0, 0]
    error_msg = "ERROR WHEEL "

    if isWheelLoaded == 1:
        wheel_valid = parse_yaml(path + "/validate/motor_load.yaml")
    elif isWheelLoaded == 0:
        wheel_valid = parse_yaml(path + "/validate/motor.yaml")

    for x in wheel_valid.items():
        cmd_velFL_pub.publish(x[1]["velocity"])
        cmd_velFR_pub.publish(x[1]["velocity"])
        cmd_velRL_pub.publish(x[1]["velocity"])
        cmd_velRR_pub.publish(x[1]["velocity"])

        rospy.sleep(x[1]["time"])

        speed_min = x[1]["velocity"] - x[1]["offset"]
        speed_max = x[1]["velocity"] + x[1]["offset"]

        for i in range(0, 4):
            if not speed_min < wheelData.velocity[i] < speed_max:
                is_error = 1
                is_error_tab[i] = 1

    cmd_velFL_pub.publish(Float32(0))
    cmd_velFR_pub.publish(Float32(0))
    cmd_velRL_pub.publish(Float32(0))
    cmd_velRR_pub.publish(Float32(0))

    if is_error == 1:
        error_msg += str(is_error_tab)
        print(bcolors.FAIL + error_msg + bcolors.ENDC)
        return 0
    else:
        print(bcolors.OKGREEN + "PASSED" + bcolors.ENDC)
        return 1


###MOTOR TORQUE TEST


def check_torque():

    print(motor_valid)


###IMU TEST


def check_imu():
    global isNewImuData
    global imuData

    msg_cnt = 0
    time_now = time.time()
    imu_valid = parse_yaml(path + "/validate/imu.yaml")

    accel_del = imu_valid["imu"]["accel_del"]
    accel_x = imu_valid["imu"]["accel_x"]
    accel_y = imu_valid["imu"]["accel_y"]
    accel_z = imu_valid["imu"]["accel_z"]

    gyro_del = imu_valid["imu"]["gyro_del"]
    gyro_x = imu_valid["imu"]["gyro_x"]
    gyro_y = imu_valid["imu"]["gyro_y"]
    gyro_z = imu_valid["imu"]["gyro_z"]

    while msg_cnt < 50:
        if time_now + imu_valid["imu"]["timeout"] < time.time():
            print(bcolors.WARNING + "TIMEOUT" + bcolors.ENDC)
            return 0

        if isNewImuData == 1:
            time_now = time.time()
            isNewImuData = 0
            msg_cnt += 1

            if not (
                accel_x - accel_del < imuData.accel_x < imuData.accel_x + accel_del
                and accel_y - accel_del < imuData.accel_y < accel_y + accel_del
                and accel_z - accel_del < imuData.accel_z < accel_z + accel_del
                and gyro_x - gyro_del < imuData.gyro_x < gyro_x + gyro_del
                and gyro_y - gyro_del < imuData.gyro_y < gyro_y + gyro_del
                and gyro_z - gyro_del < imuData.gyro_z < gyro_z + gyro_del
            ):
                print(bcolors.FAIL + "INVALID DATA" + bcolors.ENDC)
                return 0

    print(bcolors.OKGREEN + "PASSED" + bcolors.ENDC)
    return 1


###BATTERY TEST


def check_battery():
    global isNewBatteryData
    global batteryData
    msg_cnt = 0
    time_now = time.time()
    batt_valid = parse_yaml(path + "/validate/battery.yaml")

    while msg_cnt < 50:
        if time_now + batt_valid["battery"]["timeout"] < time.time():
            print("TIMEOUT")
            return 0

        if isNewBatteryData == 1:
            time_now = time.time()
            isNewBatteryData = 0
            msg_cnt += 1

            if batteryData.data <= batt_valid["battery"]["voltage_min"]:
                print(bcolors.FAIL + "LOW VOLTAGE" + bcolors.ENDC)
                return 0
            elif batteryData.data >= batt_valid["battery"]["voltage_max"]:
                print(bcolors.FAIL + "HIGH VOLTAGE" + bcolors.ENDC)
                return 0

    print(bcolors.OKGREEN + "PASSED" + bcolors.ENDC)
    return 1


#####################################################


def validate_hw(
    hardware: Optional[TestHW] = TestHW.ALL,
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

    if hardware == TestHW.ALL or hardware == TestHW.BATTERY:
        write_flush("--> Battery validation.. ")
        check_battery()

    if hardware == TestHW.ALL or hardware == TestHW.IMU:
        write_flush("--> IMU validation.. ")
        check_imu()

    if (
        hardware == TestHW.ALL
        or hardware == TestHW.TORQUE
        or hardware == TestHW.ENCODER
    ):
        write_flush("--> Motors load test.. ")
        check_motor_load()

    if hardware == TestHW.ALL or hardware == TestHW.ENCODER:
        write_flush("--> Encoders validation.. ")
        check_encoder()

    if hardware == TestHW.ALL or hardware == TestHW.TORQUE:
        write_flush("--> Torque sensors validation.. ")
        check_torque()

    #####################################################

    cmd_vel_pub.unregister()

    cmd_pwmFL_pub.unregister()

    cmd_pwmRL_pub.unregister()

    cmd_pwmFR_pub.unregister()

    cmd_pwmRR_pub.unregister()
