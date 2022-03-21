from enum import Enum

import time

import rospy
import rosnode
import rospkg

from leo_msgs.msg import Imu, WheelStates
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from .utils import write_flush
from .board import BoardType, determine_board, check_firmware_version
from .utils import CSIColor, parse_yaml


class TestMode(Enum):
    ENCODER = "encoder"
    TORQUE = "torque"
    IMU = "imu"
    BATTERY = "battery"
    ALL = "all"

    def __str__(self):
        return self.value


class HardwareTester:

    is_new_imu_data = 0
    is_new_wheel_data = 0
    is_new_battery_data = 0
    is_wheel_loaded = 1

    imu_data = Imu()
    wheel_data = WheelStates()
    battery_data = Float32()

    def __init__(self) -> None:

        self.rospack = rospkg.RosPack()
        self.path = self.rospack.get_path("leo_fw") + "/test_data/"

        ###Publisher

        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.cmd_pwmfl_pub = rospy.Publisher(
            "firmware/wheel_FL/cmd_pwm_duty", Float32, queue_size=1
        )
        self.cmd_pwmrl_pub = rospy.Publisher(
            "firmware/wheel_RL/cmd_pwm_duty", Float32, queue_size=1
        )
        self.cmd_pwmfr_pub = rospy.Publisher(
            "firmware/wheel_FR/cmd_pwm_duty", Float32, queue_size=1
        )
        self.cmd_pwmrr_pub = rospy.Publisher(
            "firmware/wheel_RR/cmd_pwm_duty", Float32, queue_size=1
        )
        self.cmd_velfl_pub = rospy.Publisher(
            "firmware/wheel_FL/cmd_velocity", Float32, queue_size=1
        )
        self.cmd_velrl_pub = rospy.Publisher(
            "firmware/wheel_RL/cmd_velocity", Float32, queue_size=1
        )
        self.cmd_velfr_pub = rospy.Publisher(
            "firmware/wheel_FR/cmd_velocity", Float32, queue_size=1
        )
        self.cmd_velrr_pub = rospy.Publisher(
            "firmware/wheel_RR/cmd_velocity", Float32, queue_size=1
        )

        ###Subscriber

        self.battery_sub = rospy.Subscriber(
            "firmware/battery", Float32, self.battery_callback
        )
        self.wheel_sub = rospy.Subscriber(
            "firmware/wheel_states", WheelStates, self.wheel_callback
        )
        self.imu_sub = rospy.Subscriber("firmware/imu", Imu, self.imu_callback)

    def battery_callback(self, data: Float32) -> None:
        self.battery_data = data
        self.is_new_battery_data = 1

    def imu_callback(self, data: Imu) -> None:
        self.imu_data = data
        self.is_new_imu_data = 1

    def wheel_callback(self, data: WheelStates) -> None:
        self.wheel_data = data
        self.is_new_wheel_data = 1

    def check_motor_load(self) -> None:
        speed_limit = 1

        for pwm in range(30):

            self.cmd_pwmfl_pub.publish(Float32(pwm))
            self.cmd_pwmfr_pub.publish(Float32(pwm))
            self.cmd_pwmrl_pub.publish(Float32(-pwm))
            self.cmd_pwmrr_pub.publish(Float32(-pwm))

            if (
                self.wheel_data.velocity[0] > speed_limit
                and self.wheel_data.velocity[1] < -speed_limit
                and self.wheel_data.velocity[2] > speed_limit
                and self.wheel_data.velocity[3] < -speed_limit
            ):
                self.is_wheel_loaded = 0
                break

            rospy.sleep(0.2)

        if self.is_wheel_loaded:
            print(CSIColor.OKGREEN + "LOAD" + CSIColor.ENDC)
        else:
            print(CSIColor.OKGREEN + "UNLOAD" + CSIColor.ENDC)

        self.cmd_pwmfl_pub.publish(Float32(0))
        self.cmd_pwmfr_pub.publish(Float32(0))
        self.cmd_pwmrl_pub.publish(Float32(0))
        self.cmd_pwmrr_pub.publish(Float32(0))

    def check_encoder(self) -> None:
        is_error = 0
        is_error_tab = [0, 0, 0, 0]

        if self.is_wheel_loaded == 1:
            wheel_valid = parse_yaml(self.path + "encoder_load.yaml")
        elif self.is_wheel_loaded == 0:
            wheel_valid = parse_yaml(self.path + "encoder.yaml")

        for wheel_test in wheel_valid["tests"]:

            self.cmd_velfl_pub.publish(wheel_test["velocity"])
            self.cmd_velfr_pub.publish(wheel_test["velocity"])
            self.cmd_velrl_pub.publish(wheel_test["velocity"])
            self.cmd_velrr_pub.publish(wheel_test["velocity"])

            rospy.sleep(wheel_test["time"])

            speed_min = wheel_test["velocity"] - wheel_test["offset"]
            speed_max = wheel_test["velocity"] + wheel_test["offset"]

            for i in range(0, 4):
                if not speed_min < self.wheel_data.velocity[i] < speed_max:
                    is_error = 1
                    is_error_tab[i] = 1

        self.cmd_velfl_pub.publish(Float32(0))
        self.cmd_velfr_pub.publish(Float32(0))
        self.cmd_velrl_pub.publish(Float32(0))
        self.cmd_velrr_pub.publish(Float32(0))

        if is_error == 1:
            error_msg = "ERROR WHEEL ENCODER "
            error_msg += str(is_error_tab)
            print(CSIColor.FAIL + error_msg + CSIColor.ENDC)
            return

        print(CSIColor.OKGREEN + "PASSED" + CSIColor.ENDC)
        return

    def check_torque(self) -> None:
        is_error = 0
        is_error_tab = [0, 0, 0, 0]

        if self.is_wheel_loaded == 1:
            torque_valid = parse_yaml(self.path + "torque_load.yaml")
        elif self.is_wheel_loaded == 0:
            torque_valid = parse_yaml(self.path + "torque.yaml")

        for torque_test in torque_valid["tests"]:
            self.cmd_pwmfl_pub.publish(torque_test["pwm"])
            self.cmd_pwmfr_pub.publish(torque_test["pwm"])
            self.cmd_pwmrl_pub.publish(-torque_test["pwm"])
            self.cmd_pwmrr_pub.publish(-torque_test["pwm"])

            rospy.sleep(torque_test["time"])

            torque_min = torque_test["torque"]
            torque_max = torque_test["torque"] + 1

            for i in range(0, 4):
                if not torque_min < self.wheel_data.torque[i] < torque_max:
                    is_error = 1
                    is_error_tab[i] = 1

        self.cmd_pwmfl_pub.publish(Float32(0))
        self.cmd_pwmfr_pub.publish(Float32(0))
        self.cmd_pwmrl_pub.publish(Float32(0))
        self.cmd_pwmrr_pub.publish(Float32(0))

        if is_error == 1:
            error_msg = "ERROR WHEEL TORQUE "
            error_msg += str(is_error_tab)
            print(CSIColor.FAIL + error_msg + CSIColor.ENDC)
            return

        print(CSIColor.OKGREEN + "PASSED" + CSIColor.ENDC)
        return

    def check_imu(self) -> None:
        msg_cnt = 0
        time_now = time.time()
        imu_valid = parse_yaml(self.path + "imu.yaml")

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
                print(CSIColor.WARNING + "TIMEOUT" + CSIColor.ENDC)
                return

            if self.is_new_imu_data == 1:
                time_now = time.time()
                self.is_new_imu_data = 0
                msg_cnt += 1

                if not (
                    accel_x - accel_del < self.imu_data.accel_x < accel_x + accel_del
                    and accel_y - accel_del
                    < self.imu_data.accel_y
                    < accel_y + accel_del
                    and accel_z - accel_del
                    < abs(self.imu_data.accel_z)
                    < accel_z + accel_del
                    and gyro_x - gyro_del < self.imu_data.gyro_x < gyro_x + gyro_del
                    and gyro_y - gyro_del < self.imu_data.gyro_y < gyro_y + gyro_del
                    and gyro_z - gyro_del < self.imu_data.gyro_z < gyro_z + gyro_del
                ):
                    print(CSIColor.FAIL + "INVALID DATA" + CSIColor.ENDC)
                    return

        print(CSIColor.OKGREEN + "PASSED" + CSIColor.ENDC)
        return

    def check_battery(self) -> None:
        msg_cnt = 0
        time_now = time.time()
        batt_valid = parse_yaml(self.path + "battery.yaml")

        while msg_cnt < 50:
            if time_now + batt_valid["battery"]["timeout"] < time.time():
                print(CSIColor.WARNING + "TIMEOUT" + CSIColor.ENDC)
                return

            if self.is_new_battery_data == 1:
                time_now = time.time()
                self.is_new_battery_data = 0
                msg_cnt += 1

                if self.battery_data.data <= batt_valid["battery"]["voltage_min"]:
                    print(CSIColor.FAIL + "LOW VOLTAGE" + CSIColor.ENDC)
                    return
                if self.battery_data.data >= batt_valid["battery"]["voltage_max"]:
                    print(CSIColor.FAIL + "HIGH VOLTAGE" + CSIColor.ENDC)
                    return

        print(CSIColor.OKGREEN + "PASSED" + CSIColor.ENDC)
        return

    # pylint: disable=too-many-branches,too-many-statements
    def test_hw(
        self,
        hardware: TestMode = TestMode.ALL,
    ) -> None:

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

        if serial_node_active:
            write_flush("--> Trying to determine board type.. ")

            board_type = determine_board()

            if board_type is not None:
                print("SUCCESS")
            else:
                print("FAIL")

        current_firmware_version = "<unknown>"

        if serial_node_active:
            write_flush("--> Trying to check the current firmware version.. ")

            current_firmware_version = check_firmware_version()

            if current_firmware_version != "<unknown>":
                print("SUCCESS")
            else:
                print("FAIL")

        if current_firmware_version == "<unknown>" or board_type is None:
            print(
                "Can not determine firmware version or board type. "
                "Flash firmware and try to rerun the script"
            )
            return

        if serial_node_active:
            write_flush("--> Initializing ROS node.. ")
            rospy.init_node("leo_core_validation", anonymous=True)
            print("OK")

        print(f"Firmware version: {current_firmware_version}")
        if board_type == BoardType.CORE2:
            print(f"Board type: Core2ROS")
        elif board_type == BoardType.LEOCORE:
            print(f"Board type: LeoCore")
        else:
            print(f"Can not determine board version")
            return

        if hardware in (TestMode.ALL, TestMode.BATTERY):
            write_flush("--> Battery validation.. ")
            self.check_battery()

        if hardware in (TestMode.ALL, TestMode.IMU) and board_type == BoardType.LEOCORE:
            write_flush("--> IMU validation.. ")
            self.check_imu()

        if hardware in (TestMode.ALL, TestMode.TORQUE, TestMode.ENCODER):
            write_flush("--> Motors load test.. ")
            self.check_motor_load()

        if hardware in (TestMode.ALL, TestMode.ENCODER):
            write_flush("--> Encoders validation.. ")
            self.check_encoder()

        if (
            hardware in (TestMode.ALL, TestMode.TORQUE)
            and board_type == BoardType.LEOCORE
        ):
            write_flush("--> Torque sensors validation.. ")
            self.check_torque()
