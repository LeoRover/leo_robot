import os
import subprocess

from typing import Optional

import rospy
import rospkg
import rosgraph
import rosnode
import rosservice

from rosmon_msgs.srv import StartStop, StartStopRequest

from .utils import is_tool, write_flush, query_yes_no, prompt_options
from .board import BoardType, determine_board, check_firmware_version


def flash_core2(bootloader_path: str, firmware_path: str):
    print("--> Disabling read/write protections and erasing flash")
    subprocess.check_call("stm32loader -c rpi -f F4 -R -u -W", shell=True)

    print("--> Erasing flash")
    subprocess.check_call("stm32loader -c rpi -f F4 -R -e", shell=True)

    print("--> Flashing bootloader")
    subprocess.check_call(
        f"stm32loader -c rpi -f F4 -R -w -v {bootloader_path}", shell=True
    )

    print("--> Flashing firmware")
    subprocess.check_call(
        f"stm32loader -c rpi -f F4 -R -a 0x08010000 -w -v {firmware_path}", shell=True
    )

    print("--> Flashing completed!")


def flash_leocore(firmware_path: str):
    print("--> Disabling flash read/write protections")
    subprocess.check_call("stm32loader -c rpi -f F4 -u -W", shell=True)

    print("--> Erasing flash")
    subprocess.check_call("stm32loader -c rpi -f F4 -e", shell=True)

    print("--> Flashing firmware")
    subprocess.check_call(f"stm32loader -c rpi -f F4 -w -v {firmware_path}", shell=True)

    print("--> Flashing completed!")


# pylint: disable=too-many-branches,too-many-statements
def flash_firmware(
    firmware_path: Optional[str] = None,
    board_type: Optional[BoardType] = None,
    check_version: bool = True,
):
    write_flush("--> Checking if stm32loader is installed.. ")

    if is_tool("stm32loader"):
        print("YES")
    else:
        print("NO")
        print(
            "ERROR: Cannot find the stm32loader tool. "
            "Make sure the python3-stm32loader package is installed."
        )
        return

    #####################################################

    write_flush("--> Checking if ROS Master is online.. ")

    if rosgraph.is_master_online():
        print("YES")
        master_online = True
    else:
        print("NO")
        master_online = False
        if check_version:
            print(
                "ROS Master is not running. "
                "Will not be able to check the current firmware version."
            )
            if not query_yes_no("Are you sure you want to continue?", default="no"):
                return

    #####################################################

    if master_online:
        write_flush("--> Initializing ROS node.. ")
        rospy.init_node("firmware_flasher", anonymous=True)
        print("DONE")

    #####################################################

    if master_online:
        write_flush("--> Checking if rosserial node is active.. ")

        if rospy.resolve_name("serial_node") in rosnode.get_node_names():
            print("YES")
            serial_node_active = True
        else:
            print("NO")
            serial_node_active = False
            if check_version:
                print(
                    "Rosserial node is not active. "
                    "Will not be able to check the current firmware version."
                )
                if not query_yes_no("Are you sure you want to continue?", default="no"):
                    return

    #####################################################

    if master_online and serial_node_active and board_type is None:
        write_flush("--> Trying to determine board type.. ")

        board_type = determine_board()

        if board_type is not None:
            print("SUCCESS")
        else:
            print("FAIL")

    #####################################################

    current_firmware_version = "<unknown>"

    if check_version and master_online and serial_node_active:
        write_flush("--> Trying to check the current firmware version.. ")

        current_firmware_version = check_firmware_version()

        if current_firmware_version != "<unknown>":
            print("SUCCESS")
        else:
            print("FAIL")

    #####################################################

    if master_online and serial_node_active:
        write_flush("--> Checking if rosmon service is available.. ")

        if rospy.resolve_name("rosmon/start_stop") in rosservice.get_service_list():
            start_stop = rospy.ServiceProxy("rosmon/start_stop", StartStop)
            print("YES")
            rosmon_available = True
        else:
            print("NO")
            rosmon_available = False

    #####################################################

    if board_type is None:
        print("Was not able to determine the board type. Choose the board manually: ")

        board_type = prompt_options(
            [
                ("LeoCore", BoardType.LEOCORE),
                ("Husarion CORE2", BoardType.CORE2),
            ]
        )

    #####################################################

    if firmware_path is not None:
        firmware_version = "<unknown>"
    else:
        if board_type == BoardType.CORE2:
            firmware_version = "2.0.0"
        elif board_type == BoardType.LEOCORE:
            firmware_version = "1.1.2"

    print(f"Current firmware version: {current_firmware_version}")
    print(f"Version of the firmware to flash: {firmware_version}")

    if not query_yes_no("Flash the firmware?"):
        return

    #####################################################

    if master_online and serial_node_active:
        if rosmon_available:
            write_flush("--> Stopping the rosserial node.. ")
            start_stop(
                "serial_node", rospy.get_namespace().rstrip("/"), StartStopRequest.STOP
            )
            rospy.sleep(1)
            print("DONE")
        else:
            print(
                "WARNING: rosserial node is active, but rosmon service "
                "is not available. You have to manually stop rosserial node "
                "before flashing the firmware."
            )
            if not query_yes_no("Continue?", default="no"):
                return

    #####################################################

    if board_type == BoardType.CORE2:
        bootloader_path = os.path.join(
            rospkg.RosPack().get_path("leo_fw"),
            "firmware/bootloader_1_0_0_core2.bin",
        )

        if firmware_path is None:
            firmware_path = os.path.join(
                rospkg.RosPack().get_path("leo_fw"),
                "firmware/core2_firmware.bin",
            )

        flash_core2(bootloader_path, firmware_path)

    elif board_type == BoardType.LEOCORE:
        if firmware_path is None:
            firmware_path = os.path.join(
                rospkg.RosPack().get_path("leo_fw"),
                "firmware/leocore_firmware.bin",
            )

        flash_leocore(firmware_path)

    #####################################################

    if master_online and serial_node_active:
        if rosmon_available:
            write_flush("--> Starting the rosserial node.. ")
            start_stop(
                "serial_node", rospy.get_namespace().rstrip("/"), StartStopRequest.START
            )
            print("DONE")
        else:
            print("You can now start the rosserial node.")
