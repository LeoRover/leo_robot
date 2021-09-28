from __future__ import print_function

import os
import sys
import subprocess

import rospy
import rospkg
import rosgraph
import rosnode
import rosservice

from std_srvs.srv import Trigger
from rosmon_msgs.srv import StartStop, StartStopRequest

from .utils import is_tool, write_flush, query_yes_no


def flash_firmware(
    bootloader_path, firmware_path, firmware_version="<unknown>", check_version=True
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

    if master_online:
        write_flush("--> Initializing ROS node.. ")
        rospy.init_node("firmware_flasher", anonymous=True)
        print("DONE")

    if master_online:
        write_flush("--> Checking if rosserial node is active.. ")

        if "/serial_node" in rosnode.get_node_names():
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

    current_firmware_version = "<unknown>"

    if check_version and master_online and serial_node_active:
        write_flush("--> Checking the current firmware version.. ")

        if "/core2/get_firmware_version" in rosservice.get_service_list():
            get_firmware_version = rospy.ServiceProxy(
                "/core2/get_firmware_version", Trigger
            )
            current_firmware_version = get_firmware_version().message
            print("OK")
        else:
            print("FAIL")
            print(
                "WARNING: Could not get the current firmware version: "
                "/core2/get_firmware_version service is not available."
            )

    print("Current firmware version: {}".format(current_firmware_version))
    print("Version of the firmware to flash: {}".format(firmware_version))

    if master_online and serial_node_active:
        write_flush("--> Checking if rosmon service is available.. ")

        if "/rosmon/start_stop" in rosservice.get_service_list():
            start_stop = rospy.ServiceProxy("/rosmon/start_stop", StartStop)
            print("YES")
            rosmon_available = True
        else:
            print("NO")
            rosmon_available = False

    if not query_yes_no("Flash the firmware?"):
        return

    if master_online and serial_node_active:
        if rosmon_available:
            write_flush("--> Stopping the rosserial node.. ")
            start_stop("serial_node", "", StartStopRequest.STOP)
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

    print("--> Disabling flash write protection")
    subprocess.check_call("stm32loader -c rpi -f F4 -W", shell=True)

    print("--> Erasing flash and flashing bootloader")
    subprocess.check_call(
        "stm32loader -c rpi -f F4 -e -w -v {}".format(bootloader_path), shell=True
    )

    print("--> Flashing firmware")
    subprocess.check_call(
        "stm32loader -c rpi -f F4 -a 0x08010000 -w -v {}".format(firmware_path),
        shell=True,
    )

    print("Flashing completed!")

    if master_online and serial_node_active:
        if rosmon_available:
            write_flush("--> Starting the rosserial node.. ")
            start_stop("serial_node", "", StartStopRequest.START)
            print("DONE")
        else:
            print("You can now start the rosserial node.")
