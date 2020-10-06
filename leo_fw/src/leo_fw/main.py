from __future__ import print_function

import sys

import rospy
import rosgraph
import rosnode
import rosservice

from std_srvs.srv import Trigger
from rosmon_msgs.srv import StartStop, StartStopRequest

from .utils import *


def update_fw():
    prompt("--> Checking if stm32loader is installed.. ")

    if is_tool("stm32loader"):
        print("YES")
    else:
        print("NO")
        print("ERROR: Cannot find the stm32loader tool. "
              "Make sure the python3-stm32loader package is installed.")
        return

    prompt("--> Checking if ROS Master is online.. ")

    if rosgraph.is_master_online():
        print("YES")
        master_online = True
    else:
        print("NO")
        master_online = False
        print("ROS Master is not running. "
              "Will not be able to check the current firmware version.")
        if not query_yes_no("Are you sure you want to continue?", default="no"):
            return

    if master_online:
        print("--> Initializing ROS node")
        rospy.init_node("firmware_updater", anonymous=True)

    if master_online:
        prompt("--> Checking if rosserial node is active.. ")

        if "/serial_node" in rosnode.get_node_names():
            print("YES")
            serial_node_active = True
        else:
            print("NO")
            serial_node_active = False
            print("Rosserial node is not active. "
                  "Will not be able to check the current firmware version.")
            if not query_yes_no("Are you sure you want to continue?", default="no"):
                return

    firmware_version = "<unknown>"

    if master_online and serial_node_active:
        print("--> Checking the current firmware version")

        if "/core2/get_firmware_version" in rosservice.get_service_list():
            get_firmware_version = rospy.ServiceProxy(
                "/core2/get_firmware_version", Trigger)
            firmware_version = get_firmware_version().message
        else:
            print("WARNING: Could not get the current firmware version: "
                  "/core2/get_firmware_version service is not available.")

    