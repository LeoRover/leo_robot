from enum import Enum
from typing import Optional

import rospy
import rosservice

from std_srvs.srv import Trigger


class BoardType(Enum):
    LEOCORE = "leocore"
    CORE2 = "core2"

    def __str__(self):
        return self.value


def determine_board() -> Optional[BoardType]:
    services = rosservice.get_service_list()

    if rospy.resolve_name("firmware/get_board_type") in services:
        get_board_type = rospy.ServiceProxy("firmware/get_board_type", Trigger)
        type_str = get_board_type().message
        if type_str == str(BoardType.CORE2):
            return BoardType.CORE2
        elif type_str == str(BoardType.LEOCORE):
            return BoardType.LEOCORE

    # Support legacy firmware
    if rospy.resolve_name("core2/get_firmware_version") in services:
        return BoardType.CORE2

    return None


def check_firmware_version() -> str:
    services = rosservice.get_service_list()

    if rospy.resolve_name("firmware/get_firmware_version") in services:
        get_firmware_version = rospy.ServiceProxy(
            "firmware/get_firmware_version", Trigger
        )
    # Support legacy firmware
    elif rospy.resolve_name("core2/get_firmware_version") in services:
        get_firmware_version = rospy.ServiceProxy(
            "core2/get_firmware_version", Trigger
        )
    else:
        return "<unknown>"

    return get_firmware_version().message
