## Overview

This package contains resources for bringing up basic Leo Rover
functionalities.

The most important file is the **leo_bringup.launch** which is run by
[LeoOS](LeoOS) at the startup process and includes core software
components used by Leo, such as: host-side
[rosserial](https://index.ros.org/p/rosserial) and
[rosbridge](https://index.ros.org/p/rosbridge_suite) nodes,
[robot_state_publisher](https://index.ros.org/p/robot_state_publisher),
camera driver and some more.

## Usage

This package is intended to be used by a system service, but it can also
be used standalone: 

```
roslaunch leo_bringup leo_bringup.launch
```



## Nodes

### leo_system

A simple node which exposes system shutdown and reboot functions on ROS
topics

#### Subscribed Topics

- `system/shutdown`
  ([std_msgs/msg/Empty](https://docs.ros.org/en/rolling/p/std_msgs/msg/Empty.html)):
  Performs system shutdown.
- `system/reboot`
  ([std_msgs/msg/Empty](https://docs.ros.org/en/rolling/p/std_msgs/msg/Empty.html)):
  Performs system reboot.

#### Published Topics

- `robot_namespace`
  ([std_msgs/msg/String](https://docs.ros.org/en/rolling/p/std_msgs/msg/String.html)):
  The namespace of the robot.
