# DC-motor API

The DC-motor service allows to drive a DC motor using only power mode.

Its type has access to all common capabilities.

----

## Functions

| **Function name and parameters** | **Action** | **Comment** |
|:---:|:---:|:---:|
| control(self) | Displays service type graphical interface | Only available using Jupyter notebook |

## Variables

| **Variable name** | **Action** | **Type** |
|:---:|:---:|:---:|
| power_ratio | Sets the power quantity send to the motor between -100% and 100%. | read / write: float |

## ROS topics
| **Topic name** | **Message type** |
|:----|:---:|
| /DC_motor1_mod/variables/power_ratio/read | std_msgs/msg/Float32
| /DC_motor1_mod/variables/power_ratio/write | std_msgs/msg/Float32
