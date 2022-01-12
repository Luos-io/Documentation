# Stepper API

This service type allows to control a stepper motor. It computes micro-stepping and motion planning.

Its type has access to all common capabilities.

### services’s type settings:

> **Warning:** This service doesn't save any of the following parameters, they must be set each time your service reboots.

The number of steps per turn must be defined, as well as the wheel diameter at the output of the motor if you wahnt to use translation. These specs may figure in your motor’s datasheet.

----

## Functions

| **Function name and parameters** | **Action** | **Comment** |
|:---:|:---:|:---:|
| setToZero(self) | Resets current position of the motor to 0 | You can use it to initialize the position of the motor |
| control(self) | Displays service type graphical interface | Only available using Jupyter notebook |

## Variables

### Motor settings

| **Variable name** | **Action** | **Type** |
|:---:|:---:|:---:|
| stepPerTurn | Defines the stepper resolution | read / write: float |
| wheel_size | Defines wheel size used for translation mode | read / write: float |

### Motor control modes

| **Variable name** | **Action** | **Type** |
|:---:|:---:|:---:|
| compliant | - True: disables the motor driver, you can use it to move the motor by hand.<br/> - False: Enables the motor driver. | read / write: Boolean (True or False) |
| rot_position_mode | Enables/Disables the motor rotation position control mode.<br/>*Disables power mode and translation position mode if enabled.*<br/>*Doesn't work if no position PID is configured.* | read / write: Boolean (True or False) |
| rot_speed_mode | Enables/Disables the motor rotation speed control mode.<br/>*Disables power mode and translation speed mode if enabled.*<br/>*Doesn't work if no speed PID configured.* | read / write: Boolean (True or False) |
| trans_position_mode | Enables/Disables the motor translation position control mode.<br/>*Disables power mode and rotation position mode if enabled.*<br/>*Doesn't work if no position PID configured.* | read / write: Boolean (True or False) |
| trans_speed_mode | Enables/Disables the motor translation speed control mode.<br/>*Disables power mode and rotation speed mode if enabled.*<br/>*Doesn't work if no speed PID configured.* | read / write: Boolean (True or False) |

### Motor commands

| **Variable name** | **Action** | **Type** |
|:---:|:---:|:---:|
| target_rot_position | Sets the target rotation position to reach in °. | read / write: float |
| target_rot_speed | Sets the target rotation speed to reach in °/s. | read / write: float |
| target_trans_position | Sets the target translation position to reach in mm. | read / write: float |
| target_trans_speed | Sets the target translation speed to reach in mm/s. | read / write: float |

## ROS topics

| **Topic name** | **Message type** | **Comment** |
|:----|:---:|:---:|
| /stepper_1/variables/stepPerTurn/read | std_msgs/msg/Float32 |
| /stepper_1/variables/stepPerTurn/write | std_msgs/msg/Float32 |
| /stepper_1/variables/wheel_size/read | std_msgs/msg/Float32 |
| /stepper_1/variables/wheel_size/write | std_msgs/msg/Float32 |
| /stepper_1/variables/compliant/read | std_msgs/msg/Bool |
| /stepper_1/variables/compliant/write | std_msgs/msg/Bool |
| /stepper_1/variables/rot_position_mode/read | std_msgs/msg/Bool |
| /stepper_1/variables/rot_position_mode/write | std_msgs/msg/Bool |
| /stepper_1/variables/rot_speed_mode/read | std_msgs/msg/Bool |
| /stepper_1/variables/rot_speed_mode/write | std_msgs/msg/Bool |
| /stepper_1/variables/trans_position_mode/read | std_msgs/msg/Bool |
| /stepper_1/variables/trans_position_mode/write | std_msgs/msg/Bool |
| /stepper_1/variables/trans_speed_mode/read | std_msgs/msg/Bool |
| /stepper_1/variables/trans_speed_mode/write | std_msgs/msg/Bool |
| /stepper_1/variables/target_rot_position/read | std_msgs/msg/Float32 | value in radians
| /stepper_1/variables/target_rot_position/write | std_msgs/msg/Float32 | value in radians
| /stepper_1/variables/target_rot_speed/read | std_msgs/msg/Float32 | value in radians
| /stepper_1/variables/target_rot_speed/write | std_msgs/msg/Float32 | value in radians
| /stepper_1/variables/target_trans_position/read | std_msgs/msg/Float32 |
| /stepper_1/variables/target_trans_position/write | std_msgs/msg/Float32 |
| /stepper_1/variables/target_trans_speed/read | std_msgs/msg/Float32 |
| /stepper_1/variables/target_trans_speed/write | std_msgs/msg/Float32 |
