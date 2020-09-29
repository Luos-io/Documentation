# Servo container type

The Servo container allows to drive RC elements like servomotor

Its type has access to all common capabilities.

----

## Functions

| **Function name and parameters** | **Action** | **Comment** |
|:---:|:---:|:---:|
| control(self) | Displays container type graphical interface | Only available using Jupyter notebook |

## Variables

| **Variable name** | **Action** | **Type** |
|:---:|:---:|:---:|
| rot_position | Rotation position in °. | read / write: float |
| max_angle | Sets `max_angle` value, in degrees (`°`) (`180.0` by default) | read / write: Float |
| min_pulse | Sets PWM minimum pulse value, in seconds (`s`) (`0.0005` by default)| read / write: Float |
| max_pulse | Sets PWM maximum pulse value, in seconds (`s`) (`0.0015` by default)| read / write: Float |

## ROS topics
| **Topic name** | **Message type** |
|:----|:---:|
| /mod/variables/rot_position/read | std_msgs/msg/Float32
| /mod/variables/rot_position/write | std_msgs/msg/Float32
| /mod/variables/max_angle/read | std_msgs/msg/Float32
| /mod/variables/max_angle/write | std_msgs/msg/Float32
| /mod/variables/min_pulse/read | std_msgs/msg/Float32
| /mod/variables/min_pulse/write | std_msgs/msg/Float32
| /mod/variables/max_pulse/read | std_msgs/msg/Float32
| /mod/variables/max_pulse/write | std_msgs/msg/Float32


<div class="cust_edit_page"><a href="https://{{gh_path}}{{containers_path}}/servo.md">Edit this page</a></div>
