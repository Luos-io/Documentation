# State API

The State service can handles a sensor (Button board for example), or an actuator (Power Switch board for example). Generally, this type of services allows to manage bi-state elements such as on/off, pushed/release, 0/1, ...

Its type has access to all common capabilities.

----

## Functions

| **Function name and parameters** | **Action** | **Comment** |
|:---:|:---:|:---:|
| control(self) | Displays service type graphical interface | Only available using Jupyter notebook |

## Variables

| **Variable name** | **Action** | **Type** |
|:---:|:---:|:---:|
| state | Sets or reads the service state | read / write: Boolean (True or False) |

## Events

| **Event name** | **Trigger** |
|:---:|:---:|
| changed | Any state modification pressed or released |
| pressed | State modification from True to False |
| released | State modification from False to True |

## ROS topics
| **Topic name** | **Message type** |
|:----|:---:|
| /button_mod/variables/state/read | std_msgs/msg/Bool
| /button_mod/variables/state/write | std_msgs/msg/Bool
| /button_mod/events/released | luos_msgs/msg/BoolChange
| /button_mod/events/pressed | luos_msgs/msg/BoolChange
| /button_mod/events/changed | luos_msgs/msg/BoolChange
