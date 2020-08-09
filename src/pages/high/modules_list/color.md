# Color module type

The Color module handles an RGB color data.

Its type has access to all common capabilities.

----

## Functions

| **Function name and parameters** | **Action** | **Comment** |
|:---:|:---:|:---:|
| control(self) | Displays module type graphical interface | Only available using Jupyter notebook |

## Variables

| **Variable name** | **Action** | **Type** |
|:---:|:---:|:---:|
| color | RGB color | read / write: \[Char, Char, Char\] |
| time | Transition time between color command | read / write: Float |

## ROS topics
| **Topic name** | **Message type** |
|:----|:---:|
| /rgb_led_mod/variables/color/read | std_msgs/msg/ColorRGBA
| /rgb_led_mod/variables/color/write | std_msgs/msg/ColorRGBA
| /rgb_led_mod/variables/time/read | std_msgs/msg/Float32
| /rgb_led_mod/variables/time/write | std_msgs/msg/Float32

<div class="cust_edit_page"><a href="https://{{gh_path}}{{modules_path}}/color.md">Edit this page</a></div>
