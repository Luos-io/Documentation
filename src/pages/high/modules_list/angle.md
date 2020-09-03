# Angle module type

The Angle module handles a rotation position value in degree.

Its type has access to all common capabilities.

----

## Variables

| **Variable name** | **Action** | **Type** |
|:---:|:---:|:---:|
| rot_position | Reads the rotation position in degree | Read only: Float |
| threshold | Thresholds position variation before *filter_changed* event triggers. Default value 10 °. | Read / write: Float |

## Events

| **Event name** | **Trigger** |
|:---:|:---:|
| changed | Any movement on the position measurement |
| filter_changed | Movement bigger than *threshold* |

## ROS topics
| **Topic name** | **Message type** |
|:----|:---:|
| /mod/variables/rot_position/read | std_msgs/msg/Float32
| /mod/variables/threshold/read | std_msgs/msg/Float32
| /mod/variables/threshold/write | std_msgs/msg/Float32
| /mod/events/changed | luos_msgs/msg/FloatChange
| /mod/events/filter_changed | luos_msgs/msg/FloatChange

<div class="cust_edit_page"><a href="https://{{gh_path}}{{modules_path}}/angle.md">Edit this page</a></div>
