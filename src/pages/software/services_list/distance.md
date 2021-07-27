# Distance API

The Distance service handles a sensor measuring a distance in mm.

Its type has access to all common capabilities.

----

## Variables

| **Variable name** | **Action** | **Type** |
|:---:|:---:|:---:|
| distance | Reads the measured distance in mm | read only: Float |
| threshold | Thresholds distance variation before filter_changed event trigers. Default value 10 mm. | read / write: Float |

## Events

| **Event name** | **Trigger** |
|:---:|:---:|
| changed | Any movement on the distance measurement |
| filter_changed | Movement bigger than *threshold* |

## ROS topics
| **Topic name** | **Message type** |
|:----|:---:|
| /mod/variables/distance/read | std_msgs/msg/Float32
| /mod/variables/threshold/read | std_msgs/msg/Float32
| /mod/variables/threshold/write | std_msgs/msg/Float32
| /mod/events/changed | luos_msgs/msg/FloatChange
| /mod/events/filter_changed | luos_msgs/msg/FloatChange



