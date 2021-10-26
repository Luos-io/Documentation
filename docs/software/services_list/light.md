# Light API

The Light service handles a sensor measuring a light intensity in lux.

Its type has access to all common capabilities.

----

## Variables

| **Variable name** | **Action** | **Type** |
|:---:|:---:|:---:|
| lux | Reads the measured light intensity in lux | read only: Float |
| threshold | Thresholds light intensity variation before *filter_changed* event trigers. Default value 10 lux. | read / write: Float |

## Events

| **Event name** | **Trigger** |
|:---:|:---:|
| changed | Any movement on the light intensity measurement |
| filter_changed | Movement bigger than *threshold* |

## ROS topics
| **Topic name** | **Message type** |
|:----|:---:|
| /mod/variables/lux/read | std_msgs/msg/Float32
| /mod/variables/threshold/read | std_msgs/msg/Float32
| /mod/variables/threshold/write | std_msgs/msg/Float32
| /mod/events/changed | luos_msgs/msg/FloatChange
| /mod/events/filter_changed | luos_msgs/msg/FloatChange



