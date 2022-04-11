# Dynamixel API

The Dynamixel service allows to control Dynamixel motors.

Its type has access to all common capabilities.

---

## Functions

| **Function name and parameters** |                **Action**                 |                                                     **Comment**                                                      |
| :------------------------------: | :---------------------------------------: | :------------------------------------------------------------------------------------------------------------------: |
|         set_id(self, id)         |             Changes motor ID              | This new Id will be saved by the Dynamixel motor. You have to detect motors again to make it work after this change. |
|           detect(self)           |        Launches a motor detection         |                          You have to run a luos detection to include or exclude new motors.                          |
|  register(self, register, val)   |     Sets a Dynamixel register value.      |                 This register only manage _word_ size register. Use it only if you know what you do.                 |
|          control(self)           | Displays service type graphical interface |                                        Only available using Jupyter notebook                                         |

## Variables

|  **Variable name**  |                                                     **Action**                                                     |                  **Type**                   |
| :-----------------: | :----------------------------------------------------------------------------------------------------------------: | :-----------------------------------------: |
|      compliant      | - True: disables the motor power, you can use it to move the motor by hand.<br/> - False: Enables the motor power. |    read / write: Boolean (True or False)    |
| target_rot_position |                                  Sets the target rotation position to reach in °.                                  |             read / write: Float             |
|  target_rot_speed   |                                  Sets the target rotation speed to reach in °/s.                                   |             read / write: Float             |
|     wheel_mode      |                                      Enables or disables wheel mode on motor                                       |    read / write: Boolean (True or False)    |
|    rot_position     |                                        Measured position of the motor in °.                                        |             read / write: Float             |
|     temperature     |                                      Measured temperature of the motor in °C.                                      |             read / write: Float             |
|     positionPid     |                  Sets position PID used for rotation position mode and translation position mode                   | read / write: \[float P, float I, float D\] |
|  power_ratio_limit  |                                               Max power limit in %.                                                |             read / write: Float             |
| rot_position_limit  |                                     Min and Max rotation position limit in °.                                      |  read / write: \[Float(min), Float(max)\]   |

## ROS topics

| **Topic name**                             |     **Message type**      |         **Comment**          |
| :----------------------------------------- | :-----------------------: | :--------------------------: |
| /dxl_1/variables/compliant/read            |   std_msgs/msg/Boolean    |
| /dxl_1/variables/compliant/write           |   std_msgs/msg/Boolean    |
| /dxl_1/variables/target_rot_position/read  |   std_msgs/msg/Float32    |
| /dxl_1/variables/target_rot_position/write |   std_msgs/msg/Float32    |
| /dxl_1/variables/target_rot_speed/read     |   std_msgs/msg/Float32    |
| /dxl_1/variables/target_rot_speed/write    |   std_msgs/msg/Float32    |
| /dxl_1/variables/wheel_mode/read           |   std_msgs/msg/Boolean    |
| /dxl_1/variables/wheel_mode/write          |   std_msgs/msg/Boolean    |
| /dxl_1/variables/rot_position/read         |   std_msgs/msg/Float32    |
| /dxl_1/variables/rot_position/write        |   std_msgs/msg/Float32    |
| /dxl_1/variables/temperature/read          |   std_msgs/msg/Float32    |
| /dxl_1/variables/temperature/write         |   std_msgs/msg/Float32    |
| /dxl_1/variables/positionPid/read          | geometry_msgs/msg/Vector3 |       (x=P, y=I, z=D)        |
| /dxl_1/variables/positionPid/write         | geometry_msgs/msg/Vector3 |       (x=P, y=I, z=D)        |
| /dxl_1/variables/power_ratio_limit/read    |   std_msgs/msg/Float32    |
| /dxl_1/variables/power_ratio_limit/write   |   std_msgs/msg/Float32    |
| /dxl_1/variables/rot_position_limit/read   | geometry_msgs/msg/Vector3 | (x=min, y=max, z=<unused />) |
| /dxl_1/variables/rot_position_limit/write  | geometry_msgs/msg/Vector3 | (x=min, y=max, z=<unused />) |

## Example code

### Example commands from ROS topics

Plot the real time angular position with RQT plot:

```bash
ros2 run rqt_plot rqt_plot /dxl_1/variables/rot_position/read
```

Disable compliance:

```bash
ros2 topic pub -1 /dxl_1/variables/compliant/write std_msgs/msg/Bool data:\ false\
```

The motor will move to its saved target.

Switch to wheel mode and set a rotational speed target:

```bash
ros2 topic pub -1 /dxl_1/variables/wheel_mode/write std_msgs/msg/Bool data:\ true\
ros2 topic pub -1 /dxl_1/variables/target_rot_speed/write std_msgs/msg/Float32 data:\ 3.14\
```

The motor will move at 3.14 rad/s

Switch to position mode and set a rotational position target:

```bash
ros2 topic pub -1 /dxl_1/variables/wheel_mode/write std_msgs/msg/Bool data:\ false\
ros2 topic pub -1 /dxl_1/variables/target_rot_position/write std_msgs/msg/Float32 data:\ 0.0\
```

The motor will go to angle zero.

**Note**: If no motor is connected to the Dynamixel board, the ROS broker will display a warning message and ignore the board.
