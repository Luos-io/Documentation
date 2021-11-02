---
custom_edit_url: null
---

<h1><a href="#json-api" className="header" id="json-api"><img src="/img/json-logo.png" width="80px"/> / JSON API</a></h1>

The <a href="https://en.wikipedia.org/wiki/JSON" target="blank_">JSON formated data</a> is very common and widely used by many programming languages. Luos allows you to convert low-level Luos information into JSON objects, enabling conventional programming languages to easily interact with your device.<br/>
To do that, you must add a specific app service called a [Gate](./services_list/gate.md) on your device.

## How to start using the JSON API

Before using your device through JSON, you have to be connected to the communication flow depending on the node type hosting your Gate service.<br/>
Then you can start the Gate by sending:

```JSON
{"detection": {}}\r
```

This command asks the Gate to start a topological detection, create a routing table, convert it into JSON and send it back to you.

## Routing table messages

> **Warning:** Make sure to read and understand how [routing table](/embedded/services/routing-table.md) works before reading this part.

After the Gate starts, the first message you receive is a routing table.<br/>
This first message is really important, because it contains all the information allowing you to create a code object for your device, containing all its features.

### Routing table structure

A routing table in JSON consists in a list of the nodes present in the Luos network:

```JSON
{
   "routing_table":[
      {
         // node 1
      },
      {
         // node 2
      },
      {
         // node 3
      }
      // ...
   ]
}
```

#### Nodes information

Each listed node of the network has basic node information and a list of hosted services:

```JSON
{ // node 1
   "node_id":1,
   "certified":true,
   "port_table":[1, 2],
   "services":[
      {
         // service 1
      },
      {
         // service 2
      }
      // ...
   ]
}
```

> **Note:** To understand the meanings of _uuid_ and _port_table_, please refer to the [routing table page](/embedded/services/routing-table.md).

#### Services

Each listed service of a node has basic services information:

```JSON
{ // service 1
   "type":"Type",
   "id":1,
   "alias":"Alias"
}
```

> **Note:** To understand the meanings of _type_, _id_ and _alias_, please refer to the [service page](/embedded/services.html).

#### Full routing table example

```JSON
{
   "routing_table":[
      {
         "node_id":1,
         "certified":true,
         "port_table":[2, 65535],
         "services":[
            {
               "type":"Gate",
               "id":1,
               "alias":"r_right_arm"
            }
         ]
      },
      {
         "node_id":2,
         "certified":true,
         "port_table":[4, 1],
         "services":[
            {
               "type":"State",
               "id":2,
               "alias":"lock"
            },
            {
               "type":"Unknown",
               "id":3,
               "alias":"start_control"
            }
         ]
      },
      {
         "node_id":3,
         "certified":true,
         "port_table":[5, 3],
         "services":[
            {
               "type":"Imu",
               "id":4,
               "alias":"gps"
            }
         ]
      },
      {
         "node_id":4,
         "certified":true,
         "port_table":[65535, 4],
         "services":[
            {
               "type":"Color",
               "id":5,
               "alias":"alarm"
            },
            {
               "type":"Unknown",
               "id":6,
               "alias":"alarm_control"
            }
         ]
      }
   ]
}
```

Below is a visual representation of this routing table:

![](/img/luos-network-ex.png)

### Service's information messages

When the JSON routing table is transmitted, the Gate starts to update and stream your network data with **services information**.

This JSON is a "service" object listing all the services by their alias and the values they send:

```JSON
{
   "services":{
      "service_alias1":{
         "value1":12.5
      },
      "service_alias2":{
         "value1":13.6,
         "value2":[1, 2, 3, 4]
      }
   }
}
```

You can use the exact same JSON object structure to send data to services.

Here is the list of all values that can be used by services:

|      Value name       |                                                       Definition                                                       |
| :-------------------: | :--------------------------------------------------------------------------------------------------------------------: |
|      power_ratio      |                                   Percentage of power of an actuator (-100% to 100%)                                   |
|  target_rot_position  |                            Actuator's target angular position (can be a number or an array)                            |
|  limit_rot_position   |                                           Actuator's limit angular position                                            |
| limit_trans_position  |                                           Actuator's limit angular position                                            |
|      limit_power      |                                         Limit ratio of an actuator's reduction                                         |
|     limit_current     |                                                  Limit current value                                                   |
|   target_rot_speed    |                                            Actuator's target rotation speed                                            |
| target_trans_position |                            Actuator's target linear position (can be a number or an array)                             |
|  target_trans_speed   |                                             Actuator's target linear speed                                             |
|         time          |                                                       Time value                                                       |
|       compliant       |                                              Actuator's compliance status                                              |
|          pid          |                                Set of PID values (proportionnal, integral, derivative)                                 |
|      resolution       |                                               Sensor's resolution value                                                |
|        offset         |                                                      Offset value                                                      |
|       reduction       |                                            Ratio of an actuator's reduction                                            |
|       dimension       |                                                    Dimension value                                                     |
|         volt          |                                                     Voltage value                                                      |
|        current        |                                                 Electric current value                                                 |
|        reinit         |                                                Reinitialisation command                                                |
|        control        |                                        Control command (play, pause, stop, rec)                                        |
|         color         |                                                      Color value                                                       |
|       io_state        |                                                        IO state                                                        |
|         uuid          |                                                     Service's uuid                                                     |
|        rename         |                                                   Renaming an alias                                                    |
|       revision        |                                                   Firmware revision                                                    |
|    trans_position     |                                               Translation position value                                               |
|      trans_speed      |                                                Translation speed value                                                 |
|     rot_position      |                                                Rotation position value                                                 |
|       rot_speed       |                                                  Rotation speed value                                                  |
|          lux          |                                              Lux (light intensity) value                                               |
|      temperature      |                                                   Temperature value                                                    |
|         force         |                                                      Force value                                                       |
|        moment         |                                                      Torque value                                                      |
|         power         |                                                      Power value                                                       |
|     linear_accel      |                                               Linear acceleration value                                                |
|    gravity_vector     |                                                  Gravity vector value                                                  |
|        compass        |                                                     Compass value                                                      |
|         gyro          |                                                    Gyroscope value                                                     |
|         accel         |                                                   Acceleration value                                                   |
|         euler         |                                                   Euler angle value                                                    |
|      quaternion       |                                                   Quaternion values                                                    |
|   rotational_matrix   |                                                Rotational matrix values                                                |
|        heading        |                                                        Heading                                                         |
|       pedometer       |                                                   Steps number value                                                   |
|       walk_time       |                                                    Walk time value                                                     |
|     luos_revision     |                                                     Luos's version                                                     |
|    luos_statistics    | Luos's memory usage statistics \[Rx stack, Luos stack, Tx stack, Dropped messages, Loop delay, Send retry max number\] |

Here is an exemple of a message sent by a Potentiometer service about the rotation angle of the associated potentiometer:

```JSON
{
   "services":{
      "potentiometer_m":{
         "rot_position":12.5
      }
   }
}
```

Here is an exemple of a message sent by a gate service about Luos statistic:

```JSON
{
   "services":{
      "gate":{
         "luos_statistics":{
            "msg_stack":60,
            "luos_stack":53,
            "msg_drop":0,
            "loop_ms":16,
            "fail_ratio":0,
            "nak_max":1,
            "collision_max":5,
         }
      }
   }
}
```

#### Custom parameters and specific messages

Some messages are specifically handled:

<!-- - If the type is `VOID_TYPE`, the service is empty and no message is converted.-->

Custom parameters can be defined and sent to services through the JSON API, either with Python (Pyluos) or any other programming language on a computer side.
Here is an example of a C function that can be implemented in order to send commands to services in a Luos Network, through a gate:

```C
def sendCmd(s, cmd, sleep_time=0.5):
    cmd = cmd + '\r'
    print(cmd)
    s.write(cmd.encode())
    time.sleep(sleep_time)
s = serial.Serial(sys.argv[1], 1000000)
# detect Luos network
sendCmd(s, '{"detection": {}}')
# set speed mode and compliant mode
sendCmd(s, '{"services": {"controller_moto": {"parameters": 2441}}}')
# set pid parameters
sendCmd(s, '{"services": {"controller_moto": { "pid": [20, 0.02, 90]}}}')
# set speed mode and non compliant mode
sendCmd(s, '{"services": {"controller_moto": {"parameters": 2440}}}')
```

Parameters are defined by a 16-bit bitfield.

|   Object   |               Definition               |                                                                         Structure                                                                         |                                                            Service(s)                                                             |
| :--------: | :------------------------------------: | :-------------------------------------------------------------------------------------------------------------------------------------------------------: | :-------------------------------------------------------------------------------------------------------------------------------: |
| parameters | enabling or disabling some measurement | [Link to structure (GitHub)](https://github.com/Luos-io/Examples/blob/master/Projects/l0/Controller_motor/lib/Controller_motor/controller_motor.h#L7-L31) | [Stepper](./services_list/stepper.md), [Controller-motor](./services_list/controller-motor.md), [Servo](./services_list/servo.md) |
| parameters | enabling or disabling some measurement |             [Link to structure (GitHub)](https://github.com/Luos-io/Examples/blob/master/Projects/l0/Imu/lib/Imu/mpu_configuration.h#L37-L56)             |                                                   [Imu](./services_list/imu.md)                                                   |

Other specific messages:

|   Object   |                            Definition                            |                Service(s)                 |
| :--------: | :--------------------------------------------------------------: | :---------------------------------------: |
|  register  |   Motor memory register filed with \[register_number, value\]    | [Dynamixel](./services_list/dxl.md), void |
|   set_id   |                         A set id command                         | [Dynamixel](./services_list/dxl.md), void |
| wheel_mode | The wheel mode parameter for Dynamixel servomotors True or False | [Dynamixel](./services_list/dxl.md), void |
|   delay    |                   reduce services refresh rate                   |      [Gate](./services_list/gate.md)      |

### Services exclusion messages

Services can be excluded from the network if a problem occurs (see [self-healing](/embedded/services/self-healing.md#service-exclusion) for more information). In this case, the Gate sends an exclusion message indicating that this service is no longer available:

```JSON
{"dead_service": "service_alias"}
```

### Node assert messages

Nodes can assert if a critical issue occurs (see [self-healing](/embedded/services/self-healing.html#assert) for more information). In this case, the Gate sends an assertion message indicating that this node is no longer available and some details about the crash:

```JSON
{
   "assert":{
      "node_id":3,
      "file":"/Users/foo/luos/Examples/Drivers/Button/button.c",
      "line":75
   }
}
```

## Sending large binary data

Binary data such as, for example, a motor tarjectory can't be included into a JSON file if it is too large. In order to allow this type of transmission, the size of the binary data is sent through the JSON, then followed by the actual data in binary format.

- If the data is short, it can be displayed inside the JSON as a regular value (see the different values in [Service's information messages section](#services-information-messages)), or as a table of several values (for example a motor trajectory).

- If the data is large, the defined value must be a **table of one element**, containing only the size of the binary data to be transfered, in bytes.

The following example shows a transfert of a binary data of 1024 bytes.

```JSON
{
   "services":{
      "service_alias1":{
         "rot_position":[1024]
      }
   }
}
###BINARY_DATA###
```
