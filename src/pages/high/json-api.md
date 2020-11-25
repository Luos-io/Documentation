<img class="print-break" src="{{img_path}}/json-logo.png" height="100px">

<h1 class="no-break"><a href="#json-api" class="header" id="json-api">JSON API</a></h1>

The <a href="https://en.wikipedia.org/wiki/JSON" target="blank_">JSON formated data</a> is very common and widely used by many programming languages. Luos allows you to convert low-level Luos information into JSON objects, enabling conventional programming languages to easily interact with your device.<br/>
To do that, you must add a specific app container called [Gate]({{containers_path}}/gate.md) on your device.

The [Gate container]({{containers_path}}/gate.md) is an app that converts Luos messages from a device's network into JSON data format, and the other way from JSON to Luos messages.<br/>
The Gate container can be hosted into different kinds of <span class="cust_tooltip">nodes<span class="cust_tooltiptext">{{node_def}}</span></span> allowing you to choose the communication way fitting with your project (USB, Wifi, Bluetooth, etc.)

> **Warning:** The Gate container refreshes sensors information as fast as it can, so that can be intensive to Luos bandwidth.

## How to start using the JSON API
Before using your device through JSON, you have to be connected to the communication flow depending on the node type hosting your Gate container.<br/>
Then you can start the Gate by sending:
```JSON
{"detection": {}}\r
```

This command asks the Gate to start a topological detection, create a routing table, convert it into JSON and send it back to you.

## Routing table messages
> **Warning:** Make sure to read and understand how [routing table](/pages/low/containers/routing-table.md) works before reading this part.

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
Each listed node of the network has basic node information and a list of hosted containers:

```JSON
{ // node 1
   "node_id":1,
   "certified":true,
   "port_table":[1, 2],
   "containers":[
      {
         // container 1
      },
      {
         // container 2
      }
      // ...
   ]
}
```
> **Note:** To understand the meanings of *uuid* and *port_table*, please refer to the [routing table page](/pages/low/containers/routing-table.md).

#### Containers
Each listed container of a node has basics containers information:

```JSON
{ // container 1
   "type":"Type",
   "id":1,
   "alias":"Alias"
}
```
> **Note:** To understand the meanings of *type*, *id* and *alias*, please refer to the [container page](/pages/low/containers.html).

#### Full routing table example

```JSON
{
   "routing_table":[
      {
         "node_id":1,
         "certified":true,
         "port_table":[2, 65535],
         "containers":[
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
         "containers":[
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
         "containers":[
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
         "containers":[
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

![](/_assets/img/luos-network-ex.png)


### Container's information messages
When the JSON routing table is transmitted, the Gate starts to update and stream your network data with **containers information**.

This JSON is a "container" object listing all the containers by their alias and the values they send:
```JSON
{
   "containers":{
      "container_alias1":{
         "value1":12.5
      },
      "container_alias2":{
         "value1":13.6,
         "value2":[1, 2, 3, 4]
      }
   }
}
```

You can use the exact same JSON object structure to send data to containers.

Here is the list of all values that can be used by containers:

|Value name|Definition|
|:---:|:---:|
|power_ratio|Percentage of power of an actuator (-100% to 100%)|
|target_rot_position|Actuator's target angular position (can be a number or an array)|
|limit_rot_position|Actuator's limit angular position|
|limit_trans_position|Actuator's limit angular position|
|limit_power|Limit ratio of an actuator's reduction|
|limit_current|Limit current value|
|target_rot_speed|Actuator's target rotation speed|
|target_trans_position|Actuator's target linear position (can be a number or an array)|
|target_trans_speed|Actuator's target linear speed|
|time|Time value|
|compliant|Actuator's compliance status|
|pid|Set of PID values (proportionnal, integral, derivative)|
|resolution|Sensor's resolution value|
|offset|Offset value|
|reduction|Ratio of an actuator's reduction|
|dimension|Dimension value|
|volt|Voltage value|
|current|Electric current value|
|reinit|Reinitialisation command|
|control|Control command (play, pause, stop, rec)|
|color|Color value|
|io_state|IO state|
|uuid|Container's uuid|
|rename|Renaming an alias|
|revision|Firmware revision|
|trans_position|Translation position value|
|trans_speed|Translation speed value|
|rot_position|Rotation position value|
|rot_speed|Rotation speed value|
|lux|Lux (light intensity) value|
|temperature|Temperature value|
|force|Force value|
|moment|Torque value|
|power|Power value|
|linear_accel|Linear acceleration value|
|gravity_vector|Gravity vector value|
|compass|Compass value|
|gyro|Gyroscope value|
|accel|Acceleration value|
|euler|Euler angle value|
|quaternion|Quaternion values|
|rotational_matrix|Rotational matrix values|
|heading|Heading|
|pedometer|Steps number value|
|walk_time|Walk time value|
|luos_revision|Luos's version|
|luos_statistics|Luos's memory usage statistics [Allocator stack, Message stack, Luos stack, Dropped messages, Loop delay]|



Here is an exemple of a message sent by a Potentiometer container about the rotation angle of the associated potentiometer:
```JSON
{
   "containers":{
      "potentiometer_m":{
         "rot_position":12.5
      }
   }
}
```

#### Custom parameters and specific messages
Some messages are specifically handled:

<!-- - If the type is `VOID_MOD`, the container is empty and no message is converted.-->

Custom parameters can be defined and sent to containers through the JSON API, either with Python (Pyluos) or any other programming language on a computer side.
Here is an example of a C function that can be implemented in order to send commands to containers in a Luos Network, through a gate:
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
sendCmd(s, '{"containers": {"controlled_moto": {"parameters": 2441}}}')
# set pid parameters
sendCmd(s, '{"containers": {"controlled_moto": { "pid": [20, 0.02, 90]}}}')
# set speed mode and non compliant mode
sendCmd(s, '{"containers": {"controlled_moto": {"parameters": 2440}}}')
```
Parameters are defined by a 16-bit bitfield.

|Object|Definition|Structure|Container(s)|
|:---:|:---:|:---:|:---:|
|parameters|enabling or disabling some measurement|[Link to structure (GitHub)](https://github.com/Luos-io/Examples/blob/master/Drivers/Controlled_motor/controlled_motor.h#L7-L31)|[Stepper]({{containers_path}}/stepper.md), [Controlled-motor]({{containers_path}}/controlled-motor.md), [Servo]({{containers_path}}/servo.md)|
|parameters|enabling or disabling some measurement|[Link to structure (GitHub)](https://github.com/Luos-io/Examples/blob/master/Drivers/Imu/mpu_configuration.h#L37-L56)|[Imu]({{containers_path}}/imu.md)|

Other specific messages:

|Object|Definition|Container(s)|
|:---:|:---:|:---:|
|register|Motor memory register filed with \[register_number, value\]|[Dynamixel]({{containers_path}}/dxl.md), void|
|set_id|A set id command|[Dynamixel]({{containers_path}}/dxl.md), void|
|wheel_mode|The wheel mode parameter for Dynamixel servomotors True or False|[Dynamixel]({{containers_path}}/dxl.md), void|
|delay|reduce containers refresh rate|[Gate]({{containers_path}}/gate.md)|

### Container exclusion messages
Container can be excluded of the network if a problem occurs (See [message handling](/pages/low/containers/msg-handling.html#container-exclusion) for more information). In this case, the Gate sends an exclusion message indicating that this container is no longer available:
```JSON
{"dead_container": "container_alias"}
```

## Sending large binary data
Binary data such as, for example, a motor tarjectory can't be included into a Json file if it is too large. In order to allow this type of transmission, the size of the binary data is sent through the Json, then followed by the actual data in binary format.

 - If the data is short, it can be displayed inside the JSON as a regular value (see the different values in [Container's information messages section](#containers-information-messages)), or as a table of several values (for example a motor trajectory).

 - If the data is large, the defined value must be a **table of one element**, containing only the size of the binary data to be transfered, in bytes.

The following example shows a transfert of a binary data of 1024 bytes.
```JSON
{
   "containers":{
      "container_alias1":{
         "rot_position":[1024]
      }
   }
}
###BINARY_DATA###
```


