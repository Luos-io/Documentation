# JSON API
The <a href="https://en.wikipedia.org/wiki/JSON" target="blank_">JSON formated data</a> is very common and widely used by many programming languages. Luos allows you to convert low-level Luos information into JSON objects, enabling conventional programming languages to interact with you device easily.<br/>
To do that, you must add a specific app module called [Gate]({{modules_path}}/gate.md) on your device.

The [Gate module]({{modules_path}}/gate.md) is an app that converts Luos messages from a device's network into JSON data format, and the other way from JSON to Luos messages.<br/>
The Gate module can be hosted into different kinds of <span class="cust_tooltip">nodes<span class="cust_tooltiptext">{{node_def}}</span></span> allowing you to choose the communication way fitting with your project (USB, Wifi, Bluetooth, etc.)

> **Warning:**: The Gate module refreshes sensors information as fast as it can, so that can be intensive to Luos bandwidth.

## How to start using the JSON API
Before using your device through JSON, you have to be connected to the communication flow depending on the node type hosting your Gate module.<br/>
Then you can start the Gate by sending:
```JSON
{"detection": {}}\r
```

This command asks the Gate to start a topological detection, create a routing table, convert it into JSON and send it back to you.

## Routing table messages
> **Warning:** Make sure to read and understand how [routing table](/pages/low/modules/routing-table.md) works before reading this part.

After the Gate starts, the first message you receive is a routing table.<br/>
This first message is really important, because it contains all the information allowing you to create a code object for your device, containing all its features.

### Routing table structure
A routing table in JSON consists in a list of the nodes present in the Luos network:

```JSON
{
   "route_table":[
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
Each listed node of the network has basic node information and a list of hosted modules:

```JSON
{ // node 1
   "uuid":[1, 2, 3],
   "port_table":[1, 2],
   "modules":[
      {
         // module 1
      },
      {
         // module 2
      }
      // ...
   ]
}
```
> **Note:** To understand the meanings of *uuid* and *port_table*, please refer to the [routing table page](/pages/low/modules/routing-table.md).

#### Modules
Each listed module of a node has basics modules information:

```JSON
{ // module 1
   "type":"Type",
   "id":1,
   "alias":"Alias"
}
```
> **Note:** To understand the meanings of *type*, *id* and *alias*, please refer to the [module page](/pages/low/modules.html).

#### Full routing table example

```JSON
{
   "route_table":[
      {
         "uuid":[2031684, 1112756496, 540423216],
         "port_table":[2, 65535],
         "modules":[
            {
               "type":"Gate",
               "id":1,
               "alias":"r_right_arm"
            }
         ]
      },
      {
         "uuid":[4915239, 1194612503, 540554032],
         "port_table":[4, 1],
         "modules":[
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
         "uuid":[2818086, 1194612503, 540554032],
         "port_table":[5, 3],
         "modules":[
            {
               "type":"Imu",
               "id":4,
               "alias":"gps"
            }
         ]
      },
      {
         "uuid":[2097186, 1194612503, 540554032],
         "port_table":[65535, 4],
         "modules":[
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


### Module's information messages
When the JSON routing table is transmitted, the Gate starts to update and stream your network data with **modules information**.

This JSON is a "module" object listing all the modules by their alias and the values they send:
```JSON
{
   "modules":{
      "module_alias1":{
         "value1":12.5
      },
      "module_alias2":{
         "value1":13.6,
         "value2":[1, 2, 3, 4]
      }
   }
}
```

You can use the exact same JSON object structure to send data to modules.

Here is the list of all values that can be used by modules:

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
|led|Board's LED|
|node_temperature|Node's temperature|
|node_voltage|Node's voltage|
|uuid|Module's uuid|
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
|luos_revision|luos's version|
|robus_revision|robus's version|


Here is an exemple of a message sent by a Potentiometer module about the rotation angle of the associated potentiometer:
```JSON
{
   "modules":{
      "potentiometer_m":{
         "rot_position":12.5
      }
   }
}
```

##### Specific messages
Some messages are specifically handled:

<!-- - If the type is `VOID_MOD`, the module is empty and no message is converted.-->

 - For[Dynamixel]({{modules_path}}/dxl.md) and void modules:

|Object|Definition|
|:---:|:---:|
|register|Motor memory register filed with \[register_number, value\]|
|set_id|A set id command|
|wheel_mode|The wheel mode parameter for Dynamixel servomotors True or False|

 - For [Stepper]({{modules_path}}/stepper.md), [Controlled-motor]({{modules_path}}/controlled-motor.md) and [Servo]({{modules_path}}/servo.md) modules:

|Object|Definition|
|:---:|:---:|
|parameters|enabling or disabling some measurement|

 - For [Imu]({{modules_path}}/imu.md) module:

|Object|Definition|
|:---:|:---:|
|parameters|enabling or disabling some measurement|

 - For [Gate]({{modules_path}}/gate.md) module:

|Object|Definition|
|:---:|:---:|
|delay|reduce modules refresh rate|

### Module exclusion messages
Module can be excluded of the network if a problem occurs (See [message handling](/pages/low/modules/msg-handling.html#module-exclusion) for more information). In this case, the Gate sends an exclusion message indicating that this module is no longer available:
```JSON
{"dead_module": "module_alias"}
```

<div class="cust_edit_page"><a href="https://{{gh_path}}/pages/high/json-api.md">Edit this page</a></div>
