# JSON API

The JSON data format is very common and widely used by many programming languages. Luos allow you to convert low level Luos informations into Json objects, enabling conventional programming languages to interact with you device easily.<br/>
To do that you have to add a sepcific App module called [Gate]({{modules_path}}/gate.md)  on your device.

The [Gate module]({{modules_path}}/gate.md) is an app that converts Luos messages from a device's network into <a href="https://en.wikipedia.org/wiki/JSON" target="blank_">JSON formated data</a>, and the other way from JSON to Luos messages.<br/>
The gate module can be hosted into different kind of nodes allowing you to choose the communication way fitting with your project (USB, Wifi, Bluetooth,...)

> **Warning:**: Gate App module refresh sensors informations as fast as it can and could be Luos bandwidth intensive.

## How to start using the Json API

Before using your device through Json you have to be connected to the communication flux depending on the node type hosting your Gate app module.<br/>
Then you can start the Gate by sending :
```JSON
{"detection": {}}\r
```

This command ask the Gate to start a topological detection, create a route table, convert it into Json and send it back to you.

## Routing table messages
> **Warning:** Make sure to read and understand how [routing table](/_pages/low/modules/routing-table.md) work before reading this part.

After Gate start, the first message you receive is a route table.<br/>
This first message is really important, because it contain all the informations allowing you to create a code object for your device containing all its features.

### Routing table structure
A routing table in JSON is a list of nodes

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

#### Nodes informations
Each listed node of the network has basic node informations and a list of hosted modules:

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
> **Note:** To understand meanings of *uuid* and *port_table*, pleas refer to the [routing table page](/_pages/low/modules/routing-table.md).

#### Modules
Each listed module of a node has basics modules informations:

```JSON
{ // module 1
   "type":"Type",
   "id":1,
   "alias":"Alias"
}
```
> **Note:** To understand meanings of *type*, *id* and "alias", pleas refer to the [module page](/_pages/low/modules.html).

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

Visual representation of this route table:

![](/_assets/img/luos-network-ex.png)


### Module's information messages

When route table Json is transmitted Gate start to update and stream your network data with modules informations.

This Json is a "modules" object listing all modules by alias and all those modules contains values :
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

You can use the exact same Json object structure to send datas to modules.

Here is the list of all values that can be used by modules:

|value name|Definition|
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
|reinit|Reinitialisation command|
|control|Control command (play, pause, stop, rec)|
|color|Color value|
|io_state|IO state|
|led|Board's LED|
|L0_temperature|Node's temperature|
|L0_voltage|Node's voltage|
|uuid|Module's uuid|
|rename|Renaming an alias|
|revision|Firmware revision|

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

Module can be excluded of the network if a problem occurs (See [message handling](/_pages/low/modules/msg-handling.html#module-exclusion) for more information). In this case Gate send an exclusion message indicating that this module is no longer available :
```JSON
{"dead_module": "module_alias"}
```


<div class="cust_edit_page"><a href="https://{{gh_path}}/_pages/high/json-api.md">Edit this page</a></div>
