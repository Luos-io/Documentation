# JSON API
The [Gate module]({{modules_path}}/gate.md) converts Luos messages from a device's network into <a href="https://en.wikipedia.org/wiki/JSON" target="blank_">JSON formated data</a>, and the other way from JSON to Luos messages. The JSON data format is very common and widely used by many programming languages.

## Message types

Three types of messages can be converted to JSON and back: [routing table messages](#routing-table-messages), [module's information messages](#modules-information-messages), and [module exclusion messages](#module-exclusion-messages).

### Routing table messages
Luos uses a routing table, a library which contains a data table listing every <span class="cust_tooltip">nodes<span class="cust_tooltiptext">{{node_def}}</span></span> and <span class="cust_tooltip">modules<span class="cust_tooltiptext">{{module_def}}</span></span> connected to a Luos Network. More information on the [Routing table page](/_pages/low/modules/routing-table.md).


#### Routing table
A routing table in JSON format has the following structure:

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

#### Node
Each listed node of the network has the following strcture, with the following parameters:

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

|Parameter|Name|Definition|Format|
|:---:|:---:|:---:|:---:|
|Node's id|`"uuid"`|The uuid is the unique id of a node. |`luos_uuid_t ` structure|
|Node's port|`"port_table"`|Display the ports connected to the node, that is the connectors.|Table of two integers:<br />`"port_table":[int, int]`|
|Node's modules|`"modules"`|All the modules existing into the node. See the next table for more details|Table of one or several module objects.<br /> `"modules":[{//module 1},{//module 2},...]`|


#### Modules
Each listed module of the node has the following structure, with the following parameters:

```JSON
{ // module 1
   "type":"Type_example",
   "id":1,
   "alias":"Alias_example"
}
```

|Parameter|Name|Definition|Format|
|:---:|:---:|:---:|:---:|
|Module's type|`"type"`|The module's type.|`module_type_t`|
|Module's id|`"id"`|The module's id.|`uint16_t`|
|Module's alias|`"alias"`|The module's name|String|

#### Full routing table example

The following JSON data is an example of a routing table: 
```JSON
{
   "route_table":[
      {
         "uuid":[
            2031684,
            1112756496,
            540423216
         ],
         "port_table":[
            2,
            65535
         ],
         "modules":[
            {
               "type":"Gate",
               "id":1,
               "alias":"r_right_arm"
            }
         ]
      },
      {
         "uuid":[
            4915239,
            1194612503,
            540554032
         ],
         "port_table":[
            4,
            1
         ],
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
         "uuid":[
            2818086,
            1194612503,
            540554032
         ],
         "port_table":[
            5,
            3
         ],
         "modules":[
            {
               "type":"Imu",
               "id":4,
               "alias":"gps"
            }
         ]
      },
      {
         "uuid":[
            2097186,
            1194612503,
            540554032
         ],
         "port_table":[
            65535,
            4
         ],
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

Visual of the associated network:

![](/_assets/img/luos-network-ex.png)


### Module's information messages

The module's information messages can be converted from message to JSON and from JSON to message.

Here is the list of all JSON objects:

|Object|Definition|
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
|register|Memory register|
|set_id|A set id command|
|wheel_mode|The wheel mode parameter for Dynamixel servomotors|

 - For [Stepper]({{modules_path}}/stepper.md), [Controlled-motor]({{modules_path}}/controlled-motor.md) and [Servo]({{modules_path}}/servo.md) modules:

|Object|Definition|
|:---:|:---:|
|parameters|Parameters for IMU|


 - For [Dynamixel]({{modules_path}}/gate.md) module:

|Object|Definition|
|:---:|:---:|
|delay|Delay time|

### Module exclusion messages 

The exclusion mesages happen when a module in a Luos network is not responsive. It's then removed from the routing table.


<div class="cust_edit_page"><a href="https://{{gh_path}}/_pages/high/json-api.md">Edit this page</a></div>
