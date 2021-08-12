# Command and object dictionary

Each message that follows the Robus communication protocol includes a command value that defines the type of the contents of the message's data.

Another feature included in Luos technology is the Object Dictionary (OD), which aims to maintain interoperability between <span class="cust_tooltip">services<span class="cust_tooltiptext">{{service_def}}</span></span>.

## Commands

There are three general categories of commands that are separated regarding the handling level of the messages received from a node. These messages are either handled at Luos level, at Robus level, or at service level.

You can find the complete list of commands <a href="https://github.com/Luos-io/Luos/blob/master/inc/luos_list.h" target = "_blank">here</a>.

### Commands handled at Robus level

| Command | Function |
| :---: | :---: |
| WRITE_NODE_ID | Set a new ID to the node |
| RESET_DETECTION | Reset detection process - Reinitialize IDs to 0 |
| SET_BAUDRATE | Set Robus Baudrate |
| ASSERT | Node Assert message  |

### Commands handled at Luos level

| Command | Function |
| :---: | :---: |
| RTB_CMD | Ask, generate or share the routing table |
| WRITE_ALIAS | Get and save a new given alias |
| UPDATE_PUB | Ask to update a sensor value each time duration to the sender |
| NODE_UUID | luos_uuid_t |
| REVISION | Service sends its firmware revision |
| LUOS_REVISION | Service sends its Luos revision |
| LUOS_STATISTICS | Service sends its Luos revision |

### Service commands

This last category includes all the specific commands that are treated by each service. Each one refers to different kinds of services. For example, the command IO_STATE is sent from services of type STATE, and it shows if the actual state of the service is true or false.

If you want to create new functionalities for your custom services, you can create and add your own commands. [See how](../../tutorials/tutorials.md).

## What is an object dictionary?

An object dictionary (OD) allows different developers of different services to make them interoperate regardless of the unit they use on the service.

> Example: If *my_service1* uses an angle in radians and *my_service2* uses degrees, what is the unit they should use to share the angle information?
 
An object dictionary defines a set of typical objects that can be transmitted through Luos messages. It allows to send these objects with a predefined type and to use it in the units the user want.

## How is it managed in Luos?

Luos defines objects based on physical values following the SI standard.

### Object and types

Each object in the Object Dictionary has a specific Type. For example:
```c
// Define object angular_position as an angular_position_t type
angular_position_t angular_position; 
```

You can create your variables using these objects but **never set OD variables directly with a value**. Instead, you have to use functions available on the Luos OD:
```c
// Set object angular_position
angular_position_t angular_position = AngularOD_PositionTo_deg (12.0); 
```
Following this rule, everybody will be able to use your values.

All the types are listed in the [table summary](#types-and-units-table-summary) at the end of this page.

## Conversions

As many units exist, many conversion functions are available. As a result, they follow **logic naming rules** in order to quickly find the desired function without having to search for it.

### Unit conversions

There are two types of unit conversion: in one way (OD type from the desired unit), and in the other way (OD type to the desired unit):

 - **`from` conversion:** Converts a value with a defined unit into a desired OD type. Format:  `[type_var] = [type]From_[unit]([value])`
```c
// save a linear_position from a mm value
linear_position_t LinearOD_PositionFrom_mm(float mm); 
```

 - **`to` conversion:** Converts a type to a unit. Format: `[value] = [type]To_[unit]([type_var])`
```c
// convert the variable linear_position into a mm
float LinearOD_PositionTo_mm(linear_position_t linear_position); 
```

### Messages conversions

In the same way, both conversions are available for messages (OD type **from** message and OD type **to** message):

 - **`from` conversion:** Gets a type from a message. Format: `[type]FromMsg([type_var], msg)`
```C
// get the linear_position from the message msg
void LinearOD_PositionFromMsg(linear_position_t* linear_position, msg_t* msg); 
```

 - **`to` conversion:** Inserts a desired type into a message. Format: `[type]ToMsg(type_var], msg)`
```c
// insert the linear_position into the message msg
void LinearOD_PositionToMsg(linear_position_t* linear_position, msg_t* msg);
```

## Types and units table summary

Here are listed the existing types:

| Type | Available prefix and other units |
| :---: | :---: |
| linear_position | nm, &mu;m, mm, cm, m, km, in, ft, mi |
| linear_speed | mm/s, m/s, km/h, in/s, mi/h |
| angular_position | deg, revolution, rad |
| angular_speed | deg/s, revolution/s, revolution/min, rad/s |
| force | N, kgf, ozf, lbf |
| moment | N.mm, N.cm, N.m, kgf.mm, kgf.cm, kgf.m, ozf.in, lbf.in |
| voltage | mV, V |
| current | mA, A |
| power | mW, W |
| ratio | percentage |
| temperature | deg_c, deg_f, deg_k |
| color | 8bit_RGB unsigned char \[3\] |

> **Note:** to find out a conversion function, replace the characters `/` or `.` in the units by the character `_`. The character `µ` is replaced by `u`, and `revolution` is replaced by `rev`.
>
> Examples: convert a linear speed to mm/s: `LinearOD_SpeedTo_mm_s()`; convert a value in &mu;m to a linear position: `LinearOD_PositionFrom_um()`; convert a value in revolutions/s to an angular speed: `AngularOD_SpeedFrom_rev_s()`;
