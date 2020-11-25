# Object Dictionary
To keep interoperability between <span class="cust_tooltip">containers<span class="cust_tooltiptext">{{container_def}}</span></span>, Luos provides an Object Dictionary (OD).

## What is OD?
An Object Dictionary (OD) allows different developers of different containers to make them interoperate regardless of the unit they uses on the container.

Let's take an example: If container1 uses an angle as radians and container2 uses degrees, what is the unit they should use to share the angle information?

An Object Dictionary defines a set of typical objects that can be transmitted through Luos messages. It allows to send these objects with a predefined type and to use it in the units the user want.

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
As many units exist, many conversion functions are available. As a result, they follow a **logic naming rules** in order to easily find a desired function without having to search for it.

### Unit conversions
There are two types of unit conversion: in one way (OD type from desired unit), and in the other way (OD type to desired unit):

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
The same both way of conversion are available for messages (OD type from message and OD type to message):

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
| current |  mA, A |
| power | mW, W |
| ratio | percentage |
| temperature | deg_c, deg_f, deg_k |
| color | 8bit_RGB unsigned char \[3\] |

> **Note:** to find out a conversion function, replace the characters `/` or `.` in the units by the character `_`. The character `µ` is replaced by `u`, and `revolution` is replaced by `rev`.
>
> Examples: convert a linear speed to mm/s: `LinearOD_SpeedTo_mm_s()`; convert a value in &mu;m to a linear position: `LinearOD_PositionFrom_um()`; convert a value in revolutions/s to an angular speed: `AngularOD_SpeedFrom_rev_s()`;


