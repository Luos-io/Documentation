---
custom_edit_url: null
---

# Object dictionary

An object dictionary (OD) allows various developers of different services to make them interoperate regardless of the units they use on their code.

> Example: If _my_service1_ uses an angle in radians and _my_service2_ uses degrees, what is the unit they should use to share the angle information?

An object dictionary defines a set of typical objects that can be transmitted through Luos messages. It allows to send these objects with a unit and to use it in any other units, in other services.

Luos defines objects based on physical values following the SI standard.

## Objects and types

Each object in the Object Dictionary has a specific Type. For example:

```c
// Define object angular_position as an angular_position_t type
angular_position_t angular_position;
```

You can create your variables using these objects but **never set OD variables directly with a value**. Instead, you have to use functions available on the Luos OD:

```c
// Set object angular_position
float deg = 12.0;
angular_position_t angular_position = AngularOD_PositionFrom_deg(deg);
```

Following this rule, everybody will be able to use your values.

All the types are listed in the [table summary](#types-and-units-table-summary) at the end of this page.

## Conversions

As many units exist, many conversion functions are available. As a result, they follow **logic naming rules** in order to quickly find the desired function without having to search for it.

### Unit conversions

There are two types of unit conversion: in one way (OD type from the desired unit), and in the other way (OD type to the desired unit):

- **`from` conversion:** Converts a value with a defined unit into a desired OD data.

Format: `[type_var] = [type]From_[unit]([value])`

```c
// save a linear_position from a mm value
linear_position_t linear_position =  LinearOD_PositionFrom_mm(float mm);
```

- **`to` conversion:** Converts an OD data into a specific unit.

Format: `[value] = [type]To_[unit]([type_var])`

```c
// convert the variable linear_position into mm
float mm = LinearOD_PositionTo_mm(linear_position_t linear_position);
```

### Messages conversions

In the same way, both conversions are available for messages (OD type **from** message and OD type **to** message):

- **`from` conversion:** Gets a OD data from a message.

Format: `[type]FromMsg([type_var], msg)`

```c
// get the linear_position from the message msg
void LinearOD_PositionFromMsg(linear_position_t* linear_position, msg_t* msg);
```

- **`to` conversion:** Inserts a desired OD data into a message.

Format: `[type]ToMsg(type_var], msg)`

```c
// insert the linear_position into the message msg
void LinearOD_PositionToMsg(linear_position_t* linear_position, msg_t* msg);
```

## Types and units table summary

Here are listed the existing types:

|       Type       |               Available prefix and other units                |
| :--------------: | :-----------------------------------------------------------: |
| linear_position  |             nm, &mu;m, mm, cm, m, km, in, ft, mi              |
|   linear_speed   |                  mm/s, m/s, km/h, in/s, mi/h                  |
| angular_position |                     deg, revolution, rad                      |
|  angular_speed   |          deg/s, revolution/s, revolution/min, rad/s           |
|      force       |                       N, kgf, ozf, lbf                        |
|      moment      |    N.mm, N.cm, N.m, kgf.mm, kgf.cm, kgf.m, ozf.in, lbf.in     |
|     voltage      |                             mV, V                             |
|     current      |                             mA, A                             |
|      power       |                             mW, W                             |
|      ratio       |                          percentage                           |
|   temperature    |                      deg_c, deg_f, deg_k                      |
|      color       |                 8bit_RGB unsigned char \[3\]                  |
|     control      |             control_t (play, pause, stop, record)             |
|       pid        | asserv_pid_t float \[3\] {proportional, integral, derivative} |

:::tip
To find out what conversion function to use if you don't know it, replace the characters `/` or `.` in the units by the character `_`. The character `µ` is replaced by `u`, and `revolution` is replaced by `rev`.

Examples:<br/>
convert a linear speed to mm/s: `LinearOD_SpeedTo_mm_s()`;<br/>
convert a value in &mu;m to a linear position: `LinearOD_PositionFrom_um()`;<br/>
convert a value in revolutions/s to an angular speed: `AngularOD_SpeedFrom_rev_s()`;<br/>
:::
