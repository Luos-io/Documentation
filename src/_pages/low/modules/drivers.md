# Drivers

This page details what is a driver, how to create and initialize one, and how to use it.

## What is a driver?
A driver is a type of module. A driver is a <span class="cust_tooltip">**module**<span class="cust_tooltiptext">{{module_def}}</span></span> which is linked to its own hardware in a Luos architecture and can manage it (send order or retrive values, for example). Drivers don't have access to other modules but [apps](/_pages/low/modules/apps.md) can access them (send and receive messages) to control their associated hardware. 

> **Note:** Theoretically, we chose separate the definitions of an app and a driver: they are both modules but they don't have the same functions into a Luos architecture. However, it's practically possible to create a module which can be in the same time an app and a driver, that is, a module which controls hardware and can have access to any other module.

## Driver properties
As a driver is a module, it has the same properties allowing to other modules to recognize and access it:

| Name | Description | Format |
| :---: | :---: | :---: |
| **ID** | The ID is an unique number given to each module depending on their physical position. The system automatically assign each ID during the detection phase. | Integer<br />e.g. `Id=1` |
| **TYPE** | The type defines the module purpose. A few types are predefined and can be used, or new ones can be created. The module type cannot be changed after module initialization. | String<br />e.g. `type=DISTANCE_MOD` |
| **ALIAS** | Alias is the name of the module. It is used to easily identify a module. Each module has a **default alias** who can be changed by users. For example, a module with the default alias `motor_mod` can be named `left_knee_motor` by user. This new name will be stored in the non-volatile memory of the board. As we do not want to have multiple modules with the same name, a duplicate name on your system will be automatically assigned with an incrementing number at its end, in the network. You can go back to the default name by setting a void name (`""`) to a module. | String<br />e.g. `alias="gate"` |

## How to create and initialize a driver
As a driver is a module, it is created and initialized the same way than any module:

A driver is initialized by declaring a variable with a `module_t*` structure type (module pointer), and implementing a void function with the format name `[name_of_module]_init` and void argument.

Inside this function, assign the previous declared variable to the function `luos_module_create(callback, module type, default alias)`, with the following arguments:
 - *callback*: Set the real-time configuration to use (see [Real-time configuration page](/_pages/low/modules/rt-config.md) for more details).
 - *module type*: Set the type of the new module. e.g. `DISTANCE_MOD`, `VOLTAGE_MOD`, etc. 
 - *default alias*: Set the alias by default for the new module. e.g. `MyModule02`.
 
> **Note:** To better define a module as a driver, the type can be one of the [existing driver's types](#drivers-types). However, this is an abstract choice because a driver is a module, and the type only helps to know what kind of module you want to create. Also, new types can be created if needed.
 
See the following code as an example:

```c
module_t* module;

void motor_init(void) {
    module_t* module = luos_module_create(rx_btn_cb, CONTROLLED_MOTOR_MOD, "motor_mod");
}
```

> **Note:** According to the real-time configuration you chose, an additional line of code may be necessary. See [Real-time configuration page](/_pages/low/modules/rt-config.md) for more details.

## Driver's types
Here is the list `module_type_t` of existing driver's types. New types can also be created as a list in your `main` file.

```c
SERVO_MOD // Control servomotors
COLOR_MOD // Control RGB Leds
ANGLE_MOD // Control potentiometers
STATE_MOD // Control states og GPIO pins
DISTANCE_MOD // Control distance sensors
VOLTAGE_MOD // Control the voltage of GPIO pins
DYNAMIXEL_MOD // Control Dynamixel servomotors
STEPPER_MOD // Control stepper motors
DCMOTOR_MOD // Control DC motors
IMU_MOD // Control IMU sensors
LIGHT_MOD // Control light sensors
CONTROLLED_MOTOR_MOD // Control DC motors with a Hall-effect sensor and a reducer.
```

<div class="cust_edit_page"><a href="https://{{gh_path}}/_pages/low/modules/drivers.md">Edit this page</a></div>
