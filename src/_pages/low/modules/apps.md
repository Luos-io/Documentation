# Applications (apps)

This page details what is an app, how to create and initialize one, and how to use it.

## What is an app?
An applications or app is a type of module. An app is a module which only manages software items such a functions, and which can communicate with and use any other module ([drivers](/_pages/low/modules/drivers.md) or apps). An app can't have access to hardware like a driver can.
Apps can be placed in any <span class="cust_tooltip">[**nodes**](#node)<span class="cust_tooltiptext">{{node_def}}</span></span> on a Luos network without any hardware or code modifications. However, the choice of the hosting node can impact global performances of the system.

> **Note:** Theoretically, we separate the definitions of an app and a driver: they are both modules but they don't have the same functions into a Luos architecture. However, it's practically possible to create a module which can be in the same time an app and a driver, that is, a module which controls hardware and can have access to any other module.


## App properties
As an app is a module, it has the same properties allowing to other modules to recognize and access it:

| Name | Description | Format |
| :---: | :---: | :---: |
| **ID** | The ID is an unique number given to each module depending on their physical position. The system automatically assign each ID during the detection phase. If you move a module from a microcontroller A to a microcontroller B on a given robot, the ID will change. In the same way, if you change the wiring order of a microcontroler on the network on a given robot, the ID will change too. | Integer<br />e.g. `Id=1` |
| **TYPE** | The type defines the module purpose. A few types are predefined and can be used, or new ones can be created. The module type cannot be changed after module initialization. | String<br />e.g. `type=DISTANCE_MOD` |
| **ALIAS** | Alias is the name of the module. It is used to easily identify a module. Each module has a **default alias** who can be changed by users. For example, a module with the default alias `motor_mod` can be named `left_knee_motor` by user. This new name will be stored in the non-volatile memory of the board. As we do not want to have multiple modules with the same name, a duplicate name on your system will be automatically assigned with an incrementing number at its end, in the network. You can go back to the default name by setting a void name (`""`) to a module. | String<br />e.g. `alias="gate"` |


### Other apps' characteristics
Apps allow to create and use new kinds of messages (besides regular Luos messages) by adding new types over the defined Luos types. As a result, apps use the Robus and Luos protocols, along with messages specific to the application. Their inputs and outputs are not entirely managed by Luos, by the gates or by Robus. Therefore, some additional code modifications may be necessary to allow proper *special* message reception into the gates or into Pyluos.

Apps can't be managed by Pyluos, except for run/stop flag.

Apps are not managed by the gates, except for regular Luos-type messages.


## How to create and initialize an app
As an app is a module, it is created and initialized the same way than any module:

An app is initialized by declaring a variable with a `module_t*` structure type (module pointer), and implementing a void function with the format name `[name_of_module]_init` and void argument.

Inside this function, assign the previous declared variable to the function `luos_module_create(callback, module type, default alias)`, with the following arguments:
 - *callback*: Set the real-time configuration to use (see [Real-time configuration page](/_pages/low/modules/rt-config.md) for more details).
 - *module type*: Set the type of the new module. e.g. `DISTANCE_MOD`, `VOLTAGE_MOD`, etc.
 - *default alias*: Set the alias by default for the new module. e.g. `MyModule02`.

> **Note:** To better define a module as an app, the type can be **APP_MOD**. However, this is an abstract choice because an app is a module, and the type only helps to know what kind of module you want to create. Also, new types can be created if needed.

See the following code as an example:

```c
module_t* module;

void app_init(void) {
    module_t* module = luos_module_create(rx_btn_cb, APP_MOD, "app_mod_1");
}
```

> **Note:** According to the real-time configuration you chose, an additional line of code may be necessary. See [Real-time configuration page](/_pages/low/modules/rt-config.md) for more details.

<div class="cust_edit_page"><a href="https://{{gh_path}}/_pages/low/modules/apps.md">Edit this page</a></div>
