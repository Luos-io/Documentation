---
layout: post
title: "Create and initialize a module"
categories: 2_modules
desc: How to create and initialize a Luos module.
order: 0
wip: 1
---
{% include var.md %}

<div class="wip_img"></div>
<blockquote class="warning"><strong>Work in progress</strong><br /><br />We are working on this page...</blockquote><br />

A module is a block of code which is able to communicate with any other modules in the Luos network. Each module provides a particular set of tasks such as managing a motor, handling a laser range finder, or compute an inverse-kinematics.
Each module is hosted in a single node (MCU), but a node can handle several modules at the same time and manage communication between them and between other modules hosted in other nodes, using the same interface.

Theoretically, a module can be a driver (managing hardware) or an app (managing software items). However, a module can also be a mix between both, hosting driver functions along with app functions, such as a [gate](/../modules_list/gate).

# Module properties
To properly work, each module owns some properties allowing to other modules to recognize and access it:

{{ table_prop_module }}
	
# How to create and initialize a module
A module is initialized by declaring a variable with a `module_t*` structure type (module pointer), and implementing a void function with the format name `[name_of_module]_init` and void argument.

Inside this function, assign the previous declared variable to the function `luos_module_create(callback, module type, default alias)`, with the following arguments:
 - *callback*: Set the real-time configuration to use (see [Real-time configuration page](rt-config) for more details).
 - *module type*: Set the type of the new module. e.g. `DISTANCE_MOD`, `VOLTAGE_MOD`, etc.
 - *default alias*: Set the alias by default for the new module. e.g. `MyModule02`.
 
See the following code as an example for a button:

```c
module_t* module;

void button_init(void) {
    module_t* module = luos_module_create(rx_btn_cb, STATE_MOD, "button_mod");
}
```

> **Note:** According to the real-time configuration you chose, an additional line of code may be necessary. See [Real-time configuration page](rt-config) for more details.
