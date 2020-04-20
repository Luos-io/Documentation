# Modules

A module is a block of code which is able to communicate with any other modules in the Luos network. 

A module can be an [application](/pages/low/modules/create-modules.html#apps-guidelines) or a [driver](/pages/low/modules/create-modules.html#drivers-guidelines).

Each module provides a particular set of tasks such as managing a motor, handling a laser range finder, or compute an inverse-kinematics.
Each module is hosted in a single <span class="cust_tooltip">node<span class="cust_tooltiptext">{{node_def}}</span></span> (MCU), but a node can handle several modules at the same time and manage communication between them and between other modules hosted in other nodes, using the same interface.

**As a developer you will always develop your functionalities into modules, and never into the `main()` program.** The only information that should be put on the `main()` code are MCU setup parameters and modules' run functions.

## Module properties
To properly work, each module owns some properties allowing to other modules to recognize and access it:

| Name | Description | Format |
| :---: | :---: | :---: |
| **ID** | The ID is a unique number given to each module depending on their physical position. The system automatically assigns each ID during the [detection phase](/pages/overview/general-basics.html#module-detection). If you move a module from a microcontroller A to a microcontroller B on a given device, the ID will change. In the same way, if you change the wiring order of a microcontroler on the network on a given device, the ID will change too. | Integer<br />e.g. `Id=1` |
| **TYPE** | The type defines the module purpose. A few types are predefined and can be used, or new ones can be created. The module type can't be changed after module initialization. | String<br />e.g. `type=DISTANCE_MOD` |
| **ALIAS** | Alias is the name of the module. It's used to easily identify a module. Each module has a **default alias** which can be changed by users. For example, a module with the default alias `motor_mod` can be named `left_knee_motor` by user. This new name will be stored in the non-volatile memory of the board. As we don't want to have multiple modules with the same name, a duplicate name on your system will be automatically assigned with an incrementing number at its end, in the network. You can go back to the default name by setting a void name (`""`) to a module. | String<br />e.g. `alias="gate"` |

<div class="cust_edit_page"><a href="https://{{gh_path}}/pages/low/modules.md">Edit this page</a></div>
