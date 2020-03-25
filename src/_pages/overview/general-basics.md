# General guide to Luos technology

Luos is a simple and lightweight <a href="https://en.wikipedia.org/wiki/Distributed_operating_system" target="_blank">distributed operating system</a> dedicated to embedded systems enabling <a href="https://en.wikipedia.org/wiki/Microservices" target="_blank">microservices</a> architecture for electronics. It's a powerful tool using modularity to simplify and link any hardware component or application code together as a <a href="https://en.wikipedia.org/wiki/Single_system_image" target="_blank">single system image</a>.

This guide contains all the basic notions you will need to use, create and understand Luos technology.

Luos is a low-level software technology uploaded into every board (<span class="cust_tooltip">[**node**](#node)<span class="cust_tooltiptext">{{node_def}}</span></span>) in a device.
You can use Luos as a lib for **bare metal** development or as a driver into your **embedded OS**.

Luos is composed as well of **code subdivisions** called <span class="cust_tooltip">[**modules**](#module)<span class="cust_tooltiptext">{{module_def}}</span></span>. Modules are distributed into every nodes in a network.

<a href="/_assets/img/feature-module-node-board.jpg" target="_blank"><img src="/_assets/img/feature-module-node-board.jpg" width="800px" /></a>

<a name="node"></a>
## Node
A node is a physical component (hardware) running Luos and hosting one or several modules. In a Luos network, nodes are all connected together using Robus, the Luos communication technology.<br/>In other words, **a node is a microcontroler** connected to other microcontrolers running Luos.
In the Luos philosophy, each node has to carry the necessary programs (modules) allowing it to manage its boards and features.

<img src="/_assets/img/MCU-luos.png" height="100px" />

Nodes can have capacities such as measuring the core temperature, sending the processor's unique ID or input voltage. A node's capacities are commonly shared by all the modules hosted into it and are accessible through each of them.

<a name="module"></a>
## Module
A module is a block of code which is able to communicate with any other modules through the Luos network. Each module provides an API allowing to manage a motor, handle a laser range finder, or compute an inverse-kinematics, for example.
**Each module is hosted in a single node**, but a node can handle several modules at the same time and manage communication between them and between other modules hosted in other nodes, using the same interface.

For example, the [Dynamixel board]({{boards_path}}/dxl.md) provided by Luos can dynamically create and manage Dynamixel modules depending on the number of Dynamixel motors linked to it. Any Dynamixel modules can get or set values to other Dynamixel modules on the same node or to any other modules in any other nodes in the network.

## Module basic properties
A module own properties allowing to other modules to recognize it and have access to it:

| Name | Description | Format |
| :---: | :---: | :---: |
| **ID** | The ID is an unique number given to each module depending on its physical position. The system automatically assigns each ID during the detection phase. If you change the wiring order of nodes on the network, the IDs of your modules will change. | Unsigned Integer 12 bits<br />e.g. `Id=1` |
| **TYPE** | The type defines the module purpose. A few types are predefined and can be used, and new ones can be created. The module's type can't be changed while it's running. | Enum Unsigned Integer 12 bits<br />e.g. `type=DISTANCE_MOD` |
| **ALIAS** | Alias is the name of the module. It is used to easily identify a module. Each module has a **default alias** which can be changed by users. For example, a module with the default alias `motor_mod` can be named `left_knee_motor` by the user. This new name will be stored in the non-volatile memory of the board. As we don't want to have multiple modules with the same name, a duplicate name on your system will be automatically assigned with an incrementing number at the end, into the network. You can go back to the default name by setting a void name (`""`) to a module. | String 16 bytes<br />e.g. `alias="gate"` |

## Routing table
A routing table is a "service" managed by the Luos network and available for any modules in any nodes. This service lists all the modules and allows to any modules to get and use basic information of any other modules. The routing table's data can be loaded or auto-generated during detection.

[Go to Routing table page](/_pages/low/modules/routing-table.md).

## Module detection
The module detection assigns IDs to modules depending on their node's physical position in the network, and generates a routing table.

IDs are assigned from the nearest to the furthest node branch by branch, from the point of view of the module running the detection. Following this logic, the module running the detection will have the ID 1, the next one will have the ID 2, etc.

> *Note:* Multiple detection by different modules at the same time is not allowed.

It is possible to detect the network frequently in order to dynamically discover included or excluded modules while running.

## Modules' types
There are two types of modules:

### Drivers
Drivers are modules managing hardware. They are passive modules that retrieve pieces of information from electronic devices, format them, and share them through the Luos API.

[Go to Drivers page](/_pages/low/modules/drivers.md).

### Applications (App)
An application or app is a module which only manages software items such as functions.
For example, you can create an app to compute the inverse-kinematic of a robotic arm. In this case, you can send an arm target position to this app so that it would compute and send orders to each motor modules (drivers) it handles, in order to reach the target.
Apps can be placed in any nodes on your network without any hardware or code modifications, but the choice of the hosting node can impact global performances of the system.

[Go to Apps page](/_pages/low/modules/apps.md).

<div class="cust_edit_page"><a href="https://{{gh_path}}/_pages/first_steps/general-use.md">Edit this page</a></div>
