---
layout: default
title: "General use"
---
{% include var.md %}

# General guide to Luos technology

Luos is a simple and lightweight [distributed operating system](https://en.wikipedia.org/wiki/Distributed_operating_system) dedicated to embedded systems. A powerful tool using modularity to simplify and link any component or application code together as a [single system image](https://en.wikipedia.org/wiki/Single_system_image).

This guide contains all basic notions you will need to use, create and understand Luos things.

## What is a node
A node is a physical component running Luos and hosting modules. In a Luos network nodes are all connected together using Robus, the Luos communication technology.<br/>In other word a node is a microcontroler connected to other microcontrolers and running Luos.

In the Luos phylosophy each node have to carry with him all necessary programs (modules) allowing to manage the board and devices it hosting.

It is possible to have multiple nodes in the same electronic board but each one will be see as different node by Luos and each one have to manage separate devices.

## What is a module
A module is a code bloc able to communicate with any other modules in the Luos network. Each module have a particular task such as manage a motor, handle a laser range finder, or compute an inverse-kinematics for example.
**Each module is hosted in a single node**, but a node can handle several modules at the same time and manage communication between them and between other modules hosted in other nodes using the same interface.

For example the dynamixel node provided by Luos Robotics can dynamically create and manage modules depending on the number of dynamixel linked to it. Any dynamixel module can't get or set value to other dynamixel modules on the same node or to any other module in any other node of the network.

## Module basic informations
To work each modules need to have some information allowing to other modules to recognize and access to it :

 - **ID** : The ID is an unic number given to each module depending on their physical position. You don't need to define ID by yourself, the system will distribute it during the detection phase. If you move a module from microcontroler A to microcontroler B on a given robot, the ID will change. The same way if you change the wiring order of microcontroler on the network on a given robot, the ID will change too.
 - **TYPE** : The type define the module purpose. For example distance sensor, servomotor... You can use predefined type or create your own. The module type can't be changed after module initialization.
 - **ALIAS** : Alias is the name of the module. This alias is used to easily identify a module. Each module have a **default alias** who can be changed by users. For example a module with the default alias "motor_mod" can be named "left_knee_motor" by user. This new name will be stored in the non-volatile memory of the board. As we don’t want to have multiple modules with the same name, a duplicate name on your system will be automatically renamed with an incrementing number at the end, in the network. You can go back to the default name by setting a void name (`""`) to a module.

## Node basic capacities
Node can have capacities such as measure core temperature, send processor unic ID, or input voltage... Node capacities are commonly shared with all modules hosted in. So any module hosted in a node able to share it's voltage input will be capable of sending this input voltage.

## Route table
A route table is a "service" managed by the Luos network and available for any module in any node. This service list all modules and allow to any module to get and use basic informations of any other modules. The routing table datas can be loaded or auto-generated during detection.

## Module detection
The module detection allow to distribute ID to modules depending on their node physical position on the cabling and generate a route table.

IDs are distributed from the nearest to the furthest branch by branch in the point of view of the module running the detection. Following this logic the module running the detection will have the ID 1, the next one will have the ID 2, etc ...

Multiple detection by different module at the same time is not allowed.

It is possible to detect the network frequently to discover included or excluded modules at run time.

## Applications (App)
Module are just piece of code, so they are not limited to manage hardware drivers. We call application or app a module who only manage software things.
For example you can create an app to compute inverse-kinematic of a robotics arm. In this case you can send an arm target to this App and it will compute and send orders to each motor modules he handle to reach the target.
Applications can be placed in any nodes on your network without any modifications, but the node choice can impact global performance of the system.
