---
layout: default
title: "General use"
---
{% include var.md %}

# General guide to Luos technology

Luos is a simple and lightweight [distributed operating system](https://en.wikipedia.org/wiki/Distributed_operating_system) dedicated to embedded systems. It's a powerful tool using modularity to simplify and link any component or application code together as a [single system image](https://en.wikipedia.org/wiki/Single_system_image).

This guide contains all the basic notions you will need to use, create and understand Luos technology.

## What is a node
A node is a physical component (hardware) running Luos and hosting one or several modules. In a Luos network, nodes are all connected together using Robus, the Luos communication technology.<br/>In other words, **a node is a microcontroler** connected to other microcontrolers and running Luos.

A **Luos board** hosts a single node. Each one provides a robotic function (gate, sensor, actuation, etc.).

In the Luos philosophy, each node has to carry the necessary programs (modules) allowing to manage the boards and devices hosting it.

It is possible to have multiple nodes in the same electronic board, but each node will be seen by Luos as a different node and has to manage separate devices, whatever the number of boards.

## What is a module
A module is a block of code which is able to communicate with any other modules in the Luos network. Each module has a particular task such as managing a motor, handling a laser range finder, or compute an inverse-kinematics, for example.
**Each module is hosted in a single node**, but a node can handle several modules at the same time and manage communication between them and between other modules hosted in other nodes, using the same interface.

For example, the [Dynamixel board](/board/dxl) provided by Luos Robotics can dynamically create and manage Dynamixel modules depending on the number of Dynamixel motors linked to it. Any Dynamixel module can get or set values to other Dynamixel modules on the same node or to any other module in any other node of the network.

![feature-module-node-board](/assets/img/feature-module-node-board.jpg)

## Module basic information
To properly work, each module needs to get some information allowing to other modules to recognize and access it:

 - **ID**: The ID is an unique number given to each module depending on their physical position. The system automatically assign each ID during the detection phase. If you move a module from a microcontroler A to a microcontroler B on a given robot, the ID will change. In the same way, if you change the wiring order of a microcontroler on the network on a given robot, the ID will change too.
 - **TYPE**: The type definea the module purpose. For example: distance sensor, servomotor, ... You can use predefined types or create your owns. The module type can't be changed after module initialization.
 - **ALIAS**: Alias is the name of the module. It's used to easily identify a module. Each module has a **default alias** who can be changed by users. For example, a module with the default alias `motor_mod` can be named `left_knee_motor` by user. This new name will be stored in the non-volatile memory of the board. As we don’t want to have multiple modules with the same name, a duplicate name on your system will be automatically assigned with an incrementing number at its end, in the network. You can go back to the default name by setting a void name (`""`) to a module.

## Node basic capacities
Nodes can have capacities such as measuring the core temperature, sending the processor unique ID, or input voltage. Nodes capacities are commonly shared with all modules hosted in. So any module hosted in a node is able to send its input voltage.

## Route table
A route table is a "service" managed by the Luos network and available for any module in any node. This service lists all the modules and allow to any module to get and use basic information of any other modules. The routing table's data can be loaded or auto-generated during detection.

## Module detection
The module detection assigns IDs to modules depending on their node's physical position on the cabling, and generates a route table.

IDs are assigned from the nearest to the furthest node branch by branch, from the point of view of the module running the detection. Following this logic, the module running the detection will have the ID 1, the next one will have the ID 2, etc.

Multiple detection by different modules at the same time is not allowed.

It is possible to detect the network frequently to discover included or excluded modules at run time.

## Applications (App)
Module are just pieces of code, so they are not limited to managing hardware drivers. We call application or app a module who only manages software items.
For example, you can create an app to compute the inverse-kinematic of a robotic arm. In this case, you can send an arm target position to this app so that it computes and sends orders to each motor modules it handles in order to reach the target.
Applications can be placed in any nodes on your network without any modification, but the node choice can impact global performances of the system.
