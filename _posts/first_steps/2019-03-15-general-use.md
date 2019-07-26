---
layout: post
title: "General use"
categories: 0_first_steps
desc: A general guide to Luos technology.
order: 0
wip: 0
---
{% include var.md %}

# General guide to Luos technology

Luos is a simple and lightweight [distributed operating system](https://en.wikipedia.org/wiki/Distributed_operating_system) dedicated to embedded systems. It's a powerful tool using modularity to simplify and link any component or application code together as a [single system image](https://en.wikipedia.org/wiki/Single_system_image).
This guide contains all the basic notions you will need to use, create and understand Luos technology.

Luos is a low-level software technology uploaded into every [**nodes**](#node) of a device. It's composed of two main parts: 
 * A **communication bus protocol** part.
 * An **API** part.
 
Luos is composed as well of **code subdivisions** called [**modules**](#module). Modules are distributed into the nodes.
 
Luos allows a real-time communication between every electronic part of a device. The technology implements a **plug-and-play network** into any robotic device.

## <a name="node"></a>What is a node
A node is a physical component (hardware) running Luos and hosting one or several modules. In a Luos network, nodes are all connected together using Robus, the Luos communication technology.<br/>In other words, **a node is a microcontroler** connected to other microcontrolers and running Luos.

A **Luos prototyping board** hosts a single node. Each one provides a robotic function (gate, sensor, actuation, etc.).

In the Luos philosophy, each node has to carry the necessary programs (modules) allowing to manage the boards and devices hosting it.

It is possible to have multiple nodes in the same electronic board, but each node will be seen by Luos as a different node and has to manage separate devices, whatever the number of boards.

## <a name="module"></a>What is a module
A module is a block of code which is able to communicate with any other modules in the Luos network. Each module provides a particular set of tasks such as managing a motor, handling a laser range finder, or compute an inverse-kinematics, for example.
**Each module is hosted in a single node**, but a node can handle several modules at the same time and manage communication between them and between other modules hosted in other nodes, using the same interface.

For example, the [Dynamixel board](/board/dxl) provided by Luos Robotics can dynamically create and manage Dynamixel modules depending on the number of Dynamixel motors linked to it. Any Dynamixel module can get or set values to other Dynamixel modules on the same node or to any other module in any other node of the network.

![feature-module-node-board](/assets/img/feature-module-node-board.jpg)

## Module basic properties
To properly work, each module owns some properties allowing to other modules to recognize and access it:

{{ table_prop_module }}

## Node basic capacities
Nodes can have capacities such as measuring the core temperature, sending the processor unique ID, or input voltage. Nodes capacities are commonly shared with all modules hosted in. So any module hosted in a node is able to send its input voltage.

## Route table
A route table is a "service" managed by the Luos network and available for any module in any node. This service lists all the modules and allow to any module to get and use basic information of any other modules. The routing table's data can be loaded or auto-generated during detection.

## Module detection
The module detection assigns IDs to modules depending on their node's physical position on the cabling, and generates a route table.

IDs are assigned from the nearest to the furthest node branch by branch, from the point of view of the module running the detection. Following this logic, the module running the detection will have the ID 1, the next one will have the ID 2, etc.

Multiple detection by different modules at the same time is not allowed.

It is possible to detect the network frequently to discover included or excluded modules at run time.

## Module types
Modules are code sections which enclosed several features. They are embedded into nodes.
There are two types of modules:

### Drivers
Drivers are modules managing hardware. They are passive modules that retrieve pieces of information from electronic devices.

### Applications (App)
An application or app is a module which only manages software items such a functions.
For example, you can create an app to compute the inverse-kinematic of a robotic arm. In this case, you can send an arm target position to this app so that it would compute and send orders to each motor modules (drivers) it handles, in order to reach the target.
Apps can be placed in any nodes on your network without any hardware or code modifications, but the choice of the node can impact global performances of the system.
