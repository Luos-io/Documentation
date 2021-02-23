# General guide to Luos technology

Take most embedded systems, and you will find the same architecture. The dictatorship of electronics; one central microcontroller that has direct control over all components. Sensors, actuators, all of them will be controlled by one chip, and ultimately, one program.
Sometimes, a design will have more than one microcontroller, but will ultimately be controlled by one central device. It will be up to the designers to implement an inter-device communication protocol, using any bus available, including instructions and messaging, but also error handling. A significant portion of time is spent on software development.
Other times, hardware constraints will force you to separate your electronics on to several boards, potentially making interconnection troublesome. 

## Introduction to embedded containerized platforms

Luos aims to change the way you develop by containerizing services on your devices. A microcontroller can host a series of services, like data acquisition from sensors to actuators like motors and relays. These services are placed inside **Containers**. Prepare your containers, and deploy them anywhere on a Luos network, and you will be able to communicate directly with your containers, no matter where they are on the network. Containers can be dynamically connected and disconnected, and can be detected by your application.
Imagine an industrial device. We will build a system with multiple sensors to monitor the device. It might contain several temperature sensors that will monitor specific parts of the system, and our system will need to know exactly what they monitor. When we read the temperature of the industrial motor, we need to get the information from the motor, not from the power supply. These sensors are placed into containers, and clearly state which components they monitor.
If the device encounters a problem, we need to notify anyone around of an incident. This could be a flashing light, a siren, or any other way of notifying people. In this case, it doesn't matter where the peripherals are placed, or even how many there are, Luos can notify all containers that identify as an alarm to activate. Alarms can even be added to the Luos network while it is operating, and they will be recognized and used.
Finally, it doesn't matter where on the Luos network sensors or actuators are placed. You can wire the different elements together as you want, each element will be detected, and you will be able to use all of your containers, wherever they are. 

## Introduction to Luos

Luos is a simple and lightweight containerization platform dedicated to embedded systems enabling a <a href="https://en.wikipedia.org/wiki/Microservices" target="_blank">microservices</a> architecture for electronics. It's a powerful tool using modularity to simplify and link any hardware component or application code together as a <a href="https://en.wikipedia.org/wiki/Single_system_image" target="_blank">single system image</a>.

This guide contains all the basic notions you will need to use, create and understand Luos technology.

Luos is a low-level software technology uploaded into every board's (<span class="cust_tooltip">[**node**](#what-is-a-node)<span class="cust_tooltiptext">{{node_def}}</span></span>) of a device.
You can use Luos as a **bare metal** lib or as a driver into your **embedded OS**.

Luos is composed as well of **code subdivisions** called <span class="cust_tooltip">[**containers**](#container)<span class="cust_tooltiptext">{{container_def}}</span></span>. Containers are distributed into every nodes in a network.

<a href="../../_assets/img/feature-container-node-board.jpg" target="_blank"><img src="../../_assets/img/feature-container-node-board.jpg" width="800px" /></a>

## What is a Node?
A node is a physical component (hardware) running Luos and hosting one or several containers. In a Luos network, nodes are all connected together using <span class="cust_tooltip">Robus<span class="cust_tooltiptext">{{robus_def}}</span></span>, the Luos communication technology.<br/>In other words, **a node is a microcontroler** connected to other microcontrolers running Luos.
In the Luos philosophy, each node has to carry the necessary programs (containers) allowing it to manage its boards and features.

<img src="../../_assets/img/MCU-luos.png" height="100px" />

Nodes can have capacities such as measuring the core temperature, sending the processor's unique ID or input voltage. A node's capacities are commonly shared by all the containers hosted into it and are accessible through each of them.

## Container
A container is code hosted on a Node. It can provide both inputs and outputs. An input could be a temperature sensor, a push button, an I2C sensor, or any other input that you would usually use on your microcontroller design. An output could be an I2C device, or a DAC, GPIO line, PWM or timer output, or any output that you are used to using in your design.
Containers, much like their server-world counterparts, can be placed anywhere in your infrastructure. Containers will be placed on the Luos network, and you do not need to know where in the network your container is located, it will be automatically detected on bootup, and accessible. The network can physically change, and your containers will still be available.

**Each container is hosted in a single node**, but a node can handle several containers at the same time and manage communication between them and between other containers hosted in other nodes, using the same interface.

For example, the [Dynamixel board](../demo_boards/boards_list/dxl.md) provided by Luos can dynamically create and manage Dynamixel containers depending on the number of Dynamixel motors linked to it. Any Dynamixel containers can get or set values to other Dynamixel containers on the same node or to any other containers in any other nodes in the network.

[Go to Containers page](../low/containers.md).

## App
An App is a special type of container, one that does not provide any hardware operations, but will contain the intelligence of your product. This is where you will concentrate your code and development time.

While your app does not contain any hardware capabilites (sensors or actuators), an App can talk to your containers to receive information, or to give orders. For example, an App will not have direct access to sensors or to digital outputs, but will be able to communicate with containers that do offer hardware capabilities. 

## Messages
Communications between continers and apps is performed through Messages. A message contains information on the destination container(s), the type of operation to be performed (a read or write operation and the type of message) as well as any supplemental data. The message will be sent on the network, and will arrive at destination, no matter where the container is placed on the network.

[Go to Messages handling page](../low/containers/msg-handling.md).

## Container detection
Containers on the network are automatically detected, and assigned IDs depending on their node's physical position in the network, and a routing table is generated.

IDs are assigned from the nearest to the furthest node branch by branch, from the point of view of the container running the detection. Following this logic, the container running the detection will have the ID 1, the next one will have the ID 2, etc.

> *Note:* Multiple detections by different containers at the same time is not allowed.

It is possible to execute a detection on the network frequently in order to dynamically discover included or excluded containers while running, detecting if hardware has been added or removed. Go to [Routing table](../low/containers/routing-table.md) page for more information.

## Routing table
A routing table is a "service" managed by the Luos network and available for any containers on any nodes. This service lists all the containers on the network and allows any containers to access and use basic information of any other containers. The routing table's data can be loaded or auto-generated during detection, and can be refreshed on demand.

[Go to Routing table page](../low/containers/routing-table.md).

