---
custom_edit_url: null
---

import { customFields } from "/docusaurus.config.js";
import Tooltip from "/src/components/Tooltip.js";
import Image from '/src/components/Images.js';

# Concept

## What is a Node?

A node is a physical component (hardware) running Luos and hosting one or several services. In a Luos network, nodes are all connected together using <Tooltip def={customFields.robus_def}>Robus</Tooltip>, the Luos network manager compatible with most of the existing communication bus.

In other words, **a node is a microcontroller** connected to other microcontrollers running Luos.
In the Luos philosophy, each node has to carry the necessary programs (services), allowing it to manage its boards and features.

<div align="center">
  <Image src="/img/MCU-luos.svg" darkSrc="/img/MCU-luos-white.svg" height="200px"/>
</div>

## Package

A package is a folder of code containing a pack of features in your project. A package can be composed of one or several services, but services in the same package share the same physical resources. For example, a dual DCmotor driver is developed into one package but exposes two services (one for each motor). The purpose of this package is to be simply copied and pasted across projects or shared as ready-to-use apps.

## Service

A service is a "public" feature hosted in a package. It provides the API of your feature by exposing inputs and outputs. An input can be an angular target position of a motor, the color of an RGB LED, or an advanced custom command to control the behavior of your product. An output can be a temperature, the torque delivered by a motor, or the result of a complex home-made algorithm.

Services, much like their server-world counterparts, can be placed anywhere in your infrastructure, and you as a developer do not need to know where they are located to access them. Luos will detect the physical position of all the nodes of your product, list all the services among them, and share the result with all your services in a routing table. The network can physically change, Luos can update dynamically the routing table allowing your services to stay available.

**Each service is hosted in a single node**, but you can have the same service duplicated multiple times in your product.

For example, the Dynamixel example (available in the <a href="https://github.com/Luos-io/luos_engine/tree/main/examples" target ="_blank" rel="external nofollow">Luos engine example folder</a>) can dynamically create servomotors services depending on the number of Dynamixel motors linked to it. Any Dynamixel services are visible as independent servomotors similar to any other servomotor technology such as the stepper service example, or the controller motor service example. Then, you can use any of your Dynamixel from any other services or even from any computer, cloud program, or ecosystem such as ROS.

There are two categories of services, [drivers](../services/service-api#drivers-guidelines) or [applications](../services/service-api#apps-guidelines).

- **Drivers** are services giving advanced access to a physical resource. Drivers cannot rely on any other services; they are independent. Drivers should comply to the adapted service profile provided by Luos and the community, allowing an universal access to any physical resource.
- **Applications** are the behaviors of your product and don't rely on any hardware. Application services search for the driver services they need and use them to physically control the device.

Following the rules of these categories will help you to improve the maintainability and the re-usability of all your developments.

[Go to the Services page](../services) for more information.

## Service detection

Services in the network are automatically detected and being assigned IDs depending on their node's physical position in the network, and a routing table is generated.

IDs are assigned from the nearest to the furthest node branch by branch, from the point of view of the service running the detection. Following this logic, the service running the detection will have ID 1, the next one will have ID 2, etc.

> _Note:_ Multiple detections by different services at the same time is not allowed.

It is possible to execute a detection in the network frequently in order to dynamically discover included or excluded services while running. This allows to detect if hardware has been added or removed. Read the [Routing table](#routing-table) section for more information.

## Routing table

A routing table is a data structure managed by the Luos network and available for any services on any nodes. This data structure lists all the services in the network and allows any services to access and use basic information of any other services or nodes. The routing table's data can be loaded or auto-generated during detection, and can be refreshed on demand.

[Go to the Routing table page](../services/routing-table) for more information.

## Messages

Communication between services and apps is performed through messages. A message contains information on the destination service(s), the type of operation to be performed (a read or write operation and the type of message), as well as any additional data. The message will be sent in the network and will arrive at the destination, no matter where the service is placed in the network.

[Go to the Messages handling page](../message/handling-message) for more information.
