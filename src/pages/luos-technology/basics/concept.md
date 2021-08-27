# Concept

## What is a Node?
A node is a physical component (hardware) running Luos and hosting one or several services. In a Luos network, nodes are all connected together using <span class="cust_tooltip">Robus<span class="cust_tooltiptext">{{robus_def}}</span></span>, the Luos network manager compatible with most of the existing communication bus.<br/>In other words, **a node is a microcontroller** connected to other microcontrollers running Luos.
In the Luos philosophy, each node has to carry the necessary programs (services), allowing it to manage its boards and features.

<img src="../../../_assets/img/MCU-luos.png" height="100px" />


## Package
At Luos, a package is a folder of code containing a pack of features in your project. A package can be composed of one or several services, but services on the same package share the same physical resource. For example a dual DCmotor driver is developed into one package but expose 2 services (1 for each motor). The purpose of this package is to be simply copy pasted across projects or shared as ready to use apps.

## Service
A service is a "public" feature hosted on a package. It provide the API of your feature by exposing inputs and outputs. An input could be an angular target position of a motor, the color of an RGBled, or an advanced custom command to control the behavior of your product. An output could be a temperature, the torque delivered by a motor, or the result of a complex home-made algorithm.

Services, much like their server-world counterparts, can be placed anywhere in your infrastructure, and you do not need to know where they are located to access it. Luos will detect the physical position of all the nodes of your product list all the services among them and share the result with all your services in a routing table. The network can physically change, Luos can update dynamically the routing table allowing your services to still available.

**Each service is hosted in a single node**, but you can have the same service duplicated multiple time on your product.

For example, our Dynamixel example (available on our [github example repository](https://github.com/Luos-io/Examples)) can dynamically create servomotors services depending on the number of Dynamixel motors linked to it. Any Dynamixel services are visible as independent servomotors similar to any other servomotor technology such as our stepper service example, or our controller motor service example. Then, you can use any of your Dinamixel from any other services or even from any computer or cloud program or ecosystem such as ROS.

There is 2 categories of services, [drivers](../services/service_api.html#drivers-guidelines) or [applications](../services/service_api.html#apps-guidelines).

 - **Drivers** are services giving advanced access to a physical resource. Drivers can't rely on any other services, they are independent. Drivers should comply to the adapted service profile provided by Luos and the community allowing an universal access to any physical resource.
 - **Applications** are the behavior of your product and don't rely on any hardware. Applications services search for driver services they need and use them to physically control the device.

 Following the rules of those categories will help you to improve the maintainability and the re-usability of all your developments.

[Go to the Services page](../services/services.md) for more information.

## Service detection
Services on the network are automatically detected and assigned IDs depending on their node's physical position in the network, and a routing table is generated.

IDs are assigned from the nearest to the furthest node branch by branch, from the point of view of the service running the detection. Following this logic, the service running the detection will have ID 1, the next one will have ID 2, etc.

> *Note:* Multiple detections by different services at the same time is not allowed.

It is possible to execute a detection on the network frequently in order to dynamically discover included or excluded services while running, detecting if hardware has been added or removed. Read the [Routing table](#routing-table) section for more information.

## Routing table
A routing table is a data structure managed by the Luos network and available for any services on any nodes. This data structure lists all the services on the network and allows any services to access and use basic information of any other services or node. The routing table's data can be loaded or auto-generated during detection, and can be refreshed on demand.

[Go to the Routing table page](../services/routing_table.md).

## Messages
Communication between services and apps is performed through Messages. A message contains information on the destination service(s), the type of operation to be performed (a read or write operation and the type of message), as well as any supplemental data. The message will be sent on the network and will arrive at the destination, no matter where the service is placed on the network.

[Go to the Messages handling page](../message/handling-message.md).

