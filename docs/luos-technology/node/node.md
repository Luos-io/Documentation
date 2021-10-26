---
custom_edit_url: null
---

# Node

A Luos node is an hardware component connected to a Luos network. In other words, each MCU connected to the system is called a node. A node contains at least one package with one or more <span class="cust_tooltip">[**services**](#service)<span class="cust_tooltiptext">{{service_def}}</span></span>.

The nodes are described by a set of unique characteristics:

<<<<<<< HEAD:src/pages/luos-technology/node/node.md
 - **node_id**: node's unique id.
 - **port_table**: physical port connections.


Each node hosts the embedded Luos API, which allows services to communicate with others in the network. Nodes have specific network hardware access that have to be defined in the [Luos's Hardware Abstraction Layer (HAL)](./luos-hal.md). They can [host, manage, and run services](./luos.md).
=======
- **node_id**: node's unique id
- **certified**: true if the node is certified
- **port_table**: physical port connections

Each node hosts the embedded Luos API, which permits its communication with all the nodes of the network, by using the Luos communication protocol, called [Robus], and specific message handling mechanisms, the Luos HAL ([Hardware Abstraction Layer](/luos-technology/node/luos-hal.md)) for the mcu configuration and the embedded code of the different functionalities ([services](/luos-technology/services/services.md)) of the specific node.
>>>>>>> docusaurus:docs/luos-technology/node/node.md
