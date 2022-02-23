---
custom_edit_url: null
---

# Node

A Luos node is the term that corresponds to each hardware component connected to a Luos network. In other words, each MCU connected into the system, is called a node. The node contains a package and one or more services, concepts that are going to be analyzed in the following pages.

The nodes are described by a set of characteristics that are unique and give them the opportunity to be separated the one from the other. These are:

- **node_id**: node's unique id
- **certified**: true if the node is certified
- **port_table**: physical port connections

Each node hosts the embedded Luos API, which permits its communication with all the nodes of the network, by using the Luos communication protocol, called [Robus], and specific message handling mechanisms, the Luos HAL ([Hardware Abstraction Layer](./luos-hal)) for the mcu configuration and the embedded code of the different functionalities ([services](../services)) of the specific node.
