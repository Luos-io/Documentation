# Node

A Luos node is the term that corresponds to each hardware component connected to a Luos network. In other words, each MCU connected to the system is called a node. A node contains at least one package with one or more <span class="cust_tooltip">[**services**](#service)<span class="cust_tooltiptext">{{service_def}}</span></span>.

The nodes are described by a set of unique characteristics that have the opportunity to be separated from the others. These characteristics are:

 - **node_id**: node's unique id.
 - **certified**: true if the node is certified.
 - **port_table**: physical port connections.


Each node hosts the embedded Luos API, which allows to communicate with all the nodes of the network. The Luos communication protocol is called [Robus](./luos.html#robus). Nodes also use specific message-handling mechanisms, such as the Luos HAL ([Hardware Abstraction Layer](./luos-hal.md)) for the MCU configuration, or the embedded code of the specific node's functionalities ([services](../services/services.md)).