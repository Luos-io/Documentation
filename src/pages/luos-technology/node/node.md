# Node

A Luos node is an hardware component connected to a Luos network. In other words, each MCU connected to the system is called a node. A node contains at least one package with one or more <span class="cust_tooltip">[**services**](#service)<span class="cust_tooltiptext">{{service_def}}</span></span>.

The nodes are described by a set of unique characteristics:

 - **node_id**: node's unique id.
 - **port_table**: physical port connections.


Each node hosts the embedded Luos API, which allows services to communicate with others on the network. Nodes have specific network hardware access that have to be defined in the [Luos's Hardware Abstraction Layer (HAL)](./luos-hal.md). They can [host, manage, and run services](./luos.md).
