# Network topology

The nodes can access the position of every other node in the network at any time, by accessing a specific structure called routing table. The routing table is designed and shared among the nodes after the execution of a devoted process called detection.

This routing table allows any service to find, locate, and use any other service on the entire network.

## Routing Table

The routing table is a feature of Luos allowing every <span class="cust_tooltip">[node](./node.md)<span class="cust_tooltiptext">{{ node_def }}</span></span> to own a "map" (or topology) of the entire network of your device. This map allows nodes to know their physical position and different functionalities, as well as to easily search and interact with the other nodes.

The routing table is designed and shared among all the nodes after a process that is called [detection](/docs/luos-technology/services/routing-table).

## PTP

PTP is the point-to-point connection between nodes used for topology detection. Every node should have between 2 to 8 PTP connections representing ports. At this time, these wires are mandatory.

### Daisy-chain

With two PTP pins per board, you must chained your device as below:

<p align="center">
	<img src="/img/daisy_chain.png" />
</p>

### Star Mounting

With at least three PTP pins per board, you can create a star mounting configuration:

<p align="center">
	<img src="/img/star_mounting.png" />
</p>
