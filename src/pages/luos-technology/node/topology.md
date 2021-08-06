# Network topology

As mentioned in the previous paragraphs, thanks to [Robus](../node/luos.md), the nodes have the capabilty to communicate with each other. However, Robus is not the only necessity to achieve using this characteristic. The nodes can have at any time, knowledge about the position of every other node in the network, by accessing a specific structure, called Routing Table. The routing table, is constructed and shared among the nodes, after the execution of a devoted process called detection.

## Routing Table

The routing table is a feature of Luos allowing every <span class="cust_tooltip">[node](./node.md)<span class="cust_tooltiptext">{{ node_def }}</span></span> to own a "map" (or topology) of the entire network of your device. This map allows nodes to know their physical position, and their different functionalities, as well as to search and interact with the other nodes easily.<br/> The routing table is constructed and shared among all the nodes, after a process that is called [Detection](../services/routing_table.md).

## PTP
Peer-to-peer connection between nodes for topology detection.

### Daisy-chain
With 2 PTP pins per board, you must chained you device as below:
![](../../../_assets/img/daisy_chain.png)

### Star Mounting
With at least 3 PTP pins per board, you can create a star mounting configuration:
![](../../../_assets/img/star_mounting.png)