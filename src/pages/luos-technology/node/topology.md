# Network topology

As mentioned in the previous paragraphs, thanks to [Robus](../node/luos.md), the nodes have the capability to communicate with each other. However, Robus is not the only necessity to achieve using this characteristic. The nodes can access at any time to the position of every other node in the network by accessing a specific structure called routing table. The routing table is designed and shared among the nodes after the execution of a devoted process called detection.

## Routing Table

The routing table is a feature of Luos allowing every <span class="cust_tooltip">[node](./node.md)<span class="cust_tooltiptext">{{ node_def }}</span></span> to own a "map" (or topology) of the entire network of your device. This map allows nodes to know their physical position and different functionalities, as well as to easily search and interact with the other nodes.

The routing table is designed and shared among all the nodes after a process that is called [detection](../services/routing_table.md).

## PTP
PTP is the peer-to-peer connection between nodes used for topology detection.

### Daisy-chain
With 2 PTP pins per board, you must chained your device as below:
![](../../../_assets/img/daisy_chain.png)

### Star Mounting
With at least 3 PTP pins per board, you can create a star mounting configuration:
![](../../../_assets/img/star_mounting.png)
