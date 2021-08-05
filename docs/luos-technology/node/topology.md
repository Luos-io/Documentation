# Network topology

As mentioned in the previous paragraphs, thanks to [Robus](../node/luos.md), the nodes have the capabilty to communicate with each other. However, Robus is not the only necessity to achieve using this characteristic. The nodes can have at any time, knowledge about the position of every other node in the network, by accessing a specific structure, called Routing Table. The routing table, is constructed and shared among the nodes, after the execution of a devoted process called detection.

## Routing Table

The routing table is a feature of Luos allowing every <span className="cust_tooltip">[node](./node.md)<span className="cust_tooltiptext">{{ node_def }}</span></span> to own a "map" (or topology) of the entire network of your device. This map allows nodes to know their physical position, and their different functionalities, as well as to search and interact with the other nodes easily.<br/>

## Detection

The routing table is automatically generated when a network detection is initiated by a node. It is then shared with other services at the end of the detection. A detection can be initiated by any service, but driver services should not be able to run it; this kind of features should be only used with app services by including routingTable.h and using this routing table API.

A non-detected node (not in the routing table) has a specific ID of `0`. At the beginning of the detection, Luos erases each node's ID in the network, so all of them will have the ID `0` during this operation.

Then, when the detection is launched by a specific node, IDs are added sequentially to the different nodes, depending on their position from the detector that started the detection process. The ID attribution begins first to the PTPA port, then PTPB, etc.
When each node in the network has an attributed ID, the detection algorithm proceeds to the creation of the routing table and shares it with all the nodes.

> **Warning:** Pay attention to the fact that during a detection, a node can change ID depending on the node ID that initialized the detection process. Do not consider your ID as fixed. Also, be aware that the nodes remove their auto-update configuration during the detection, to prevent any ID movement.

## Modes

As explained in this [page](../basics/basics.md), <span className="cust_tooltip">nodes<span className="cust_tooltiptext">{{ node_def }}</span></span> can host multiple services. To get the topology of your device, the routing table references physical connections between your nodes and lists all the services in each one of them.

The routing table is a table of a `routing_table_t` structure containing nodes or services information.
The maximum number of services and nodes are managed by the precompilation constant `MAX_SERVICES_NUMBER` (set to 40 by default).

```c
routing_table_t routing_table[MAX_SERVICES_NUMBER];
```

The routing table structure has two modes: _service entry mode_ and _node entry mode_.

```c
typedef struct __attribute__((__packed__))
{
    entry_mode_t mode;
    union
    {
        struct __attribute__((__packed__))// SERVICE mode entry
        {
            uint16_t id;                // Service ID.
            uint16_t type;              // Service type.
            char alias[MAX_ALIAS_SIZE]; // Service alias.
        };
        struct __attribute__((__packed__))// NODE mode entry
        {
            // Watch out, this structure has a lot of similarities with the node_t struct.
            // It is similar to allow copy of a node_t struct directly in this one
            // but there is potentially a port_table size difference so
            // do not replace it with node_t struct.
            struct __attribute__((__packed__))
            {
                uint16_t node_id : 12;  // Node id
                uint16_t certified : 4; // True if the node have a certificate
            };
            uint16_t port_table[(MAX_ALIAS_SIZE + 2 + 2 - 2) / 2]; // Node link table
        };
        uint8_t unmap_data[MAX_ALIAS_SIZE + 2 + 2];
    };
} routing_table_t;
```

### Node entry mode

This mode gives physical information of your devices.

The **node_id** is the unique number that you can use to identify each one of your nodes. At the beginning (or when a reset detection is perfomed), all node IDs are set to 0. When the RoutingTB_DetectServices API is called, Luos assigns a unique ID to nodes and services in your system topology.

The **certified** Luos node can be certified for your system by including Luos licencing number in your product (feature in progress).

The **port_table** allows sharing of topological information of your network. Each element of this table corresponds to a physical Luos port of the node and indicates which node is connected to it by sharing a node's `id`.

Here is an example:

<img src="/img/routing-table.png" title="Routing table"/>

As shown on this image, elements of the `port_table` indicate the first or last service id of the connected node through a given port.

Specific values taken by `port_table`:

- **0**: this port is waiting to discover who is connected with. You should never see this value.
- **0x0FFF**: this port is not connected to any other Node.

> **Note:** Routing tables can be easily displayed using [Pyluos](/tools/pyluos.md) through a USB [gate](/tools/gate.md). Please refer to the [Pyluos routing table section](/tools/pyluos.md) for more information.

### Service entry mode

Except from the node entry mode, there is a second mode, the service entry mode, that allows the `routing_table` to contain information about the smallest entity contained in a node, which is called a service. As a node can host one or more services, the routing table is able to obtain the specific information of each one:

- id: service's unique id
- type: service's type
- alias: service's alias

More information about what the services are and how they are used, are given in the following [pages](../services/services.md).
