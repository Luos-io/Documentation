---
custom_edit_url: null
---

import { customFields } from "/docusaurus.config.js";
import Tooltip from "/src/components/Tooltip.js";
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
import Image from '/src/components/Images.js';

# Routing Table

:::caution
Make sure you have read and understood the [network topoly section](../node/topology) before reading this page.
:::

The routing table is a feature of Luos allowing every <Tooltip def={customFields.service_def}>services</Tooltip> to own a "map" (or topology) of the entire network of your device. This map enables services to know their physical position and to search and interact with other services quickly.

This feature is particularly used by app services to find other services they need to interact with. The routing table is shared by the service that launches the detection to other services.

## Detection

The routing table is automatically generated when a service demands Luos to initiate a network detection. It is then shared with other services at the end of the detection. Any service can demand a detection, but driver services should not do it; this features should be only used with app services by including routingTable.h and using this routing table API.

To run a detection, type:

```c
Luos_Detect(app);
```

where `app` is the `service_t` pointer running the detection.

A non-detected service (not in the routing table) has a specific ID of `0`. At the beginning of the detection, Luos erases each service's ID in the network, so all of them will have ID `0` during this operation. You can use it on your services code to act consequently to this detection if you need it (for example, a service can monitor its ID to detect if a detection has been made and if it has to reconfigure its auto-update).

Then the service running the detection will have ID `1`, and the other services will have an ID between `2` and `4096`, depending on their position from the service detector. IDs are attributed to the services according to their position from the detector service, and to the branch they are in. ID attribution begins first to the PTPA port, then PTPB, etc.
When each service in the network has an attributed ID, the detection algorithm proceeds to create the routing table and shares it with every service (saved only one time per node).

Sometimes, multiple services in the network can have the same alias, which is not allowed to prevent service confusion. In this case, the detection algorithm will add a number after each alias instance on the routing table.

At the end of the detection process, each service receives a message with the command of `END_DETECTION`, that can be used in your code in order to understand the exact moment that the detection has finished, and you can then reinitialize the behavior of your services (for example reinitialize the auto-updates, etc,). Also, you can check the detection status of your node, by using the dedicated Luos API:

```c
Luos_IsNodeDetected();
```

that returns `true` or `false` depending on whether this node is detected or not.

:::caution Warnings

1. Be careful that a service can change ID during a detection depending on the service running this detection.
2. Do not consider your service's ID fixed.
3. Be aware that every service removes its auto-update configuration during the detection to prevent any ID movement.
4. Make sure that by the creation of your services you specify a callback pointer for each of them, so that the messages arriving to them, concerning the detection do not stay stored in your node's memory.
   :::

## Modes

Nodes can host multiple services. To get the topology of your device, the routing table references physical connections between the nodes and lists all the services in each one of them.

The routing table is a table of a `routing_table_t` structure containing nodes or services information.
The precompilation constant MAX_SERVICES_NUMBER manages the maximum number of services (set to 40 by default).

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

### Service entry mode

Service entry mode allows the routing table to include information about a service. As a node can host one or more services, the routing table is able to obtain the specific information for each one of them:

- **id**: service's unique id
- **type**: service's type
- **alias**: service's alias

You can read the [services page](../services) for more information about what services are and how they are used.

### Node entry mode

This mode gives physical information about your devices.

The **node_id** is the unique number that you can use to identify each one of your nodes. At the beginning (or when a reset detection is performed), all node IDs are set to 0. When the RoutingTB_DetectServices API is called, Luos assigns a unique ID to nodes and services in your system topology.

The **certified** Luos node can be certified for your system by including the Luos licencing number in your product (feature in progress).

The **port_table** allows sharing of topological information of your network. Each element of this table corresponds to a physical Luos port of the node and indicates which node is connected to it by sharing a node's `id`.

Here is an example:

<div align="center">
  <Image src="/img/routing-table.svg" darkSrc="/img/routing-table-dark.svg"/>
</div>

As shown on this image, elements of the `port_table` indicate the first or last service id of the connected node through a given port.

Specific values can be taken by `port_table`:

- **0**: this port is waiting to discover which is connected with. You should never see this value.
- **0x0FFF**: this port is not connected to any other node.

:::info
Routing tables can be easily displayed using [Pyluos](../../tools/pyluos) through a [USB gate](../../tools/gate). Please refer to the [Pyluos routing table section](../../tools/pyluos) for more information.
:::

<Tabs className="unique-tabs">
    <TabItem value="Search tools" label="Search tools">
    The routing table library provides the following search tools to find services and nodes' information into a Luos network:

|                             Description                              |                     Function                      |      Return      |
| :------------------------------------------------------------------: | :-----------------------------------------------: | :--------------: |
|                  Find a service's id from its alias                  |       `RoutingTB_IDFromAlias(char* alias);`       |    `uint16_t`    |
|   Find a service's id from its type (return the first of the list)   |     `RoutingTB_IDFromType(luos_type_t type);`     |    `uint16_t`    |
|                  Find a service's id from a service                  |  `RoutingTB_IDFromService(service_t *service);`   |    `uint16_t`    |
|  Find a service's alias from its id (return the first of the list)   |       `RoutingTB_AliasFromId(uint16_t id);`       |     `char*`      |
|                  Find a service's type from its id                   |       `RoutingTB_TypeFromID(uint16_t id);`        | `service_type_t` |
|                 Find a service's type from its alias                 |      `RoutingTB_TypeFromAlias(char* alias);`      | `service_type_t` |
| Find a service's string from its type (return the first of the list) |   `RoutingTB_StringFromType(luos_type_t type);`   |     `char*`      |
|                 Test if a service's type is a sensor                 | `RoutingTB_ServiceIsSensor(service_type_t type);` |    `uint8_t`     |
|              Get the number of nodes in a Luos network               |           `RoutingTB_GetNodeNB(void);`            |    `uint16_t`    |
|                           Get a node's id                            |   `RoutingTB_GetNodeID(unsigned short index);`    |    `uint16_t`    |

</TabItem>

<TabItem value="Management tools" label=" Management tools">
Here are the management tools provided by the routing table library:

|                 Description                  |                                       Function                                        |       Return       |
| :------------------------------------------: | :-----------------------------------------------------------------------------------: | :----------------: |
|          Compute the routing table           |                     `RoutingTB_ComputeRoutingTableEntryNB(void);`                     |       `void`       |
|    Detect the services in a Luos network     |                    `RoutingTB_DetectServices(service_t* service);`                    |       `void`       |
|   Convert a node to a routing table entry    |     `RoutingTB_ConvertNodeToRoutingTable(routing_table_t *entry, node_t *node);`      |       `void`       |
|  Convert a service to a routing table entry  | `RoutingTB_ConvertServiceToRoutingTable(routing_table_t* entry, service_t* service);` |       `void`       |
| Remove an entry in the routing table (by id) |                    `RoutingTB_RemoveOnRoutingTable(uint16_t id);`                     |       `void`       |
|             Erase routing table              |                               `RoutingTB_Erase(void);`                                |       `void`       |
|            Get the routing table             |                                `RoutingTB_Get(void);`                                 | `routing_table_t*` |
|    Get the last service in a Luos network    |                           `RoutingTB_GetLastService(void);`                           |     `uint16_t`     |
|     Get the last entry in a Luos network     |                            `RoutingTB_GetLastEntry(void);`                            |     `uint16_t`     |
|     Get the last node in a Luos network      |                            `RoutingTB_GetLastNode(void);`                             |    `uint16_t*`     |

</TabItem>

</Tabs>
