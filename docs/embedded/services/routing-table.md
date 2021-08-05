# Routing Table

> **Warning:** Make sure to read and understand how to [Create Luos services](./create-services.md) before reading this page.

The routing table is a feature of Luos allowing every <span className="cust_tooltip">[service]<span className="cust_tooltiptext">{{ service_def }}</span></span> to own a "map" (or topology) of the entire network of your device. This map allows services to know their physical position and to search and interact with other services easily.<br/>
This feature is particularly used by apps services to find other services they need to interact with. The routing table is shared by the service which launches the detection to other services, but only apps services store the routing table internaly.

## Detection

The routing table is automatically generated when a network detections is initiated by a service. It is then shared with other services at the end of the detection. A detection can be initiated by any service, but driver services should not be able to run it; this kind of features should be only used with app services by including routingTable.h and using this routing table API.

To run a detection, type:

```C
RoutingTB_DetectServices(app);
```

where app is the `service_t` pointer running the detection.

A non-detected service (not in the routing table) has a specific ID of `0`. At the beginning of the detection, Luos erases each service's ID in the network, so all of them will have the ID `0` during this operation. You can use it on your services code to act consequently to this detection if you need it (for example, a service can monitor its ID to detect if a detection has been made and if it has to reconfigure its auto-update).

Then the service running the detection will have the ID `1` and the other services will have an ID between `2` and `4096`, depending on their position from the service detector. The IDs are attributed to the services according to their position from the detector service and to the branch they are in. The ID attribution begins first to the PTPA port, then PTPB, etc.
When each service in the network has an attributed ID, the detection algorithm proceeds to the creation of the routing table and shares it with every services (saved only one time per node).

Sometimes, multiple services in the network can have the same alias, which is not allowed to prevent service confusion. In this case, detection algorithm will add a number after each instance of this alias on the routing table.

> **Warning:** Be careful that during a detection, a service can change ID depending on the service running this detection. Do not consider your service's ID fixed. Also, be aware that every services remove their auto-update configuration during the detection to prevent any ID movement.

## Modes

As explained in [this page], <span className="cust_tooltip">nodes<span className="cust_tooltiptext">{{ node_def }}</span></span> can host multiple services. To get the topology of your device, the routing table references physical connexions between your nodes and lists all the services in each one of them.

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

### service entry mode

This mode allows `routing_table` to contain:

- id: service's unique id
- type: service's type
- alias: service's alias

For more information, please refer to the [services](../services.md) page of this documentation.

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

> **Note:** Routing tables can be easily displayed using [Pyluos] through a [USB gate](/software/services_list/gate.md). Please refer to the [Pyluos routing table section] for more information.

## Search tools

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

## Management tools

Here are the management tools provided by the routing table library:

|                 Description                  |                                       Function                                        |       Return       |
| :------------------------------------------: | :-----------------------------------------------------------------------------------: | :----------------: |
|          Compute the rooting table           |                     `RoutingTB_ComputeRoutingTableEntryNB(void);`                     |       `void`       |
|    Detect the services in a Luos network     |                    `RoutingTB_DetectServices(service_t* service);`                    |       `void`       |
|   Convert a node to a routing table entry    |     `RoutingTB_ConvertNodeToRoutingTable(routing_table_t *entry, node_t *node);`      |       `void`       |
|  Convert a service to a routing table entry  | `RoutingTB_ConvertServiceToRoutingTable(routing_table_t* entry, service_t* service);` |       `void`       |
| Remove an entry in the routing table (by id) |                    `RoutingTB_RemoveOnRoutingTable(uint16_t id);`                     |       `void`       |
|             Erase routing table              |                               `RoutingTB_Erase(void);`                                |       `void`       |
|            Get the routing table             |                                `RoutingTB_Get(void);`                                 | `routing_table_t*` |
|    Get the last service in a Luos network    |                           `RoutingTB_GetLastService(void);`                           |     `uint16_t`     |
|     Get the last entry in a Luos network     |                            `RoutingTB_GetLastEntry(void);`                            |     `uint16_t`     |
|     Get the last node in a Luos network      |                            `RoutingTB_GetLastNode(void);`                             |    `uint16_t*`     |
