# Routing Table
> **Warning:** Make sure to read and understand how to [Create Luos containers](/pages/low/containers/create-containers.md) before reading this page.

The routing table is a feature of Luos allowing every <span class="cust_tooltip">[containers](/pages/overview/general-basics.md#container)<span class="cust_tooltiptext">{{ container_def }}</span></span> to own a "map" (or topology) of the entire network of your device. This map allows containers to know their physical position and to search and interact with other containers easily.<br/>
This feature is particularly used by apps containers to find other containers they need to interact with. The routing table is shared by the container which launches the detection to other containers, but only apps containers store the routing table internaly.

## Detection
Routing table is automatically generated when a network detections is initiate by a container and shared with other containers at the end of the detection. A detection can be initiated by any container, but driver containers should not be able to run it and this kind of features should be only used on app containers by including routingTable.h an using this routing table API.

To run a detection, type:
```C
RoutingTB_DetectContainers(app);
```
where app is the `container_t` pointer running the detection.

A non-detected container (not in the routing table) has a specific ID of `0`. At the beginning of the detection, Luos erases each container's ID in the network, so all of them will have the ID `0` during this operation. You can use it on your containers code to act consequently to this detection if you need it (for example, a container can monitor its ID to detect if a detection has been made and if it has to reconfigure its auto-update).

Then the container running the detection will have the ID `1` and the other containers will have an ID between `2` and `4096`, depending on their position from the container detector. The IDs are attributed to the containers according to their position from the detector container and to the branch they are in. The ID attribution begins first to the PTPA port, then PTPB, etc.
When each container in the network has an attributed ID, the detection algorithm proceeds to the creation of the routing table and shares it with every containers (saved only one time per node).


Sometimes, multiple containers in the network can have the same alias, which is not allowed to prevent container confusion. In this case, detection algorithm will add a number after each instance of this alias on the routing table.

> **Warning:** Be careful that during a detection, a container can change ID depending on the container running this detection. Do not consider your container's ID fixed. Also, be aware that every containers remove their auto-update configuration during the detection to prevent any ID movement.

## Modes
As explained in [this page](/pages/overview/general-basics.md#what-is-a-node), <span class="cust_tooltip">nodes<span class="cust_tooltiptext">{{ node_def }}</span></span> can host multiple containers. To get the topology of your device, the routing table references physical connexions between your nodes and lists all the containers in each one of them.

The routing table is a table of a `routing_table_t` structure containing nodes or containers information.
The maximum number of containers and nodes are managed by the precompilation constant `MAX_containerS_NUMBER` (set to 40 by default).

```c
routing_table_t routing_table[MAX_CONTAINERS_NUMBER];
```

The routing table structure has two modes: *container entry mode* and *node entry mode*.

```c
typedef struct __attribute__((__packed__)){
    entry_mode_t mode;
    union {
        struct __attribute__((__packed__)){ // container entry mode
            unsigned short id; // container ID
            unsigned char type; /*!< container type. */
            char alias[MAX_ALIAS_SIZE]; /*!< container alias. */
        };
        struct __attribute__((__packed__)){ // NODE entry mode
            luos_uuid_t uuid; // Node UUID
            unsigned short port_table[4]; // Node link table
        };
    };
}routing_table_t;
```

### container entry mode
This mode allows `routing_table` to contain:
 - id: container's unique id
 - type: container's type
 - alias: container's alias

For more information, please refer to the [containers](/pages/low/containers.md) page of this documentation.

### Node entry mode
This mode gives physical information of your devices.

The **uuid** is the serial number of the microcontroler hosting Luos. This number is unique, you can use it to identify each one of your nodes.

The **port_table** allows to share topological information of your network. Each element of this table corresponds to a physical Luos port of the node and indicates which node is connected to it by sharing a container's `id`.

Here is an example:

<img src="{{container}}/routing-table.png" title="Routing table">

As shown on this image, elements of the `port_table` indicate the first or last container id of the connected node through a given port.

Specific values taken by `port_table`:

 - **0**: this port is waiting to discover who is connected with. You should never see this value.
 - **0x0FFF**: this port is not connected to any other Node.

> **Note:** Routing tables can be easily displayed using [Pyluos](/pages/high/pyluos.md) through a [USB gate](/pages/high/containers_list/gate.md). Please refer to the [Pyluos routing table section](/pages/high/pyluos.md#routing-table-display) for more information.

## Search tools
The routing table library provides the following search tools to find containers and nodes' information into a Luos network:

| Description | Function | Return |
| :---: | :---: | :---: |
| Find a container's id from its alias | `RoutingTB_IDFromAlias(char* alias);` | `int` |
| Find a container's id from its type (return the first of the list) | `RoutingTB_IDFromType(luos_type_t type);` | `int` |
| Find a container's string from its type (return the first of the list) | `RoutingTB_StringFromType(luos_type_t type);` | `char*` |
| Find a container's alias from its id (return the first of the list) | `RoutingTB_AliasFromId(uint16_t id);` | `char*` |
| Find a container's type from its id | `RoutingTB_TypeFromID(uint16_t id);` | `container_type_t` |
| Find a container's type from its alias | `RoutingTB_TypeFromAlias(char* alias);` | `container_type_t` |
| Test if a container's type is a sensor | `RoutingTB_ContainerIsSensor(container_type_t type);` | `uint8_t` |
| Get a node's id | `RoutingTB_GetNodeID(unsigned short index);` | `int` |
| Get the number of nodes in a Luos network | `RoutingTB_GetNodeNB(void);` | `int` |
| Get the list of all the nodes in a Luos network | `RoutingTB_GetNodeList(unsigned short* list);` | `void` |

## Management tools
Here are the management tools provided by the routing table library:

| Description | Function | Return |
| :---: | :---: | :---: |
| Compute the rooting table | `RoutingTB_ComputeRoutingTableEntryNB(void);` | `void` |
| Detect the containers in a Luos network | `RoutingTB_DetectContainers(container_t* container);` | `void` |
| Convert a node to a routing table entry | `RoutingTB_ConvertNodeToRoutingTable(routing_table_t* entry, luos_uuid_t uuid, unsigned short* port_table, int branch_nb);` | `void` |
| Convert a container to a routing table entry | `RoutingTB_ConvertContainerToRoutingTable(routing_table_t* entry, container_t* container);` | `void` |
| Insert an entry into the routing table | `RoutingTB_InsertOnRoutingTable(routing_table_t* entry);` | `void` |
| Remove an entry in the routing table (by id) | `RoutingTB_RemoveOnRoutingTable(int id);` | `void` |
| Erase routing table | `RoutingTB_Erase(void);` | `void` |
| Get the routing table | `RoutingTB_Get(void);` | `routing_table_t*` |
| Get the last container in a Luos network | `RoutingTB_GetLastContainer(void);` | `int` |
| Get the last entry in a Luos network | `RoutingTB_GetLastEntry(void);` | `int` |

<div class="cust_edit_page"><a href="https://{{gh_path}}/pages/low/containers/routing-table.md">Edit this page</a></div>
