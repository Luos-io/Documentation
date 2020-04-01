> **Warning:** Make sure to read and understand how to [Create Luos modules](/_pages/low/modules/create-modules.md) before reading this page.

# Routing Table

The routing table is a feature of Luos allowing every modules to have a "map" of the entire network  of your device. This map allow modules to know their physical position and to search and interact with other modules easily.<br/>
This feature is particularly used by apps modules to find others modules they need to interact with.

## Modes
As you know <span class="cust_tooltip">[nodes](/_pages/overview/general-basics.md#node)<span class="cust_tooltiptext">{{ node_def }}</span></span> can host multiple <span class="cust_tooltip">[modules](/_pages/overview/general-basics.md#module)<span class="cust_tooltiptext">{{ module_def }}</span></span>. To get topology of your device the routing table reference physical connexions between your <span class="cust_tooltip">[nodes](/_pages/overview/general-basics.md#node)<span class="cust_tooltiptext">{{ node_def }}</span></span> and list all <span class="cust_tooltip">[modules](/_pages/overview/general-basics.md#module)<span class="cust_tooltiptext">{{ module_def }}</span></span> in each of them.

Basically the routing table is a table of routing_table_t structure containing nodes or modules informations.
Tha maximum number of modules and node are managed by `MAX_MODULES_NUMBER` precompilation constant (set to 40 by default).
```C
route_table_t route_table[MAX_MODULES_NUMBER];
```

The routing table structure have two modes: *module entry mode* and *node entry mode*.

```c
typedef struct __attribute__((__packed__)){
    entry_mode_t mode;
    union {
        struct __attribute__((__packed__)){ // MODULE entry mode
            unsigned short id; // Module ID
            unsigned char type; /*!< Module type. */
            char alias[MAX_ALIAS_SIZE]; /*!< Module alias. */
        };
        struct __attribute__((__packed__)){ // NODE entry mode
            luos_uuid_t uuid; // Node UUID
            unsigned short port_table[4]; // Node link table
        };
    };
}routing_table_t;
```

### Module entry mode
This mode allow route_table to contain :
 - ID : module unic Id
 - type : module type
 - alias : module alias

For more informations please refer to the [Modules](/_pages/low/modules.md) section of this documentation.

### Node entry mode
This mode give physical informations of your devices.

The **uuid** is the serial number of the microcontroler hosting Luos. This number is unic, you can use it to identify all your nodes.

The **port_table** allow to share topological informations of your network. Each element of this table correspond to a physical Luos port of the node and indicate which node is connected to it by sharing a module ID.

let's take an example :

<img src="{{img_path}}/routing-table.png" title="Route table">

As you can see here elements of the port_table indicate the first or last module id of the Node connected trough a given port.

Specific values of port_table :

 - **0** : this port is waiting to discover who is connected with. You should never see one.
 - **0xFF** : this port is not connected to any other Node.

> **Info:** Route tables can be easily displayed using [Pyluos](/_pages/high/pyluos.md) through a [USB gate](/_pages/high/modules_list/gate.md). Please refer to the [Pyluos routing table section](/_pages/high/pyluos.md#routing-table-display)

## Search tools
The routing table library provides the following search tools to find modules and nodes' information into a Luos network:

| Description | Function | Return |
| :---: | :---: | :---: |
| Find a module's ID from its alias | `id_from_alias(char* alias);` | `int` |
| Find a module's ID from its type (return the first of the list) | `id_from_type(module_type_t type);` | `int` |
| Find a module's string from its type (return the first of the list) | `string_from_type(module_type_t type);` | `char*` |
| Find a module's alias from its ID (return the first of the list) | `alias_from_id(uint16_t id);` | `char*` |
| Find a module's type from its ID | `type_from_id(uint16_t id);` | `module_type_t` |
| Find a module's type from its alias | `type_from_alias(char* alias);` | `module_type_t` |
| Test if a module's type is a sensor | `is_sensor(module_type_t type);` | `uint8_t` |
| Get a node's ID | `get_node_id(unsigned short index);` | `int` |
| Get the number of nodes in a Luos network | `get_node_nb(void);` | `int` |
| Get the list of all the nodes in a Luos network | `get_node_list(unsigned short* list);` | `void` |

## Management tools
Here are the management tools provided by the routing table library:

| Description | Function | Return |
| :---: | :---: | :---: |
| Compute the rooting table | `compute_route_table_entry_nb(void);` | `void` |
| Detect the modules in a Luos network | `detect_modules(module_t* module);` | `void` |
| Convert a node to a routing table entry | `convert_board_to_route_table(route_table_t* entry, luos_uuid_t uuid, unsigned short* port_table, int branch_nb);` | `void` |
| Convert a module to a routing table entry | `convert_module_to_route_table(route_table_t* entry, module_t* module);` | `void` |
| Insert an entry into the routing table | `insert_on_route_table(route_table_t* entry);` | `void` |
| Remove an entry in the routing table (by ID) | `remove_on_route_table(int id);` | `void` |
| Erase routing table | `flush_route_table(void);` | `void` |
| Get the routing table | `get_route_table(void);` | `route_table_t*` |
| Get the last module in a Luos network | `get_last_module(void);` | `int` |
| Get the last entry in a Luos network | `get_last_entry(void);` | `int` |

<div class="cust_edit_page"><a href="https://{{gh_path}}/_pages/low/modules/routing-table.md">Edit this page</a></div>
