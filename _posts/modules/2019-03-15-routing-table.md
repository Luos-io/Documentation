---
layout: post
title: "Routing table"
categories: 2_modules
desc: How the routing table works.
order: 4
wip: 1
---
{% include var.md %}

<div class="wip_img"></div>
<blockquote class="warning"><strong>Work in progress</strong><br /><br />We are working on this page...</blockquote><br />

# Routing Table

The routing table is a library which contains a data table listing every <span class="tooltip">[nodes](/../first_steps/general-use#node)<span class="tooltiptext">{{ node_def }}</span></span> and <span class="tooltip">[modules](/../first_steps/general-use#module)<span class="tooltiptext">{{ module_def }}</span></span> connected to a Luos Network. It also provides tools to search or manage modules and nodes.

## Modes

The routing table has two modes: *module entry mode* and *node entry mode*.

The structure of *module entry mode* contains the id, type and alias of each module.
The structure of *node entry mode* contains the UUID and a node link table which shows the number of connectors.

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

The table can be displayed with both mode in Jupyter Notebook after connecting to a Luos network through an [USB gate](/../modules_list/gate) on a computer with [Pyluos](/../first_steps/pyluos). 

### Calling *module entry mode*
The *module entry mode* displays a list of all the modules into the Luos network, and their associated characteristics (type, alias and ID).
To call this mode, type the following line in Jupyter Notebook:
```python
robot.modules
```

> **Note:** `robot` is the name of the network. 

The routing table is displayed as in the following example:
```
-------------------------------------------------
Type                Alias               ID   
-------------------------------------------------
Gate                gate                1    
Voltage             analog_read_P1      2    
Voltage             analog_read_P7      3    
Voltage             analog_read_P8      4    
Voltage             analog_read_P9      5    
State               digit_read_P5       6    
State               digit_read_P6       7    
State               digit_write_P2      8    
State               digit_write_P3      9    
State               digit_write_P4      10   
ControlledMotor     controlled_moto     11   
Imu                 Imu                 12   
Color               bat_lvl             13   
Angle               potentiometer_m     14   
```

In this example, 14 modules are listed, but the nodes (where they physically are in the network) are not displayed.

### Calling *node entry mode*
The *node entry mode* displays the various nodes of the Luos network, and the modules contained in each of them.
To call this mode, type the following line in Jupyter Notebook:
```python
robot.nodes
```

> **Note:** `robot` is the name of the network. 

The routing table is displayed as in the following example:

```
 root : [4653093, 1194612501, 540554032]
        |  Type                Alias               ID   
        └> Gate                gate                1    
└── 1<=>0 : [4653107, 1347571976, 540555569]
            |  Type                Alias               ID   
            └> Angle               potentiometer_m     2    
    └── 1<=>0 : [2687014, 1194612503, 540554032]
                |  Type                Alias               ID   
                └> Color               bat_lvl             3    
        └── 1<=>0 : [3801124, 1498566923, 540621113]
                    |  Type                Alias               ID   
                    └> Imu                 Imu                 4    
            └── 1<=>0 : [4259877, 1194612501, 540554032]
                        |  Type                Alias               ID   
                        └> ControlledMotor     controlled_moto     5    
                └── 1<=>0 : [4456498, 1347571976, 540555569]
                            |  Type                Alias               ID   
                            └> Voltage             analog_read_P1      6    
                            └> Voltage             analog_read_P7      7    
                            └> Voltage             analog_read_P8      8    
                            └> Voltage             analog_read_P9      9    
                            └> State               digit_read_P5       10   
                            └> State               digit_read_P6       11   
                            └> State               digit_write_P2      12   
                            └> State               digit_write_P3      13   
                            └> State               digit_write_P4      14   
                    └── 1<=>1 : [1638451, 1430802710, 540227889]
                                |  Type                Alias               ID   
                                └> DCMotor             DC_motor1_mod       15   
                                └> DCMotor             DC_motor2_mod       16
```
In this example, 7 nodes (MCU) and their associated UUID are listed, along with their modules and associated characteristics (type, alias and ID). 
The characters after each set of node's modules and before the UUID's next node specify which connector is used. For example, `1<=>0` means the first node is connected from it's second connector (1) to the first connector (0) of the next node.


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

