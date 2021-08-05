# General integration consideration

## Library

Luos works as a code library running on nodes. To match Luos library with your hardware, Luos offers a _Hardware Abstraction Layer_ for various devices in <span className="cust_tooltip">LuosHAL<span className="cust_tooltiptext">{{luoshal_def}}</span></span>.

- <a href="https://github.com/Luos-io/LuosHAL" target="_blank">LuosHAL</a>: This repository provides a list of family devices covered to match the Luos library with your hardware.
- <a href="https://github.com/Luos-io/Luos" target="_blank">Luos</a>: This is the main library you will be working with.

To make it work in your environment, you have to:

- Include the Luos lib folders in your project compilation;
- Select the right LuosHAL for your device family in LuosHAL folder, and include `luos_hal.c`, `luos_hal.h` and `luos_hal_config.h` in your project;
- Change, if necessary, `luos_hal_config.h` for your project. The default configuration created by Luos is an example for a MCU family that can be modify to fit with your design (eg: match pins with your design);
- Include `luos.h` on your source file.

## Configuration

Luos allow you to configure some parameters allowing to optimize the memory usage to your needs. To make it we advise to use a configuration file call node_config.h. Put the file at the root folder of your project and add it in the compiling variables section of your IDE using a _-include node_config.h_.

You can use it to set all your custom configuration:

- for the services of your node
- for Luos library of your node
- to modify Luos HAL config to make it fit with your design

|     Parameters     |    Defaults value     |                                                Description                                                 |
| :----------------: | :-------------------: | :--------------------------------------------------------------------------------------------------------: |
|   NBR_NAK_RETRY    |          10           |                              Number of retries to send after a received NAK.                               |
| MAX_SERVICE_NUMBER |           5           |                           Number of services in the node (memory optimisation).                            |
|  MSG_BUFFER_SIZE   |      3\*size_msg      |     Message buffer size. Max size of a message (3 \* (7 bytes header + 128 bytes data + 2 bytes CRC)).     |
|     MAX_MSG_NB     | 2\*MAX_SERVICE_NUMBER |                        Max number of messages for a service that can be referenced.                        |
|      NBR_PORT      |           2           | Number of PTP on the node ( max 8). See [electronic design](../hardware_topics/electronic-design.md) page. |

You will find the default configuration for Luos Library in the file <a href="https://github.com/Luos-io/Luos/tree/master/Robus/inc/config.h" target="_blank">config.h</a>,

Check the Luos_hal_config.h of your MCU family to see parameter that can be change to fit your design.

> **FYI:** [Every examples](https://github.com/Luos-io/Examples) provided by luos have a node_config.h files that can be use as base to fit your project needs.
