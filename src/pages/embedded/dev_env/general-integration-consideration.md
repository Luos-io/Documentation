# General integration consideration

## Library
Luos works as a code library running on nodes. To match Luos library with your hardware, Luos offers a *Hardware Abstraction Layer* for various devices in <span class="cust_tooltip">LuosHAL<span class="cust_tooltiptext">{{luoshal_def}}</span></span>.

 - <a href="https://github.com/Luos-io/LuosHAL" target="_blank">LuosHAL</a>: This repository provides a list of family devices covered to match the Luos library with your hardware.
 - <a href="https://github.com/Luos-io/Luos" target="_blank">Luos</a>: This is the main library you will be working with.

To make it work in your environment, you have to:

 - Include the Luos lib folders in your project compilation;
 - Select the right LuosHAL for your device family in LuosHAL folder, and include `luos_hal.c`, `luos_hal.h` and `luos_hal_config.h` in your project;
 - Change, if necessary, `luos_hal_config.h` in you project, the default configuration created by Luos (before including `luos.h`) in order to match LuosHAL with your hardware (eg: match pins with your design);
 - Include `luos.h` on your source file.

## Configuration

Through the configuration file <a href="https://github.com/Luos-io/Luos/tree/master/Robus/inc/config.h" target="_blank">config.h</a>, Luos allows users to customize many parameters needed for the library. These parameters can be set as compiler variables listed above:

| Parameters | Defaults value | Description |
| :---: | :---: | :---: |
| PROTOCOL_REVISION | 0 | Luos protocol version. |
| DEFAULTBAUDRATE | 1Mbps | Default network bauderate. |
| MAX_DATA_MSG_SIZE | 128 | Number of bytes of the data field. |
| NBR_NAK_RETRY | 10 | Number of retries to send after a received NAK. |
| MAX_CONTAINER_NUMBER | 5 | Number of containers in the node (memory optimisation). |
| MSG_BUFFER_SIZE | 3*size_msg | Message buffer size. Max size of a message (3 * (7 bytes header + 128 bytes data + 2 bytes CRC)). |
| MAX_MSG_NB | 2*MAX_CONTAINER_NUMBER | Max number of messages for a container that can be buffered. |
| NBR_PORT | 2 | Number of PTP on the node ( max 8). See [electronic design](../hardware_topics/electronic-design.md) page.|
