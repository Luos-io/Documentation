# General integration consideration

## Library
Luos works as a code library running on nodes. To match Luos library with your hardware, Luos offers a *Hardware Abstraction Layer* for a lot of devices in <span class="cust_tooltip">LuosHAL<span class="cust_tooltiptext">{{luoshal_def}}</span></span>.  

 - <a href="https://github.com/Luos-io/LuosHAL" target="_blank">LuosHAL</a>: This repository gives you a list of family device covers to match Luos library with your hardware.
 - <a href="https://github.com/Luos-io/Luos/tree/master/luos" target="_blank">Luos</a>: This is the main library you will be working with.

To make it work on your environment, you have to:

 - Include Luos lib folders in your project compilation;
 - Select the right LuosHAL from your family device in LuosHAL folder, and include `luos_hal.c`, `luos_hal.h` and `luos_hal_config.h` in your project;
 - Change, if necessary, `luos_hal_config.h` in you project, the default configuration created by Luos (before including `luos.h`) in order to match LuosHAL with your hardware (eg: match pins with your design);
 - Include `luos.h` on your source file.

## Configuration

Through a config file Luos allow user to custumize many parameters needed for the library. this parameters can be set as compiler variable

| Parameters | Defaults value | Desciption
| :---: | :---: | :---: | 
| PROTOCOL_REVISION | 0 | Luos protocol version
| DEFAULTBAUDRATE | 1Mbps | Nefaut network bauderate
| MAX_DATA_MSG_SIZE | 128 | number of byte of the data field
| NBR_NAK_RETRY | 10 | number of retry to send after a NAK received
| MAX_CONTAINER_NUMBER | 5 | Number of container in the node (memory optimisation)
| MSG_BUFFER_SIZE | 3*size_msg | message buffer size (3 * (7 bytes header + 128 bytes data + 2 bytes CRC))
| MAX_MSG_NB | 2*MAX_CONTAINER_NUMBER | number of task can be create (every message can create one or more task)
| NBR_PORT | 2 | number of PTP on the node ( max 8)
