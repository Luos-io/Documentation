# Luos

The embedded part of a node is separated to the Luos embedded API and to the node's different functionalities. Luos is responsible for the creation of each node's identity, the integration of the node to a Luos network by locating it among the other nodes, the communication with each other and the management of all their different functionalities.

## Luos Integration

Luos works as a code library running on nodes. To match Luos library with your hardware, Luos offers a *Hardware Abstraction Layer* for various devices in <span class="cust_tooltip">LuosHAL<span class="cust_tooltiptext">{{luoshal_def}}</span></span>.

 - <a href="https://github.com/Luos-io/LuosHAL" target="_blank">LuosHAL</a>: This repository provides a list of family devices covered to match the Luos library with your hardware.
 - <a href="https://github.com/Luos-io/Luos" target="_blank">Luos</a>: This is the main library you will be working with.

To make it work in your environment, you have to:

 - Include the Luos lib folders in your project compilation;
 - Select the right LuosHAL for your device family in LuosHAL folder, and include `luos_hal.c`, `luos_hal.h` and `luos_hal_config.h` in your project;
 - Change, if necessary, `luos_hal_config.h` for your project. The default configuration created by Luos is an example for a MCU family that can be modify to fit with your design (eg: match pins with your design);
 - Include `luos.h` on your source file.
 
The Luos functions need to be called only in one place for each node, but it should be run constantly.

Luos is like a task that has to be run regularly. The primary Luos functions that should be called in order to integrate Luos into the embedded code of a node, are `luos_init()` and `luos_loop()` that should be added in the `main()` of your program.<br/>

Basically, your `main()` will look like this:

```C
#include "luos.h"

int main(void)
{
    Luos_Init();
    while(1)
    {
        Luos_Loop();
    }
    return 0;
}
```
Putting this code into a <span class="cust_tooltip">node<span class="cust_tooltiptext">{{node_def}}</span></span> makes it able to react to a Luos network. It's now ready to host your services.

**As a developer you will always develop your functionalities into services and never into the `main()` program.**

> **Note:** The only information that should be put on the `main()` code are MCU setup parameters and services' run functions.

## Luos APIs

In the main Luos embedded technology, we added the following tools, in order to integrate more capabilities and functionalities in your design.

| Description | Function | Return |
| :---: | :---: | :---: |
| Send a Luos message | `Luos_SendMsg(service_t *service, msg_t *msg);` | `error_return_t` |
| Read a Luos message | `Luos_ReadMsg(service_t *service, msg_t **returned_msg);` | `error_return_t` |
| Send the remaining data in case of long messages| `Luos_SendData(service_t *service, msg_t *msg, void *bin_data, uint16_t size);` | `void` |
| Receive the remaining data  in case of long messages| `Luos_ReceiveData(service_t *service, msg_t *msg, void *bin_data);` | `error_return_t` |
| Send data stored in a streaming channel | `Luos_SendStreaming(service_t *service, msg_t *msg, streaming_channel_t *stream);` | `void` |
| Receive data from a streaming channel | `Luos_ReceiveStreaming(service_t *service, msg_t *msg, streaming_channel_t *stream);` | `error_return_t` |
| Share network's baudrate| `Luos_SendBaudrate(service_t *service, uint32_t baudrate);` | `void` |
| Set the ID of a container through the network | `Luos_SetExternId(container_t *container, target_mode_t target_mode, uint16_t target, uint16_t newid);` | `uint16_t` |
| Get the number of the non treated messages left | `Luos_NbrAvailableMsg(void);` | `uint16_t` |
| Get the total tick number from the initialization of Luos | `Luos_GetSystick(void);` | `uint32_t` |
| Return true if all the messages are completed | `Luos_TxComplete(void);` | `error_return_t` |
| Flush the entire Luos message buffer | `Luos_Flush(void);` | `void` |


## Robus

As already mentioned, the nodes have the capability to communicate with each other thanks to a specific part of Luos, Robus.

Robus is the communication protocol provided by Luos and the low layer of Luos technology. It is responsible for functionalities like the communication initialization between the different nodes, the messages' management (message format control, TX, and RX), the memory allocation, the topology detection and for the attribution of messages to the suitable handling level. 

These functionalities are analyzed in the next pages.

## Luos Statistics

Into Luos embedded code, you are given the opportunity to obtain important information about different factors of the functioning of each node, as you can find stored several statistical values in the specific field of the structure that describes each node, like for example the mcus memory utilization, or timing information.

The statistics of a node can be occupied from any other node of the system, giving you the chance to explore the behavior of all your mcus by having direct access to any of them.

More detals of how to access the statistics are given in the [Monitoring tools page](../../tools/monitoring.md).

## Node Parameters Configuration 

Luos allow you to configure some parameters allowing to optimize the memory usage to your needs. To make it we advise to use a configuration file call node_config.h. Put the file at the root folder of your project and add it in the compiling variables section of your IDE using a *-include node_config.h*.

You can use it to set all your custom configuration:
 - for the services of your node
 - for Luos library of your node
 - to modify Luos HAL config to make it fit with your design

| Parameters | Defaults value | Description |
| :---: | :---: | :---: |
| NBR_NAK_RETRY | 10 | Number of retries to send after a received NAK. |
| MAX_SERVICE_NUMBER | 5 | Number of services in the node (memory optimisation). |
| MSG_BUFFER_SIZE | 3*size_msg | Message buffer size. Max size of a message (3 * (7 bytes header + 128 bytes data + 2 bytes CRC)). |
| MAX_MSG_NB | 2*MAX_SERVICE_NUMBER | Max number of messages for a service that can be referenced. |
| NBR_PORT | 2 | Number of PTP on the node ( max 8). See [electronic design](../../hardware-consideration/electronics.md) page.|

You will find the default configuration for Luos Library in the file <a href="https://github.com/Luos-io/Luos/tree/master/Robus/inc/config.h" target="_blank">config.h</a>,

Check the Luos_hal_config.h of your MCU family to see parameter that can be change to fit your design.

> **FYI:** [Every examples](https://github.com/Luos-io/Examples) provided by luos have a node_config.h files that can be use as base to fit your project needs.

