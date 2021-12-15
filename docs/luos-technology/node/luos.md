---
custom_edit_url: null
---

import { customFields } from "/docusaurus.config.js";
import Tooltip from "/src/components/Tooltip.js";
import Image from '/src/components/Images.js';
import IconExternalLink from '@theme/IconExternalLink';

# Luos

The node's embedded code hosts the Luos's embedded code and the node's various functionalities, stored in services. Luos is responsible for creating each node's identity, integrating the node to a Luos network, locating it among the other nodes, communicating with each other, and managing all their services.

## Luos Integration

Luos works as a code library running on nodes. To match the Luos library with your hardware, Luos offers a _Hardware Abstraction Layer_ for various devices in <Tooltip def={customFields.luoshal_def}>LuosHAL</Tooltip>.

- <a href="https://github.com/Luos-io/LuosHAL" target="_blank">LuosHAL<IconExternalLink width="10" /></a>: This repository provides a list of family devices covered to match the Luos library with your hardware.
- <a href="https://github.com/Luos-io/Luos" target="_blank">Luos<IconExternalLink width="10" /></a>: The main library you will be working with.

To make it work in your environment, you have to:

- Include the Luos lib folders in your project compilation;
- Select the right LuosHAL for your device family in LuosHAL folder, and include `luos_hal.c`, `luos_hal.h` and `luos_hal_config.h` in your project;
- If necessary, overload `luos_hal_config.h` with a `node_config.h` file describing specificities of your node. The default configuration created by Luos is an example of an MCU family that can be modified to fit with your design (e.g. match pins with your design);
- Include `luos.h` on your main file.

The Luos functions need to be called only in one place for each node, but it should be run constantly.

Luos is like a task that has to be run regularly. The primary Luos functions that should be called to integrate Luos into the embedded code of a node are `luos_init()` and `luos_loop()`. They should be added in the `main()` of your program.

Basically, your `main()` function will look like this:

```c
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

Adding this code to a <Tooltip def={customFields.node_def}>node</Tooltip> makes it able to react to a Luos network. It is now ready to host your services by [running packages](../package/package.md) on your main.

**As a developer, you will always develop your functionalities into services and never into the `main()` program.**

:::note
The only information that should be put on the `main()` code are MCU's setup parameters and services' run functions.
:::

## A complete software node view

At the node level, communication is achieved by receiving and sending [messages](../message/message.md) with the other components of a Luos network. The nodes can communicate with each other thanks to a specific part of Luos, called Robus.

Robus is the communication protocol provided by Luos and the low layer of Luos technology. It is responsible for functionalities like communication initialization between different nodes, messages' management ([message format control](../message/message.md), TX, and RX), memory allocation, topology [detection](/docs/luos-technology/services/routing-table), and attribution of messages to the suitable handling level.

Robus executes a format control, and store messages in the `msg_buffer` of your node. Depending on the specified destination and the type of each message, they are either treated automatically by Robus and Luos or sent to one or several [services](../services/services.md).

<div align="center">
<Image src="/img/NodeFlow.svg" darkSrc="/img/NodeFlow-dark.svg" />
</div>

## Node Parameters Configuration

Luos allows you to configure some parameters to optimize the memory usage and adapt it to fit your needs. To make it, we advise using a configuration file called _node_config.h_. Put the file at the root folder of your node project and add it in the compiling variables section of your IDE by adding the following line:

`#include node_config.h`

You can use it to set all your custom configurations:

- for the services of your node
- for Luos library of your node
- to modify Luos HAL config to make it fit with your design

|     Parameters     |    Defaults value     |                                                      Description                                                      |
| :----------------: | :-------------------: | :-------------------------------------------------------------------------------------------------------------------: |
|   NBR_NAK_RETRY    |          10           |                                    Number of retries to send after a received NAK.                                    |
| MAX_SERVICE_NUMBER |           5           |                                 Number of services in the node (memory optimization).                                 |
|  MSG_BUFFER_SIZE   |      3\*size_msg      |          Message buffer size. Max size of a message (3 \* (7 bytes header + 128 bytes data + 2 bytes CRC)).           |
|     MAX_MSG_NB     | 2\*MAX_SERVICE_NUMBER |                                Max number of messages that can be referenced by Luos.                                 |
|      NBR_PORT      |           2           | Number of PTP (port) on the node ( max 8). See [electronic design](../../hardware-consideration/electronics.md) page. |

You will find the default configuration for Luos Library in the file <a href="https://github.com/Luos-io/Luos/tree/master/Robus/inc/config.h" target="_blank">config.h<IconExternalLink width="10" /></a>,

Check the Luos_hal_config.h of your MCU family to see parameters that can be changed to fit your design.

> **Note:** [Every example](https://github.com/Luos-io/Examples) provided by Luos has a _node_config.h_ file that can be used as a base to fit your project's needs.

## Luos Statistics

Into Luos embedded code, you are given the opportunity to obtain important information about different factors of the functioning of each node, as you can find stored several statistical values in the specific field of the structure that describes each node, like for example, the MCUs memory utilization, or timing information.

The statistics of a node can be occupied from any other node of the system, giving you the chance to explore the behavior of all your MCUs by having direct access to any of them.

More details of how to access the statistics are given in the [Monitoring tools page](../../tools/monitoring.md).
