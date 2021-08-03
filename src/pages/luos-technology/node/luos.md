# Luos

The embedded part of a node is separated to the Luos embedded API and to the node's different functionalities. Luos is responsible for the creation of each node's identity, the integration of the node to a Luos network by locating it among the other nodes, the communication with each other and the management of all their different functionalities.

The Luos functions need to be called only in one place for each node, but it should be run constantly.

### Luos Integration

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

## Robus

As already mentioned, the nodes have the capability to communicate with each other thanks to a specific part of Luos, Robus.

Robus is the communication protocol provided by Luos and the low layer of Luos technology. It is responsible for functionalities like the communication initialization between the different nodes, the messages' management (message format control, TX, and RX), the memory allocation, the topology detection and for the attribution of messages to the suitable handling level. 

These functionalities are analyzed in the next pages.

## Luos Statistics

Into Luos embedded code, you are given the opportunity to obtain important information about different factors of the functioning of each node, as you can find stored several statistical values in the specific field of the structure that describes each node, like for example the mcus memory utilization, or timing information.

The statistics of a node can be occupied from any other node of the system, giving you the chance to explore the behavior of all your mcus by having direct access to any of them.

More detals of how to access the statistics are given in the [Monitoring tools page](../../tools/monitoring.md).