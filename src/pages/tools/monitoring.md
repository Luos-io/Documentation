# Monitoring

As a developer, you will encounter bugs. 😲

In order to help you in your development journey, Luos provides monitoring and debugging mechanisms, that can give you the opportunity of having a clearer visibility of your network.

## Luos self-healing capabilities
> **Warning:** Make sure to read and understand how to create Luos services(./create-project.md) before reading this page.

Finding, understanding, and managing bugs on multiple boards running multiple services can be really hard. To make your life easier, Luos allows you to get some basic information about any problems in your system allowing you to adapt to them.

## Service exclusion
Luos includes an acknowledgement management using the **ID_ACK** target_mode. This mode guaranties the proper reception of critical messages.

If Luos fails to reach its target using ID_ACK, it will retry 10 times. If the acknowledgement still fails, the targeted service is declared excluded. Excluded services are removed from the routing table to avoid any messaging from any services, preserving bandwidth for the rest of the system.

> **Note:** Gates services can report service exclusion through JSON.

> **Note:** Pyluos can report service exclusion through Gates.

## Luos statistics
Luos monitors some values representing the sanity of your nodes and services.

> **Note:** Gates services can report statistics trough JSON.

> **Note:** Pyluos can display statistics trough Gates.

### Node statistics
Inside any service, you can have access to the host node's statistics values using the `luos_stats_t` structure.
This structure gives you access to several values:

 - **memory**: Memory statisctics information.
     - **rx_msg_stack_ratio**: Percentage of memory occupation of Rx message management tasks.
     - **luos_stack_ratio**: Percentage of memory occupation of Luos tasks.
     - **tx_msg_stack_ratio**: Percentage of memory occupation of Tx message management tasks.
     - **buffer_occupation_ratio**: Percentage of memory occupation of the message buffer.
     - **msg_drop_number**: Number of messages dropped due to a lack of memory (older messages are dropped to be replaced by new ones).
 - **max_loop_time_ms**: Maximum time in ms between luos_loop executions.

You can access to node statistics by using `service.node_statistics`.

### Service statistics
In any service you have access to statistics values using the `service_stats_t` structure.
This structure gives you access to a specific service's statistic value:

 - **max_retry**: Maximum number of sent retries due to a NAK or collision with another service.

You can access node statistics by using `service.statistics`.

## Assert
Luos allows you to declare to an entire network a critical failure on a service.
To handle it, Luos exposes a `LUOS_ASSERT` macro that allows you to test some conditions on it to prevent wrong values.
for example:
``` C
 LUOS_ASSERT(arg_ptr != NULL);
```
In this case, if `arg_ptr` is not initialized, Luos will crash the entire node and send a message to all other services with the file and line were the crash occured. All other nodes will remove all the services from the crashed node from the routing table.

> **Note:** Gates services can [report assert of other nodes through Json](../api/api.md).

> **Note:** Pyluos can display assert through Gates.

## Sniffer

An additional monitoring mechanism provided by Luos is the integration of a sniffer MCU into the network. The sniffer is responsible of gathering all the messages that are transfered into a Luos network, transmitting them serially to your computer, and displaying them in a logger, giving you the opportunity to examine the behavior of your nodes and services.

The sniffer, which consists of an application and a driver, can be easily ported on a simple MCU in the same way as a serial gate and it can be connected to your network as any other node. The reception of the messages from the computer is being achieved by transmitting the messages serially using a USB cable, while these messages handled by pyluos (link).

All you have to do is to connect the sniffer to your MCU network and to the computer, initialize the connection using pyluos, after you spot the name of the usb port that the sniffer is connected (for example COM13) 

```python
import pyluos
from pyluos import Sniffer
sniffer = Sniffer('COM13')
```
and send the command:

```python
 sniffer.start
```

> **Note:** Do you need more information on how to debug your application using a sniffer? 

> Contact us at <a href="mailto:hello@luos.io">  hello@luos.io</a>.
