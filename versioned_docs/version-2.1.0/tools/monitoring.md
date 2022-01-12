---
custom_edit_url: null
---

# Monitoring

As a developer, you will encounter bugs. 😲

To help you in your development journey, Luos provides monitoring and debugging mechanisms that can give you the opportunity of having clearer visibility of your network.

## Luos self-healing capabilities

:::caution
Make sure to read and understand how to create Luos services(./create-project.md) before reading this page.
:::

Finding, understanding, and managing bugs on multiple boards running multiple services can be hard. To make your life easier, Luos allows you to get some basic information about any problems in your system, allowing you to adapt to them.

## Service exclusion

Luos includes an acknowledgment management using the **ID_ACK** target_mode. This mode guarantees the proper reception of critical messages.

If Luos fails to reach its target using ID_ACK, it will retry 10 times. If the acknowledgment still fails, the targeted service is declared excluded. Excluded services are removed from the routing table to avoid messaging from any services, preserving bandwidth for the rest of the system.

:::note

- Gates services can report service exclusion through JSON. <br/> <br/>
- Pyluos can report service exclusion through gates.

:::

## Luos statistics

Luos monitors some values representing the sanity of your nodes and services.

:::note

- Gates services can report statistics through JSON.<br/><br/>
- Pyluos can display statistics through gates.

:::

### Node statistics

Inside any service, you can access the host node's statistics values using the `luos_stats_t` structure.
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

In any service, you have access to statistics values using the `service_stats_t` structure.
This structure gives you access to a specific service's statistic value:

- **max_retry**: Maximum number of sent retries due to a NAK or collision with another service.

You can access node statistics by using `service.statistics`.

## Assert

Luos allows you to declare to an entire network a critical failure on a service.
To handle it, Luos exposes a `LUOS_ASSERT` macro that will enable you to test some conditions on it to prevent wrong values.
for example:

```c
 LUOS_ASSERT(arg_ptr != NULL);
```

In this case, if `arg_ptr` is not initialized, Luos will crash the entire node and send a message to all other services with the file and line where the crash occurred. All other nodes will remove all the services from the crashed node from the routing table.

:::note

- Gates services can [report asserting of other nodes through JSON].<br/><br/>
- Pyluos can display assert through gates.

:::

## Sniffer

An additional monitoring mechanism provided by Luos is the integration of a sniffer MCU into the network. The sniffer is responsible for gathering all the messages that are transferred into a Luos network, transmitting them serially to your computer, and displaying them in a logger, allowing you to examine the behavior of your nodes and services.

The sniffer, which consists of an application and a driver, can be easily ported on a simple MCU in the same way as a serial gate, and it can be connected to your network as any other node. The reception of the messages from the computer is achieved by transmitting the messages serially using a USB cable, while these messages are handled by pyluos (link).

All you have to do is to connect the sniffer to your MCU network and to the computer, initialize the connection using Pyluos after you spot the name of the USB port that the sniffer is connected (for example, COM13)

```python
import pyluos
from pyluos import Sniffer
sniffer = Sniffer('COM13')
```

and send the command:

```python
 sniffer.start
```

:::info
Do you need more information on how to debug your application using a sniffer?
Contact us at <a href="mailto:hello@luos.io">hello@luos.io</a>.
:::

## Inspector

In order to facilitate the debugging process, Luos offers a more advanced monitoring tool, in order to help you investigate the behavior of your network. In other words, inspector is capable of receiving the information of all the nodes and services in the network, as well as all the messages that are transmitted through it, and finally sending them to any other machine via a serial USB port.

The Inspector includes the functionalities of the sniffer, with the main difference that it is a common Luos service that works like any other service in the network. That is to say, the inspector service is detected, and can interact with the other services existing in the same network. Also, we have added some aditional functionalities, which enrich the capacities of monitoring the embedded systems.

Inspector, exactly like the Gate, is a Luos embedded [App service](/docs/luos-technology/services/services#apps-guidelines), so it is able to be hosted in any Luos running MCU.

In order to exploit the functionalities of the inspector app, we need to combine it with the communication driver service, which is called **pipe**.

The format of the messages that are transmitted serially form the inspector using the pipe driver, follow the specified [Luos communication protocol](/docs/luos-technology/message/message).

### Inspector Features

1. System's messages reception
2. Routing table sharing
3. Services' and Nodes' Statics occupation
4. Services' and Nodes' firmware revision occupation
5. Services' and Nodes' Luos revision
6. Verbose mode
7. Assert messages occupation

The Inspector and the pipe are two separate services, they can be put on the same node or on separate node. However, if they are put on the same node, they charge the network with additional messages, so there is a possibility that it can have an impact on the normal behavior of the other services.

:::note

- The inspector does not receive the messages that concern the detection, as it is a normal service which is not yet detected.

:::

:::info
Do you need more information on how to debug your application using a sniffer?
Contact us at <a href="mailto:hello@luos.io">hello@luos.io</a>.
:::