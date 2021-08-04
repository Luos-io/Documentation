
# Luos self-healing capabilities
> **Warning:** Make sure to read and understand how to [create Luos services](./create-project.md) before reading this page.

As a developer, you will encounter bugs. 😲

Finding, understanding, and managing bugs on multiple boards running multiple services can be really hard. To make your life easier, Luos allows you to get some basic information about any problems in your system allowing you to adapt to them.

## Service exclusion
Luos includes an [acknowledgement management using the **ID_ACK** target_mode](./msg-handling.md). This mode guaranties the proper reception of critical messages.

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

> **Note:** Gates services can [report assert of other nodes trough Json](../../software/json-api.md#node-assert-messages).

> **Note:** Pyluos can display assert trough Gates.
