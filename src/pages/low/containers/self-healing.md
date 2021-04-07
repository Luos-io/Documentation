
# Luos self-healing capabilities
> **Warning:** Make sure to read and understand how to [create Luos containers](./create-project.md) before reading this page.

As a developer, you will have bugs. 😲

Finding, understanding, and managing bugs on multiple boards running multiple containers can be really hard. To make your life easier, Luos allows you to get some basic information about any problems in your system allowing you to adapt to them.

## Container exclusion
Luos includes an [acknowledgement management using the **ID_ACK** target_mode](./msg-handling.md). This mode guaranties the proper reception of critical messages.

If Luos fails to reach its target using ID_ACK, it will retry 10 times. If the acknowledgement still fails, the targeted container is declared excluded. Excluded containers are removed from the routing table to avoid any messaging from any containers, preserving bandwidth for the rest of the system.

> **Note:** Gates containers can report container exclusion through JSON.

> **Note:** Pyluos can report container exclusion through Gates.

## Luos statistics
Luos monitors some values representing the sanity of your nodes and containers.

> **Note:** Gates containers can report statistics trough JSON.

> **Note:** Pyluos can display statistics trough Gates.

### Node statistics
Inside any container, you can have access to the host node's statistics values using the `luos_stats_t` structure.
This structure gives you access to several values:

 - **memory**: Memory statisctics information
     - **rx_msg_stack_ratio**: Prcentage of memory occupation of rx message management tasks
     - **luos_stack_ratio**: Percentage of memory occupation of luos tasks
     - **tx_msg_stack_ratio**: Prcentage of memory occupation of tx message management tasks
     - **buffer_occupation_ratio**: Prcentage of memory occupation of the message buffer
     - **msg_drop_number**: Number of messages dropped due to a lack of memory (older messages are dropped to be replaced by new ones)
 - **max_loop_time_ms**: Maximum time in ms between luos_loop executions.

You can access to node statistics by using `container.node_statistics`.

### Container statistics
In any container you have access to statistics values using the `container_stats_t` structure.
This structure gives you access to a specific container statistic value:

 - **max_retry**: Maximum number of send retries due to a NAK or collision with another container

You can access node statistics by using `container.statistics`.

## Assert
Luos allows you to declare to an entire network a critical failure on a container.
To handle it, Luos exposes a `LUOS_ASSERT` macro that allows you to test some conditions on it to prevent wrong values.
for example:
``` C
 LUOS_ASSERT(arg_ptr != NULL);
```
In this case, if `arg_ptr` is not initialized, Luos will crash the entire node and send a message to all other containers with the file and line were the crash occured. All other nodes will remove all the containers from the crashed node from the routing table.

> **Note:** Gates containers can [report assert of other nodes trough Json](../../high/json-api.md#node-assert-messages).

> **Note:** Pyluos can display assert trough Gates.
