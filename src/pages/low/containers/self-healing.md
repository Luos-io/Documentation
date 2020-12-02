
# Luos self-healing capabilities
> **Warning:** Make sure to read and understand how to [Create Luos containers](./create-project.md) before reading this page.

As a developer, you will have bugs. 😲

Find, understand, and manage bugs on multiple board running multiple containers could be really hard. To make your life easier Luos allow you to get some basic informations about the defaults of your system allowing you to react to it.

## Container exclusion
Luos includes an [acknowledgement management using the **ID_ACK** target_mode](./msg-handling.md). This mode guaranties the proper reception of critical messages.

If Luos fails to reach its target using ID_ACK, it will retries 10 times. If the acknowledgement still fails, the targeted container is declared excluded. Excluded containers are removed from the routing table to avoid any messaging by any containers, preserving bandwidth for the rest of the system.

> **Note:** Gates containers can report container exclusion trough Json

> **Note:** Pyluos can report container exclusion trough Gates

## Luos statistics
Luos monitor some values representing the sanity of your node and your containers.

> **Note:** Gates containers can report statistics trough Json

> **Note:** Pyluos can display statistics trough Gates

### Node statistics
In any container you have access to the host node statistics values using the `luos_stats_t` structure.
This structure give you access to different values :

 - **memory** : give you some memory statisctics informations
     - **msg_stack_ratio** : percentage of memory occupation of message management tasks
     - **luos_stack_ratio** : percentage of memory occupation of luos tasks
     - **msg_drop_number** : Number of message dropped due to a lack of memory (older message are dropped to be replaced by new ones)
 - **max_loop_time_ms** : Representing the max time in ms between luos_loop execution.

You can access node statistics by using `container.node_statistics`.

### Container statistics
In any container you have access to statistics values using the `container_stats_t` structure.
This structure give you access to different values :

 - **msg_fail_ratio** : Percentage of failed message transmission
 - **max_collision_retry** : Max number of retry due to collision on the network
 - **max_nak_retry** : Max number of retry due to a NAK of another container

You can access node statistics by using `container.statistics`.

## Assert
Luos allow you to declare to an entire network a critical failure on a container.
To manage it Luos expose a `LUOS_ASSERT` macro that allow you to test some conditions on it to prevent wrong values.
for example :
``` C
 LUOS_ASSERT(arg_ptr != NULL);
```
In this case if `arg_ptr` is not initialized Luos will crash the entire node and send a message to all other container with the file and line were the crash occure. All other nodes will remove all the containers of the crashed node from the routing table.

> **Note:** Gates containers can [report assert of other nodes trough Json](../../high/json-api.md#node-assert-messages)

> **Note:** Pyluos can display assert trough Gates
