# Gate

Gate is a major tool of the Luos eco-system. It's allowing you to translate any Luos achitecture into a more convenient format for standard software ([Json](../../software/json-api.md) most of the time) and to stream and receive those formated informations into any kind of communication way such as serial interface, wifi, bluetooth, Lora, ...

You can use it to take control of any embedded container with [any languages](../../software/json-api.md) on any machine. For example we use it in [Pyluos](../../software/pyluos.md) or [ROS](../../software/ros.md)!

Gate is a simple embedded [App container](../containers/create-containers.html#apps-guidelines), so it can work on any MCU running Luos without any modification.

The Gate container must be used with a driver container called **pipe** that can be hosted into different kinds of <span class="cust_tooltip">nodes<span class="cust_tooltiptext">{{node_def}}</span></span> allowing you to choose the communication way fitting with your project (USB, Wifi, Bluetooth, etc.)

## Default gate Process

 1. At power up, gate make a network detection to find a pipe container. *(Optional)*
 2. Gate wait to receive a detection message from the pipe.
 3. At detection command, Gate perform a new detection and generate a formated routing table to send it back to pipe.
 4. Then Gate evaluate the time needed to convert the entire network values into the selected format. *(Optional)*
 5. Gate setup all the network container to send back their values at the optimal frequency. *(Optional)*
 6. At this optimal frequency Gate generate formated data and send commands comming from pipe.

> **Warning:** The Gate container refreshes sensors informations as fast as it can, so that can be intensive to Luos bandwidth.

Gate and pipe are two separate containers, they can be put on the same node or on separate node.

## Gate and pipe on same node

In that configuration you put 2 containers in the node like below.

```C
#include "luos.h"
#include "pipe.h"
#include "gate.h"

int main(void)
{
  Luos_Init();
  Pipe_Init();
  Gate_Init();

  while (1)
  {
    Luos_Loop();
    Pipe_Loop();
    Gate_Loop();
  }
}
```

![](../../../_assets/img/gate_pipe.png)

In that configuration formated messages don't pass through the Luos network and stay in localhost.

## Gate and pipe on separate node

When Gate and Pipe are on separate nodes, formated messages transit into the network and use even more bandwidth on the network and add latency.
![](../../../_assets/img/gate_pipe_separate.png)
