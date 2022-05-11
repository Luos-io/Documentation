---
custom_edit_url: null
---

import { customFields } from "/docusaurus.config.js";
import Tooltip from "/src/components/Tooltip.js";
import Image from '/src/components/Images.js';

# Gate

The gate is a major tool of the Luos eco-system. It's allowing you to translate any Luos achitecture into a more convenient format for standard software ([JSON] most of the time) and to stream and receive those formated informations into any kind of communication way such as serial interface, wifi, bluetooth, Lora, ...

You can use it to take control of any embedded service with [any languages](/docs/api) on any machine. For example we use it in [Pyluos](/docs/tools/pyluos) or [ROS](/docs/tools/ros)!

Gate is a simple embedded [App service](../luos-technology/services/service-api#apps-guidelines), so it can work on any MCU running Luos without any modification.

The gate service must be used with a driver service called **pipe** that can be hosted into different kinds of <Tooltip def={customFields.node_def}>node</Tooltip> allowing you to choose the communication way fitting with your project (USB, Wifi, Bluetooth, etc.)

## Default Gate Process

The default behavior of the gate is optimized for system that only have drivers and control the entire behavior through a distant machine.

1.  At power up, the gate make a network detection to find a pipe service. _(Optional)_
2.  The gate wait to receive a detection message from a pipe.
3.  At detection command, the gate perform a new detection and generate a formated routing table to send it back to the pipe.
4.  Then the gate evaluate the time needed to convert the entire network values into the selected format. _(Optional)_
5.  the gate setup all the network service to send back their values at the optimal frequency. _(Optional)_
6.  At this optimal frequency the gate generate formated data and send commands comming from a pipe.

:::caution
The gate service refreshes sensors informations as fast as it can, so that can be intensive to Luos bandwidth.
:::

The gate and the pipe are two separate services, they can be put on the same node or on separate node.

## A Gate and a pipe on the same node

In that configuration you put 2 services in the node like below.

```c
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

<div align="center">
    <Image src="/img/gate_pipe.svg" darkSrc="/img/gate_pipe_dark.svg"/>
</div>

In that configuration formated messages don't pass through the Luos network and stay in localhost.

## A Gate and a pipe on separate node

When the gate and the pipe are on separate nodes, formated messages transit into the network and use even more bandwidth on the network and add latency.

<div align="center">
    <Image src="/img/gate_pipe_separate.svg" darkSrc="/img/gate_pipe_separate-dark.svg"/>
</div>

## The gate configurations

The default process described above can be changed using different configurations that you can use on [your node_config.h](/docs/luos-technology/basics/orga#configuration).

You could need to change it if you have Apps on you Luos embedded system.

|   Parameters   | Defaults value |                            Description                             |
| :------------: | :------------: | :----------------------------------------------------------------: |
| GATE_BUFF_SIZE |      1024      |                 Maximum size of 1 formatted Data.                  |
|  GATE_POLLING  |  NOT DEFINED   | No autorefresh always ask data (more intensive to Luos bandwidth.) |
|  NODETECTION   |  NOT DEFINED   |               Gate do not make detection a power up                |

If you have an App service on your device managing detections you should define **NODETECTION** avoiding useless detection from the gate at boot.

If you have an App service on your device using auto-update you should define **GATE_POLLING** avoiding the gate to take the lead on the services your App is using.
