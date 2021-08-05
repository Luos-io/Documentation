# Gate

The Gate is a major tool of the Luos eco-system. It's allowing you to translate any Luos achitecture into a more convenient format for standard software ([Json] most of the time) and to stream and receive those formated informations into any kind of communication way such as serial interface, wifi, bluetooth, Lora, ...

You can use it to take control of any embedded service with [any languages] on any machine. For example we use it in [Pyluos] or [ROS]!

Gate is a simple embedded [App service](../services/create-services.html#apps-guidelines), so it can work on any MCU running Luos without any modification.

The Gate service must be used with a driver service called **pipe** that can be hosted into different kinds of <span className="cust_tooltip">nodes<span className="cust_tooltiptext">{{node_def}}</span></span> allowing you to choose the communication way fitting with your project (USB, Wifi, Bluetooth, etc.)

## Default Gate Process

The default behavior of the Gate is optimized for system that only have drivers and control the entire behavior trough a distant machine.

1.  At power up, the Gate make a network detection to find a pipe service. _(Optional)_
2.  The Gate wait to receive a detection message from a pipe.
3.  At detection command, the Gate perform a new detection and generate a formated routing table to send it back to the pipe.
4.  Then the Gate evaluate the time needed to convert the entire network values into the selected format. _(Optional)_
5.  the Gate setup all the network service to send back their values at the optimal frequency. _(Optional)_
6.  At this optimal frequency the Gate generate formated data and send commands comming from a pipe.

> **Warning:** The Gate service refreshes sensors informations as fast as it can, so that can be intensive to Luos bandwidth.

The Gate and the pipe are two separate services, they can be put on the same node or on separate node.

## A Gate and a Pipe on the same node

In that configuration you put 2 services in the node like below.

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

![](/img/gate_pipe.png)

In that configuration formated messages don't pass through the Luos network and stay in localhost.

## A Gate and a Pipe on separate node

When the Gate and the Pipe are on separate nodes, formated messages transit into the network and use even more bandwidth on the network and add latency.
![](/img/gate_pipe_separate.png)

## The Gate configurations

The default process described above can be changed using different configurations that you can use on [your node_config.h](../dev_env/general-integration-consideration.html#configuration).

You could need to change it if you have Apps on you Luos embedded system.

|   Parameters   | Defaults value |                            Description                             |
| :------------: | :------------: | :----------------------------------------------------------------: |
| GATE_BUFF_SIZE |      1024      |                 Maximum size of 1 formatted Data.                  |
|  GATE_POLLING  |  NOT DEFINED   | No autorefresh always ask data (more intensive to Luos bandwidth.) |
|  NODETECTION   |  NOT DEFINED   |               Gate do not make detection a power up                |

If you have an App service on your device managing detections you should define **NODETECTION** avoiding useless detection from the Gate at boot.

If you have an App service on your device using auto-update you should define **GATE_POLLING** avoiding the Gate to take the lead on the services your App is using.
