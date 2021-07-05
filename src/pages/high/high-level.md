# High level
This part of the Luos documentation is dedicated to people aiming to control or monitor a device trough a computer or a Raspberry Pi using classic programming ways such as Python, JavaScript, or Rust.

Luos provides a Python library, [**Pyluos**](./pyluos.md) and also works with [**ROS**](./ros.md). If you want to use another language, you should start reading about our [**JSON API**](./json-api.md).


## Interface

Before using your device through JSON, you have to be connected to the communication flow depending on the node type hosting your Gate container.<br/>
The [Gate container](./containers_list/gate.md) is an app that converts Luos messages from a device's network into JSON data format, and the other way from JSON to Luos messages.<br/>
The Gate container must by use with a driver container call pipe that can be hosted into different kinds of <span class="cust_tooltip">nodes<span class="cust_tooltiptext">{{node_def}}</span></span> allowing you to choose the communication way fitting with your project (USB, Wifi, Bluetooth, etc.)

> **Warning:** The Gate container refreshes sensors information as fast as it can, so that can be intensive to Luos bandwidth.

## Using Gate and Pipe

Gate is an App container that implies this container can't be HW relative. A driver container Pipe must be create to output JSON Frame as serial com. USB, wifi or bleutooth can be use as flow to communicate in JSON with embbedded software. This two container can be put on the same node or on separate node. Without a gate and a pipe its will be impossible to communicate in JSON.

### Process
    1. Gate at the power on make a network detection to find a pipe to output JSON frame.
    2. Pipe buffered JSON message come from the flow
    3. Gate pull message in Pipe buffer
    4. Gate put message in Pipe buffer
    5. Pipe output JSON on the flow


### Gate and pipe on same node

In that configuration you put 2 containers in the node like below.

```C
#include "luos.h"
#include "gate.h"
#include "pipe.h"

int main(void)
{
  Luos_Init();
  Gate_Init();
  Pipe_Init();

  while (1)
  {
    Luos_Loop();
    Gate_Loop();
    Pipe_Loop();
  }
}
```

![](../../_assets/img/gate_pipe.png)

In that configuration JSON message from the flow don't pass through Luos network.

### Gate and pipe on separate node

When Gate and Pipe are on separate node, JSON message are buffered by pipe and when Gate pull message in Pipe buffer. The pipe divided JSON message in Luos message to gate container
![](../../_assets/img/gate_pipe_separate.png)
