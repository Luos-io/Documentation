---
custom_edit_url: null
---

import Image from '/src/components/Images.js';
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

# Part 1: The topology of your system

# Summary

1. Introduction
2. Our first embedded app, a Switcher service
3. Your first detection
4. Turn the led on with our app

## 1. Introduction

In the previous training, we created a led and a button service to control them using Pyluos.

We will reuse those services and create our first fully embedded application able to detect the topology of our system and use the available services in it.

## 2. Our first embedded app, a Switcher service

We will create a simple application able to find a `led` and `button` services to turn **on** or **off** the led depending on the button state.

First, clone or download the training [repository](https://github.com/Luos-io/Training).

In your Platformio IDE, in the `Training/3_First_Detection/Work_base` open the folder corresponding to your board, and connect the board with an USB cable.

Because we already previously learn how to make a services skeleton, we already made it for you in this one.

Open the `lib/switcher/switcher.c` file on Platformio. This will be your base work!

This switcher application is specific to this tutorial, so there is no default Luos type available for it. This means that you will need to create a new custom one. Let’s call it `SWITCHER_APP`:

:::caution
Throughout this tutorial, you will have to locate the right lines where to copy/paste the new line(s) code we provide to you, into your C file (Nucleo) or your INO file (Arduino). We explain it below for this first case, and then we will let you do it by your own.

:::

```c
void Switcher_Init(void)
{
    revision_t revision = {1, 0, 0};
    Luos_CreateService(Switcher_MsgHandler, SWITCHER_APP, "Switcher", revision);
}
```

Now we need to add this type after the Luos default ones. To do that Luos provide a constant defining the beginning of the custom type list. To start your custom type list you can do :

```c
enum // Custom type list
{
    SWITCHER_APP = LUOS_LAST_TYPE
};
```

## 3. Your first detection

To be able to use other services we need to list them, define their position and purpose. To do that Luos allow you to make a topology detection. The detection has to be done at least one time by one of the services of the network to allow Luos to know how to route the different messages to their destinations. **Without detection, you can’t exchange any information between services**.

In the previous tutorials, the Gate application made the detection for you. Now it’s time to emancipate from the gate and make it by yourself

:::caution
More details about detections and topology are on [the related Luos documentation page](/docs/luos-technology/node/topology).
:::caution

To make a detection, Luos engine will need to know which service is asking for it. So we will need to pass our Switcher service to the detection function.

To do that you need to create a `service_t` pointer, and at the service creation, Luos will allow you to link it to the real created service :

```c
/*******************************************************************************
 * Variables
 ******************************************************************************/
// the new line to copy and paste
service_t *switcher_app; // This will be our switcher service
```

```c
void Switcher_Init(void)
{
    revision_t revision = {1, 0, 0};
    // the new line to copy and paste
    switcher_app = Luos_CreateService(Switcher_MsgHandler, SWITCHER_APP, "Switcher", revision);
}
```

Now we can make the detection using **`Luos_Detect()`**:

```c
void Switcher_Init(void)
{
    revision_t revision = {1, 0, 0};
    switcher_app = Luos_CreateService(Switcher_MsgHandler, SWITCHER_APP, "Switcher", revision);
    // the new line to copy and paste
	Luos_Detect(switcher_app);
}
```

:::caution
After using this API Luos will assign unique ID to all the services of the system and create a routing table containing all the service information present in the network: [ID, Alias](/docs/luos-technology/services/services/), [topology](docs/luos-technology/node/topology/). This routing table create by the service starting the detection and share to the other. More details about the routing table on the [dedicated Luos documentation page](/docs/luos-technology/services/routing-table).
:::

If you check your main files, we have a switcher package and a Led Package. All those packages create only 1 service. The service performing the detection always has the id 1 because this service will be the “root” of your system. So in our case, the switcher app will receive ID 1. Then the other services will get the next ID following the creation order, So the Led service will receive ID 2.

let’s try to turn on the led!

## 4. Turn the led on with our app

At the end of a detection, every service will receive an **END_DETECTION** message**.** To simply turn on the led on after the detection we will use this **END_DETECTION** message to send an order to the led.

```c
void Switcher_MsgHandler(service_t *service, msg_t *msg)
{
    // the new block to copy and paste
    if (msg->header.cmd == END_DETECTION)
    {
					msg_t pub_msg;
          pub_msg.header.cmd = IO_STATE;
          pub_msg.header.target_mode = ID;
          pub_msg.header.target = 2;
          pub_msg.header.size = 1;
          pub_msg.data[0] = 1;
          Luos_SendMsg(switcher_app, &pub_msg);
    }
}
```

Compile and upload the project to the board.

The service led was detected by the switcher application and at the end of the detection, a message was sent to the led service to turn on the LED.

<div align="center">
  <img src ="https://media.giphy.com/media/KcQ73bfmmsy6lg2RzG/giphy.gif" className="gif_tutorial"/>
</div>
