---
custom_edit_url: null
---

import Image from '/src/components/Images.js';
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

# Part 2: Send Message from button service

# Summary

1. Introduction
2. Setup your Hardware
3. Structure of a Luos message
4. Send the button value back
5. Test the response
6. Exercice

## 1. Introduction

There is 2 way to receive a message but what about sending one!

<div align="center">
  <img src ="https://c.tenor.com/GM0o1UWir1wAAAAC/i-dont-know-how-it-works-eric-cartman.gif" className="gif_tutorial"/>
</div>

Don’t worry we see that step by step.

## 2. Setup your Hardware

Setup the MCU pin for your button in your button.c file:

<Tabs>
<TabItem value="Arduino" label="Arduino">

```c
#include <Arduino.h>
#include "button.h"

/*******************************************************************************
* Definitions
******************************************************************************/
**#define BTN_PIN 8**
```

Set the Pin as Input

```c

void Button_Init(void)
{
    revision_t revision = {1, 0, 0};
    button_service = Luos_CreateService(0, STATE_TYPE, "button", revision);
    **pinMode(BTN_PIN, INPUT);**
}
```

</TabItem>
<TabItem value="Nucleo" label="Nucleo">

For Nucleo the initialisation of your PIN as input is done in the main files by the function

```c
MX_GPIO_Init();
```

You have nothing special to do just remember the value of the pin : BTN_GPIO_Port and BTN_Pin (see main.h)
</TabItem>
</Tabs>

Now everything is ready to send back the button value.

## 3. Setup your Hardware

> By catching a message we already see that they contain some interesting information allowing you to understand the meaning of the transmitted data.

Any message contains 2 main parts :

<div align="center">
  <Image src="/img/your-first-message/your-first-message-2.png" darkSrc="/img/your-first-message/your-first-message-2.png"/>
</div>

The **Data** is the actual transmitted data.

The **Header** contains some contextual information allowing you to get the purpose of the Data.

:::info
For more information about the messages please read [the related documentation page](https://docs.luos.io/docs/luos-technology/message/message).
:::

:::tip
To send a msg you will need to fill in some header information regarding the target and the data of you message.
:::

### Message target

In this button service, we want to reply to a request coming from another service. So we will want to target this specific service and be sure it gets our answer.

To do that we will need to configure :

- **target_mode** as **IDACK ⇒** because we target only one service and want to be sure it gets our message

- **target** as the requested message **source** ⇒ because we want to target the one sending us the request.

### Message data meaning

In this message, we also need to explain the meaning of the data.

In this button service, we want to send an **IO_STATE** and our data **size** will be a byte because we just need to send 1 or 0.

To do that we will need to configure :

- **cmd** as **IO_STATE** ⇒ to give the kind of data we transmit

- **size** as **1** ⇒ because we want to send only 1 byte

- **data** as the button state ⇒ This will be our actual data

:::info
On the message request filtering, you previously made, we don’t look at the message size. This is because a request doesn’t need any data. most of the time, a message where **size = 0** is equivalent to a “get” command. A **size != 0** mean you want to set something by sending a value.
:::

## 4. Send the button value back

When we fill in all those information we are now able to send it using the `Luos_SendMsg` function.

This function needs to know who wants to send this message, you will need to give it your service, and the message you want to send.

:::info
Luos_SendMsg function return an error_return_t information allowing you to know if your message transmission **FAILED** or **SUCCEED**.
`error_return_t Luos_SendMsg(service_t *service, msg_t *msg)`
The only reason this function send you back a FAILED status is because you don’t have enough RAM space.
:::

### Make it in the code

Let’s fill in the information of the structure.

<Tabs>
<TabItem value="Arduino" label="Arduino">

```c
Button_Loop()
{
	msg_t* msg;
	while (Luos_ReadMsg(button_service, &msg) == SUCCEED)
	{
    if ((msg->header.cmd == IO_STATE)||(msg->header.cmd == UNKNOW)
    {
        // fill the message infos
        msg_t pub_msg;
        pub_msg.header.cmd         = IO_STATE;
        pub_msg.header.target_mode = IDACK;
        pub_msg.header.target      = msg->header.source;
        pub_msg.header.size        = sizeof(char); // 1 byte
        pub_msg.data[0]            = digitalRead(BTN_PIN);
        Luos_SendMsg(service, &pub_msg);
    }
	}
}
```

</TabItem>
<TabItem value="Nucleo" label="Nucleo">

```c
Button_Loop()
{
	msg_t* msg;
	while (Luos_ReadMsg(button_service, &msg) == SUCCEED)
	{
    if ((msg->header.cmd == IO_STATE)||(msg->header.cmd == UNKNOW)
    {
        // fill the message infos
        msg_t pub_msg;
        pub_msg.header.cmd         = IO_STATE;
        pub_msg.header.target_mode = IDACK;
        pub_msg.header.target      = msg->header.source;
        pub_msg.header.size        = sizeof(char); // 1 byte
        pub_msg.data[0]            = HAL_GPIO_ReadPin(BTN_GPIO_Port, BTN_Pin);
        Luos_SendMsg(service, &pub_msg);
    }
	}
}
```

</TabItem>
</Tabs>

##

Now our **STATE** button service returns an **IO_STATE** value when a service asks for it!
Let’s see if our service react as expected.

<div align="center">
  <img src ="https://media.giphy.com/media/DIxETbmfEdc6A/giphy-downsized-large.gif" className="gif_tutorial"/>
</div>

## 5. Test the response

1. Compile and upload the project to the board.
2. Using `pyluos-shell` on your terminal you should see :

```bash
  ┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓
  ┃  ╭node 1            /!\ Not certified            ┃
  ┃  │  Type                Alias               ID   ┃
  ┃  ├> Pipe                Pipe                2    ┃
  ┃  ├> Gate                gate                1    ┃
  ┃  ╰> State               button              3    ┃
╔>┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
```

Typing : `device.button.state` you should see the value of the pin

```c
In [1]: device.button.state
Out[1]: False
```

<Tabs>
<TabItem value="Arduino" label="Arduino">
To simulate a press button, connect a wire between the BTN_PIN (Pin 8) and GND.
<div align="center">
  <Image src="/img/your-first-message/your-first-message-2-1.png" darkSrc="/img/your-first-message/your-first-message-2-1-dark.png"/>
</div>

</TabItem>
<TabItem value="Nucleo1" label="STM32F072RB Nucleo/STM32F401RE Nucleo/STM32F410RB Nucleo">
Now push on the B1 button (the blue one) on your board
</TabItem>
<TabItem value="Nucleo2" label="STM32G431KB Nucleo/STM32L432KC Nucleo">
To simulate a press button, connect a wire between the BTN_PIN (Pin D10) and GND.
<div align="center">
  <Image src="/img/your-first-message/your-first-message-2-1.png" darkSrc="/img/your-first-message/your-first-message-2-1-dark.png"/>
</div>
</TabItem>
</Tabs>

Typing : `device.button.state` you should see the value of the pin

```c
In [2]: device.button.state
Out[2]: True
```

## 6. Exercice: control a led depending on your button

Let’s try a small exercice:

> Make a LED turn on when the button is True and off when it’s False

1. Add the led package created on the [previous tutorial](/tutorials/your-first-service/your-first-service) on your board
2. Compile and flash your board again.
3. Using `pyluos-shell` again, you should see :

   ```bash
     ┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓
     ┃  ╭node 1            /!\ Not certified            ┃
     ┃  │  Type                Alias               ID   ┃
     ┃  ├> Pipe                Pipe                2    ┃
     ┃  ├> Gate                gate                1    ┃
     ┃  ├> State               button              3    ┃
     ┃  ╰> State               led                 4    ┃
   ╔>┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛
   ```

4. Now on `Pyluos-shell` do:

   ```c
   while True :
   	device.led.state = device.button.state
   ```

5. Run this while loop and try to push on the button

<div align="center">
  <img src ="https://media.giphy.com/media/fSRs6SJH6m0c3iG2hk/giphy.gif" className="gif_tutorial"/>
</div>
