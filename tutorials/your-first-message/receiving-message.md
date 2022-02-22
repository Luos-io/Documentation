---
custom_edit_url: null
---

import Image from '/src/components/Images.js';
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

# Part 1: Receiving Message

# Summary

1. Introduction
2. Callback Vs Polling
3. Create your Button service
4. Handle a request using the polling method

## 1. Introduction

A button can be represented (as the led) by a state-type service. But contrary to the Led the button will have to send a state value instead of receiving it.

A button service only has to manage a button, so it’s kind of dumb! Your button service doesn’t know where it needs to send its value, that’s why you will have to react to a request.

<div align="center">
  <img src ="https://media.giphy.com/media/vRNpn8HOGmpOpPmS7g/giphy.gif" className="gif_tutorial"/>
</div>

Your button service will have to receive a request message asking for the state of the button, then your service will have to send the state value back to the asking service.

<div align="center">
  <Image src="/img/your-first-message/your-first-message-1.png" darkSrc="/img/your-first-message/your-first-message-1-dark.png"/>
</div>

## 2. Callback vs polling

Since the [Create a Package ](/tutorials/your-first-service/create-a-package) tutorial, you know how to create a service on a package with a callback and stuff.

But in fact, Luos Services have 2 different ways to receive messages, callback, and polling.

- Asynchronously (CallBack)
<div align="center">
  <Image src="/img/your-first-message/your-first-message-1-1.png" darkSrc="/img/your-first-message/your-first-message-1-1-dark.png"/>
</div>
In the callback option, you give to Luos a shipping address (a message handler) allowing it to just deliver the message to your service. Your service need to be ready to receive and handle the message directly.

- Synchronously (Polling)
<div align="center">
  <Image src="/img/your-first-message/your-first-message-1-2.png" darkSrc="/img/your-first-message/your-first-message-1-2-dark.png"/>
  In the polling option, Luos don’t know your shipping address, so your service will have to check if Luos have a messages available and get it if there is.
Your service can go get a messages when he want to.
</div>
This time we will create a service without any callback to explore the polling option.

In PlatformIO IDE, open the folder corresponding to your board from the repository you cloned or downloaded `Training/2_First_Message/Work_base`, and connect the board with an USB cable.

## 3. Create your Button service

In the button package init function you can create your service but this time doesn’t give any callback and replace it with a 0:

:::caution
Throughout this tutorial, you will have to locate the right lines where to copy/paste the new line(s) code we provide to you, into your C file (Nucleo) or your INO file (Arduino). We explain it below for this first case, and then we will let you do it by your own.
:::

```c
void Button_Init(void)
{
    // the two new lines to copy and paste
    revision_t revision = {1, 0, 0};
    Luos_CreateService(0, STATE_TYPE, "button", revision);
}

void Button_Loop(void){}
```

Because you don’t give any shipping address to Luos you will have to go get it by yourself!

We only need 2 functions here `Button_Init()` and `Button_loop()`.

Let’s try it!

1. Compil and upload to the board the project.
2. Use pyluos-shell. You should see :

```bash
  ┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓
  ┃  ╭node 1            /!\ Not certified            ┃
  ┃  │  Type                Alias               ID   ┃
  ┃  ├> Pipe                Pipe                2    ┃
  ┃  ├> Gate                gate                1    ┃
  ┃  ╰> State               button              3    ┃
╔>┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛
```

<div align="center">
  <img src ="https://media.giphy.com/media/26u4lOMA8JKSnL9Uk/giphy.gif" className="gif_tutorial"/>
</div>

## 4. Handle a request using the polling method

To Pull message from Luos engine we will need to know which service is asking for it. To do that you need to create a `service_t` pointer, and at the service creation, Luos will allow you to link it to the real created service :

```c
/*******************************************************************************
 * Variable
 ******************************************************************************/
// the new line to copy and paste
service_t *button_service;
```

Assign your variable to your service creation

```c
void Button_Init(void)
{
  // the two new lines to copy and paste
  revision_t revision = {1, 0, 0};
  button_service = Luos_CreateService(0, STATE_TYPE, "button", revision);
}
```

Because we don’t give any message handler to Luos we will have to get the available messages into the service loop function using `Luos_ReadMsg` function:

```c
void Button_Loop(void)
{
  // the new block to copy and paste
  msg_t* msg;
  if (Luos_ReadMsg(button_service, &msg) == SUCCEED)
  {
    // We get a message!
  }
}
```

If we enter in this if condition this is because we have received a message so we can deal with it.

To be able to send back the button value we need to check if the received message is a good request:

```c
void Button_Loop(void)
{
		msg_t* msg;
		if (Luos_ReadMsg(button_service, &msg) == SUCCEED)
		{
        // the new line to copy and paste
				if ((msg->header.cmd == IO_STATE)||(msg->header.cmd == UNKNOW)
				{
		      // We will have to send our button info here
		    }
		}
}
```

As you can see in this message filtering, this request could be 2 different command :

- A service could ask specifically for an IO_STATE.

- A service could ask for an UNKNOW value because it doesn’t known the returned type of the value.

:::info
Sometime a service want an **UNKNOW** because it doesn’t specifically know what the returned value will be an **IO_STATE**. You can use it as a common way to get any kind of value from services.
:::

Everything is ready, we can now learn how to reply to the request and send Message!!!

<div align="center">
  <img src ="https://media.giphy.com/media/BpGWitbFZflfSUYuZ9/giphy.gif" className="gif_tutorial"/>
</div>
