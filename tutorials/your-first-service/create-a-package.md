---
custom_edit_url: null
---

import Image from '/src/components/Images.js';
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

# Part 2: Create a Package

# Summary

1. How to make our service easy to move and share
2. How to create your package
3. Put the led service in your Package
4. Try our package
5. Exercice

## 1. How to make your services easy to move and share

If we want to be able to easily move our led service from one project to another it’s really more convenient to have something similar to the other services we have on the project (Pipe, Gate, Blinker). This avoids us copying and past multiple pieces of code.

The service you just created doesn’t need any direct access to any other services on your network. But the other services still have the possibility to interact with it. This is the purpose of microservice: it’s allowing you to have loosely coupled pieces of code.

Thanks to it we can put it in a folder, move it as we want, and share all our services extremely easily. We call it a [package](/docs/luos-technology/package/package).

Packages also allow you to have plenty of code on your project and keep it clean anyway! Let's see how to make one.

## 2. How to create your package

First, you will have to create a dedicated folder on your lib folder and call it “Led”:

<div align="center">
  <Image src="/img/your-first-service/luos-service-2.png" darkSrc="/img/your-first-service/luos-service-2.png"/>
</div>

Then on this folder create 2 files, “led.h” and “led.c” :

<div align="center">
  <Image src="/img/your-first-service/luos-service-2-1.png" darkSrc="/img/your-first-service/luos-service-2-1.png"/>
</div>

On the “led.h” file we will have to declare some functions allowing us to call the package from the main, declarate in the header files give acces in other files to your function.

```c
#include "luos.h"

void Led_Init(void);
void Led_Loop(void);

```

Now we can create these functions on the “led.c” file. Like you main file, package need 2 functions. `Init` call one time at the beguining and on `loop` that will be call periodicly:

:::tip
Beguin your function with the name of the package is a Luos convention : `Led_Init`
:::

<Tabs>
<TabItem value="Arduino" label="Arduino">

```c
#include "led.h"
#include "Arduino.h"

void Led_Init(void)
{
}

void Led_Loop(void)
{
}
```

</TabItem>
<TabItem value="Nucleo" label="Nucleo">

```c
#include "led.h"
#include "gpio.h"

void Led_Init(void)
{
}

void Led_Loop(void)
{
}
```

</TabItem>
</Tabs>

## 3. Put the led service in your Package

To finish you have to move the code you created on the Arduino.ino file directly into the led.c:

<Tabs>
<TabItem value="Arduino" label="Arduino">

```c
#include "led.h"
#include "Arduino.h"

// the new line to copy and paste
void Led_MsgHandler(service_t *service, msg_t *msg);

void Led_Init(void)
{
    // the three new lines to copy and paste
    pinMode(LED_BUILTIN, OUTPUT);
    revision_t revision = {1, 0, 0};
    Luos_CreateService(Led_MsgHandler, STATE_TYPE, "led", revision);
}

void Led_Loop(void)
{
 // Nothing to do here in the case of the lED because everything is made on event on Led_MsgHandler.
}

// the new block to copy and paste
void Led_MsgHandler(service_t *service, msg_t *msg)
{
    if (msg->header.cmd == IO_STATE)
    {
        if (msg->data[0] == 0)
        {
            digitalWrite(LED_BUILTIN, false);
        }
        else
        {
            digitalWrite(LED_BUILTIN, true);
        }
    }
}
```

</TabItem>
<TabItem value="Nucleo" label="Nucleo">

```c
#include "led.h"
#include "gpio.h"

// the new line to copy and paste
static void Led_MsgHandler(service_t *service, msg_t *msg);

void Led_Init(void)
{
    // the two new lines to copy and paste
    revision_t revision = {1, 0, 0};
    Luos_CreateService(Led_MsgHandler, STATE_TYPE, "led", revision);
}

void Led_Loop(void) {}

// the new block to copy and paste
static void Led_MsgHandler(service_t *service, msg_t *msg)
{
    if (msg->header.cmd == IO_STATE)
    {
        if (msg->data[0] == false)
        {
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, false);
        }
        else
        {
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, true);
        }
    }
}
```

</TabItem>
</Tabs>

Now you can directly integrate your Led_Init and Led_Loop on your Arduino.ino file like the other packages :

<Tabs>
<TabItem value="Arduino" label="Arduino">

```c
#include <Arduino.h>

#ifdef __cplusplus
extern "C"
{
#endif

#include <luos.h>
#include "blinker.h"
#include "pipe.h"
#include "gate.h"
// the new line to copy and paste
#include "led.h"

#ifdef __cplusplus
}
#endif

void setup()
{
    Luos_Init();
    Pipe_Init();
    Gate_Init();
    Blinker_Init();
    // the new line to copy and paste
	Led_Init();
}

void loop()
{
    Luos_Loop();
    Pipe_Loop();
    Gate_Loop();
    Blinker_Loop();
    // the new line to copy and paste
    Led_Loop();
}
```

</TabItem>
<TabItem value="Nucleo" label="Nucleo">

```c
#include <luos.h>
#include "blinker.h"
#include "pipe.h"
#include "gate.h"
// the new line to copy and paste
#include "led.h"

void SystemClock_Config(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();

    Luos_Init();
    Pipe_Init();
    Gate_Init();
    Blinker_Init();
    // the new line to copy and paste
    Led_Init();

    while (1)
    {
        Luos_Loop();
        Pipe_Loop();
        Gate_Loop();
        Blinker_Loop();
        // the new line to copy and paste
        Led_Loop();
    }
}
```

</TabItem>
</Tabs>

## 4. Try your new package

First, compile and upload the project to the board.

Then, use pyluos-shell, and you should see :

```bash
    ┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓
    ┃  ╭node 1            /!\ Not certified            ┃
    ┃  │  Type                Alias               ID   ┃
    ┃  ├> State               led                 2    ┃
    ┃  ├> Pipe                Pipe                3    ┃
    ┃  ├> Gate                gate                1    ┃
    ┃  ╰> Unknown             blinker             4    ┃
  ╔>┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛
```

## 5. Try your new package

Now try to move your led package on another board with only the led package on it, keep Pipe, Gate, and Blinker into the first one, and wire a one-wire network as you do on the part of the [Get Started](/get-started/get-started3).
