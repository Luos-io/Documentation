---
custom_edit_url: null
---

import Image from '/src/components/Images.js';

# Part 3: Unleash your code

## The network part: Create your first Luos network

:::caution Warning
In this last part, we assume that you had followed the first two parts of the [Get started tutorial](/get-started/get-started). Make sure to read and follow them before reading this page.
:::

In this part, we will learn how to configure Luos to access to your physical network between boards of your system, allowing you to deal with a multiple-boards system. Then we will try to run our first example into multiple boards.

We will use the same boards than in the two first parts, but **you need to have two of them to create a network**.

Supported boards are listed below [here](/get-started/get-started1#setup-development-environment).

### Create a physical network

In this tutorial, we will use the default wiring defined by Luos. The customization of your physical interface will be addressed in another tutorial.
The default hardware interface used by Luos is defined on the [LuosHAL](https://github.com/Luos-io/LuosHAL) folder corresponding to your device.

Luos needs a broadcast communication; in this example, we will create a OneWire network limiting the circuit to simple wires.
Luos also needs a Point To Point (PTP) connection allowing to define the physical position of your boards. In this example, we will have two PTP lines per board.

To create your network, you have to identify the pins used to perform Luos communication:

| Function name | Arduino pin | STM32L432KC pin | STM32F072RB pin | STM32F401RE pin | STM32F410RB pin | STM32G431KB pin |
| ------------- | ----------- | --------------- | --------------- | --------------- | --------------- | --------------- |
| TX            | Pin 0       | PA9             | PA9             | PB7             | PB7             | PA9             |
| RX            | Pin 1       | PA10            | PA10            | PB6             | PB6             | PA10            |
| PTPA          | Pin 6       | PB5             | PA8             | PB5             | PB5             | PB5             |
| PTPB          | Pin 7       | PB4             | PB13            | PB4             | PB4             | PB4             |

Now you can link both boards following this wiring :

<div align="center">
  <Image src="/img/Get_started_board_connection_black.png" darkSrc="/img/Get_started_board_connection_white.png"/>
</div>

:::tip
You can have any PTP* connected to any another PTP* of another board. But you need only one PTP connection between boards!
:::

:::note
Obviously, you will have to power up both of your board. In the next step of this tutorial, we will plug the board 1 to the USB, so you can optionally use the power pins of _board 1_ to power _board 2_.
:::

### Use this network!

From the [first _Get started_ tutorial](/get-started/get-started), you should have a _get_started_ repository on your computer. We will use this code to demonstrate how Luos works using a network.

To make it, we will move the blinker app service into _board 2_ and see what is happening.

Open the Get*started project corresponding to your first board, then open \_the src/main.c* or _src/Arduino.ino_ file.

In this file, there is some setup function with the naming `packageName_Init()` and `packageName_Loop()`. These lines reprensent the init and loop execution of all the packages of your project.
We want to move the blinker from _board 1_ to _board 2_. To make it, we have to remove it from _board 1_:

```c
...
    Luos_Init();
    Led_Init();
    Pipe_Init();
    Gate_Init();
    //Blinker_Init(); <== comment this line
...
    Luos_Loop();
    Led_Loop();
    Pipe_Loop();
    Gate_Loop();
    //Blinker_Loop(); <== comment this line
```

Now flash _board 1_.

Then open the corresponding project for the second board you have. If you have twice the same board, just keep the same one.
In _board 2_ we only want to have the blinker app service, so we can write the following code in the _src/main.c_ or _src/Arduino.ino_ files:

```c
...
    Luos_Init();
    //Led_Init(); <== comment this line
    //Pipe_Init(); <== comment this line
    //Gate_Init(); <== comment this line
    Blinker_Init();
...
    Luos_Loop();
    //Led_Loop(); <== comment this line
    //Pipe_Loop(); <== comment this line
    //Gate_Loop(); <== comment this line
    Blinker_Loop();
```

Now flash _board 2_, and it's done!

To check if everything is OK, plug a USB cable into _board 1_. The _board 1_ LED should blink thanks to the _board 2_ app.

**Congratulation, you just create your first Luos distributed system!**

Now you can try to use pyluos-shell:

```bash
$ pyluos-shell
Searching for a gate available
Testing /dev/cu.usbserial-D308N885
Testing /dev/cu.usbmodem13102
Connected to "/dev/cu.usbmodem13102".
Sending detection signal.
Waiting for routing table...
Device setup.

 Hit Ctrl-D to exit this interpreter.

Your luos device have been successfully mounted into a "device" object:
  ┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓
  ┃  ╭node 1            /!\ Not certified            ┃
  ┃  │  Type                Alias               ID   ┃
  ┃  ├> State               led                 2    ┃
  ┃  ├> Pipe                Pipe                3    ┃
  ┃  ╰> Gate                gate                1    ┃
╔>┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛
║     ┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓
╚══ 0>┃0 ╭node 2            /!\ Not certified            ┃
      ┃  │  Type                Alias               ID   ┃
      ┃  ╰> Unknown             blinker             4    ┃
     >┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛


In [1]: device.blinker.time=0.25

In [2]: device.blinker.pause()

In [3]: device.led.state=True

In [4]: device.led.state=False

In [5]: device.blinker.play()

```

## Next steps

Congratulation, you have plugged and use your firts Luos network! You can check out our [tutorials](/tutorials/tutorials) to learn how to use the features of Luos technology. We also invite you to check out our [documentation](/docs/luos-technology/luos_tech) if you want to learn more about the core concepts of Luos.
