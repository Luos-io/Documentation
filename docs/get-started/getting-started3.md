# Part 3 Unleash your code

## The network part: Create your first Luos network.

> **Warning:** This tutorial will assume that you had followed the [Get started tutorial](/docs/get-started/getting-started). Make sure to read and follow it before reading this page.

In this part we will learn how to configure Luos to access to your physical network between boards of your system allowing you to deal with a multiple board system. Then we will try to run our first example into multiple boards.

We will use the same boards than the 2 first part, but **you need to have 2 of them to create a network**.

Supported boards are listed below:
- Arduino zero, MKRzero, MKR1000, or any SAMD21-based Arduino board
- STM32L432KC Nucleo

> **Note:** This list will grow longer with time.

### Create a physical network

In this first tutorial we will use the default wiring defined by Luos. We will see how to customize your physical interface in another tutorial.
Default hardware interface used by Luos is defined on the [LuosHAL](https://github.com/Luos-io/LuosHAL) folder corresponding to your device.

Luos need a broadcast communication, in this example we will create a OneWire network limiting the circuit to simple wires.
Luos also need a Point To Point (PTP) connection allowing to define the physical position of your boards. In this example we will have 2 PTP lines per boards.

To create your network you have to identify pins used to perform Luos communication :

| Function name | Arduino pin | STM32L432KC pin |
|--|--|--|
| TX | Pin 0 | PA9 |
| RX | Pin 1 | PA10 |
| PTPA | Pin 6 | PA5 |
| PTPB | Pin 7 | PB4 |

Now you can link your 2 boards following this wiring :

<p align="center">
      <img src="/img/Get_started_board_connection.png" />
</p>

> **Note:** You can have any PTP* connected to any another PTP* of another board. But you need only one PTP connection between boards!

> **Note 2:** Obviously you will have to power up both of your board. In the next step of this tutorial we will plug board 1 to the USB, so eventually you can use the power pins of *board 1* to power *board 2*.

### Use this network!

From the [first tutorial](/docs/get-started/getting-started) you should have a getting_started repository on your computer. We will use this code to demonstrate how Luos work using a network.

To make it we will move the blinker app service into *board 2* and see what is happening.

Open the Getting_started project corresponding to your first board then open the src/main.c or src/Arduino.ino file.

In this file there is some setup function with the naming `packageName_Init()` and `packageName_Loop()`. These line reprensent the init and loop execution of all the packages of your project.
We want to move the blinker from *board 1* to *board 2*. To make it we have to remove it from *board 1* :

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
Now flash *board 1*.

Then open the corresponding project for the second board you have. If you have twice the same board, just keep the same one...
In *board 2* we only want to have the blinker app service, so in the src/main.c or src/Arduino.ino file we just can make it like that:

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
Now flash *board 2* and it's done!

To check if everything is OK, plug an USB cable into *board 1*. The *board 1* led should blink thank to the *board 2* app!
**Congratulation you just create your first Luos distributed system!**

Now you can try to use pyluos-shell :

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
