---
custom_edit_url: null
---

import Image from '/src/components/Images.js';
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

# Part 3: Unleash your code

<div align="center"><iframe className="player_iframe" src="https://www.youtube.com/embed/3NsDadp1IYM?feature=oembed" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture; fullscreen" ></iframe></div>

## Summary

1. Introduction
2. Create a physical network
3. Build Luos distributed system
4. Use Pyluos to control your new network

## 1. Introduction

The first two parts of the [_Get started_ tutorial](/get-started/get-started) are now done. Good work! 💪

In this part, we will learn how to configure Luos to access to multiple boards in your physical network. Then, we will run our first example into these boards.

:::info
We will use the same board from the first two parts, but **you also need a second board to create a network**. Supported boards are listed [here](/get-started/get-started1#setup-development-environment).

:::

## 2. Create a physical network

As we saw in Part 1, Luos allows you to define services and use them together on one MCU. What really sets Luos apart and make it special is that you can also make services work together on separated MCUs.

Let’s make a network with two boards!

In this part, we will use the “default wiring” defined by Luos to create a network of two MCUs. We will use a [OneWire network](/docs/hardware-consideration/electronics), limiting the circuit to simple wires and pins:

- Tx and Rx pins, which are connected together and wired to the Tx and Rx pins of the other board (see the next image).
- One PTP wire.

:::tip
💡 Luos Point-To-Point (PTP) connection allows to find your board’s [physical position](/docs/luos-technology/node/topology). Luos can deal with up to 8 PTP lines on each board to connect as many boards as you want.

:::

:::caution
To prevent any mistakes, unplug the USB cables from the boards before wiring.

:::

<div align="center">
  <Image src="/img/get-started/get-started-3.png" darkSrc="/img/get-started/get-started-3-dark.png"/>
</div>

To create your network, you have to identify the pins used to perform Luos communication:

| Function name | Arduino pin | STM32L432KC pin | STM32F072RB pin | STM32F401RE pin | STM32F410RB pin | STM32G431KB pin |
| ------------- | ----------- | --------------- | --------------- | --------------- | --------------- | --------------- |
| TX            | Tx          | PA9 (D0)        | PA9 (D8)        | PB7(21)         | PB7 (21)        | PA9 (D0)        |
| RX            | Rx          | PA10 (D1)       | PA10 (D2)       | PB6 (D10)       | PB6 (D10)       | PA10 (D1)       |
| PTP           | D6          | PB5 (D12)       | PA8 (D7)        | PB5 (D5)        | PB5 (D5)        | PB5 (D12)       |

Below are the schematics of various boards and how to wire them:

<Tabs>
<TabItem value="Arduino MKR" label="Arduino MKR">
<div align="center">
  <Image src="/img/get-started/get-started-3-1.png" darkSrc="/img/get-started/get-started-3-1-dark.png"/>
</div>
</TabItem>
<TabItem value="Arduino Classic" label="Arduino Classic">
<div align="center">
  <Image src="/img/get-started/get-started-3-2.png" darkSrc="/img/get-started/get-started-3-2-dark.png"/>
</div>
</TabItem>
<TabItem value="NUCLEO 32" label="NUCLEO 32">
<div align="center">
  <Image src="/img/get-started/get-started-3-3.png" darkSrc="/img/get-started/get-started-3-3-dark.png"/>
</div>
</TabItem>
<TabItem value="NUCLEO 64" label="NUCLEO 64">
<Tabs>
<TabItem value="STM32F072RB" label="STM32F072RB">
<div align="center">
  <Image src="/img/get-started/get-started-3-4.png" darkSrc="/img/get-started/get-started-3-4-dark.png"/>
</div>
</TabItem>
<TabItem value="STM32F401RE/STM32F410RB" label="STM32F401RE/STM32F410RB">
<div align="center">
  <Image src="/img/get-started/get-started-3-5.png" darkSrc="/img/get-started/get-started-3-5-dark.png"/>
</div>
</TabItem>
</Tabs>
</TabItem>
</Tabs>

You should now be able to wire the two boards together. From now on, we will call these boards _board 1_ and _board 2_.

:::warning
After wiring the boards together, you will have to power both of them for the network to work correctly. In the next steps of this tutorial, _board 1_ will already be plugged to the computer with the USB cable. To power _board 2_, you can either connect the power output pin of _board 1 (5V pin)_ to the power input pin of _board 2 (Vin pin)_, or simply plug _board 2_ to another USB cable so that both boards are powered by the computer. **In the first case, do not forget to wire the GND pins together on both boards.**

:::

## 3. Build Luos distributed system

In Part 1, you have downloaded or cloned the _Get started_ code folder in your computer. We will use this code to demonstrate how Luos works using a network. We will begin by moving the blinker app service from _board 1_ to _board 2_ and see what is happening next.

### Flash _board 1_

:::caution
Connect the USB cable of _board 1_ and leave the USB cable of _board 2_ deconnected.

:::

1. In VS Code, open the folder _Get_started_ project corresponding to your _board 1_: `file/open folder`.
2. From the left panel, locate and open the file _src/main.c_ or _src/Arduino.ino_ file.
3. Comment the two lines to remove the blinker service from this board: `Blinker_Init();` and `Blinker_Loop();`

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

:::info
These lines trigger the initialization and looping execution of all the packages in your project.

:::

4. Check whether the proper board environnement is selected depending on your board:

  <div align="center">
  <Image src="/img/get-started/get-started-3-6.png" darkSrc="/img/get-started/get-started-3-6.png"/>
</div>
    
5. Build and flash _board 1_ by clicking on the right arrow button in the bottom left in VS Code.

### Flash _board 2_

:::caution
You should now unplug the USB cable of _board 1_ and connect the USB cable of _board 2_.

:::

1. In VS Code, open the folder _Get_started_ project corresponding to your _board 2_, `file/open folder`. (If you have twice the same board, you will open the same folder.)
2. From the left panel, find and open the file *src/main.c* (or *src/Arduino.ino* for Arduino users).
3. This time, comment the six lines to remove all the services except the blinker: `Led_Init();`, `Pipe_Init();`, `Gate_Init();`, `Led_Loop();`, `Pipe_Loop();`, and `Gate_Loop();`

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

:::tip
In order to keep using Luos Engine in your MCU, `Luos_init()` and `Luos_Loop()` must not be commented.

:::

4. Check if the proper board environnement is selected depending on your board:

<div align="center">
  <Image src="/img/get-started/get-started-3-7.png" darkSrc="/img/get-started/get-started-3-7.png"/>
</div>
    
5. Build and flash _board 2_ by clicking on the right arrow button in the bottom left in VS Code.

We are done!

To check if everything is OK, plug a USB cable into _board 1,_ and power _board 2_ according to your previous choice (power pins from _board 1_ or USB cable from computer).

The LED of _board 1_ should blink thanks to the blinker app in _board 2_.

**Congratulation, you just created your first Luos distributed system where a service from one board is used in another board to perform an action on this second board.**

## 4. Use Pyluos to control your network

You can now use `pyluos-shell` in your terminal, as we did in Part 2. You should see the following:

```bash
**$ pyluos-shell**
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
```

As we can see, the blinker application is now displayed on a separate board. You still can control and interact with services on both boards with pyluos, as we did in Part 2.

For example, with your network connected to the computer, follow the step 3 from Part 2 and try to execute these lines one by one in an IPython session:

1. Set up the blinking timing to 250ms (you should see the LED blink faster):

```
device.blinker.time=0.25
```

2. Pause the blinking of the LED:

```
device.blinker.pause()
```

3. Turn on the LED:

```
device.led.state=True
```

4. Turn off the LED:

```
device.led.state=False
```

5. Restart the blinking of the LED:

```
device.blinker.play()
```

## Next steps

:::info
In this _Get started_ tutorial, we used the default wiring defined by Luos. The customization of your physical interface will be addressed in a future tutorial. The default hardware interface used by Luos is defined on the [LuosHAL](https://github.com/Luos-io/LuosHAL) folder corresponding to your device.

:::

**Congratulation, you have plugged, configured, and used your first Luos network!**

You can check out our [tutorials](/tutorials/tutorials) to learn more about Luos and understand how to use the features of Luos Engine. We also invite you to check out our [documentation](/docs/luos-technology/luos_tech) to learn more about the core concepts of Luos Engine.

⭐ If you liked this tutorial, feel free to star our [Luos repository](https://github.com/Luos-io/Luos) ⭐
