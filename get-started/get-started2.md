---
custom_edit_url: null
---

import Image from '/src/components/Images.js';
import IconExternalLink from '@theme/IconExternalLink';

# Part 2: Take the control

<div align="center"><iframe className="player_iframe" src="https://www.youtube.com/embed/VcK-LJ-gnDo?start=363&feature=oembed" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture; fullscreen" ></iframe></div>

## Summary

1. Introduction
2. Setup devellopment environnement
3. Connecting and controlling your device

## 1. Introduction

This is the remote control part of this tutorial, for you to control the Matrix 💊: The service gate running on your board allows you to take control through a computer of any service loaded on your device.

## 2. Set up your development environment

We will use Python to control your Luos board with a library called **pyluos**.

In order to install it, run the following command in the terminal:

```bash
pip install pyluos
```

:::caution
As you need to use a `pip` command in your terminal, [make sure to have a working pip environment](https://pip.pypa.io/en/stable/getting-started/).

:::

:::warning
Depending on your version of Python, you may need to use the command `pip3` instead of `pip`.

:::

:::caution
If you already have Pyluos installed on your computer, please make sure it is up to date by using this prompt in your terminal: `pip install pyluos --upgrade`

:::

## 3. Connecting and controlling your device

Pyluos provides a set of application programming interfaces (APIs). To control your device, connect the same board you flashed in Part 1 and run the following command in the terminal:

```
pyluos-shell
```

This command will find the serial port of your device and mount it into a “device” object.

For example:

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
  ┃  ├> Gate                gate                1    ┃
  ┃  ╰> Unknown             blinker             4    ┃
╔>┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛
```

:::caution
In some cases, you may encounter an error message at this point. Most of the time, this is related to IPython. Installation or performing a new installation solves the problem. Type the command `pip install ipython` in the terminal.

:::

The `device` object is an object representation of your actual device displayed by pyluos-shell.

Now that you have opened an IPython session, you are able to execute Python commands line by line. By doing so, you can interact with your actual device directly into the pyluos-shell interface.

In the device object, there are multiple services. Each of those services will have special capabilities depending on their type.

For example, try to execute these lines one by one:

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

:::info
You can always check the list of the commands available for all services using your `tab` key:

<div align="center">
  <Image src="/img/get-started/get-started-2.png" darkSrc="/img/get-started/get-started-2.png"/>
</div>

See [Pyluos](https://docs.luos.io/docs/tools/pyluos) documentation for more information.

:::

## Next steps

With your development environment installed in Part 1, you have now taken the control of a Luos app running on your MCU, with Python. The [next part](/get-started/get-started3) of this section deals with creating your first Luos network.
