---
layout: post
title:  "USB"
date:   2019-03-15 17:44:00 +0100
categories: -_boards_list
tags: [power, communication]
---
{% assign board = "USB" %}
{% assign alias = "Gate" %}
{% assign type = "[Gate](/../modules_list/gate)" %}
{% include var.md %}

# How to start with the {{ board }} board
{% include card.md %}

## Driver installation
With Windows, you must install `VCP` and `D2XX` drivers first. They are available for download on these pages:

[https://www.ftdichip.com/Drivers/VCP.htm](https://www.ftdichip.com/Drivers/VCP.htm)<br />
[https://www.ftdichip.com/Drivers/D2XX.htm](https://www.ftdichip.com/Drivers/D2XX.htm)

Select the files to download according to your system (x86 or x64) and install them on your computer.


## How to connect the {{ board }} board to your computer

As you can see on your board, there are 2 micro-USB ports. One of them is only used to [update your board]({{ "/../prototyping_boards/update-module-firmware.html" | absolute_url }}), and the other one is the one we will use in this page.

The right USB port used on this page is the one at the opposite of the 2 Luos connectors.

![{{ board }} board]({{ "/" | absolute_url }}/assets/img/usb-1.jpg)

## How to use the {{ board }} board
Luos' {{ board }} board acts like a serial port on your system.
To control your robot, you have to get and set Json data into the serial port opened by the {{ board }} board. In order do that with pyluos, you can use the following python code:

```python
from pyluos import Robot
robot = Robot('COM13')
robot.modules
```

### On Windows
On Windows, a *COM* port is usually used (like `COM1`, `COM2`, `COM3`, etc.). It can be found in *Device Manager* (right-click on *Start* button), after it’s plugged:

![Port COM]({{ "/" | absolute_url }}/assets/img/usb-2.png)

Once the port is known, the connexion can be set on pyluos with python:

```python
robot = Robot('COM27')
```

### On MacOS
To list the available serial ports, use the following command line:

```bash
ls /dev/cu.usbserial-*
```

Once the port is known, the connexion can be set on pyluos with python:

```python
robot = Robot('/dev/cu.usbserial-DN30VKKB')
```

### On Linux
To list the available serial ports, type the following in a terminal:

```bash
dmesg | grep tty
```

Once the port is known, the connexion can be set on pyluos with python:

```python
robot = Robot('/dev/ttyS0')
```

## {{ board }} board power delivery

The {{ board }} board power-delivery in the Luos network is limited to `500 mA`. This board can’t power too many boards and power-demanding ones like DC-motor board with one or two motors connected. If you experiment power issues, feel free to add a power category board like AC-power board.
