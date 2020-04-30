# USB board
<div class="cust_sheet" markdown="1">
<p class="cust_sheet-title" markdown="1"><strong>Default Alias:</strong> gate</p>
<p class="cust_sheet-title" markdown="1"><strong>Type:</strong> <a href="/pages/high/modules_list/gate.md">Gate</a></p>
<p class="cust_sheet-title" markdown="1"><strong>Number of module(s):</strong> 1</p>
<p class="cust_sheet-title" markdown="1"><strong>Image</strong></p>
<p class="cust_indent" markdown="1"><img height="150" src="{{img_path}}/usb-module.png"></p>
<p class="cust_sheet-title" markdown="1"><strong>Category(-ies)</strong></p>
<p class="cust_indent" markdown="1">
<img height="50" src="{{img_path}}/sticker-communication.png" title="Communication">
<img height="50" src="{{img_path}}/sticker-power.png" title="Power">
</p>
<p class="cust_sheet-title" markdown="1"><strong>Project source </strong></p>
<a class="github-button" data-size="large" aria-label="Star Luos-io/Luos on GitHub" href="https://github.com/Luos-io/Examples/tree/master/Projects/Gate" target="_blank">Gate</a>
</div>

## Driver installation
With Windows, you must install `VCP` and `D2XX` drivers first. They are available for download on these pages:

<a href="https://www.ftdichip.com/Drivers/VCP.htm" target="_blank">https://www.ftdichip.com/Drivers/VCP.htm</a>

<a href="https://www.ftdichip.com/Drivers/D2XX.htm" target="_blank">https://www.ftdichip.com/Drivers/D2XX.htm</a>

Select the files to download according to your system (x86 or x64) and install them on your computer.


## How to connect the USB board to your computer
There are 2 micro-USB ports, but one of them is only used to manually update the board. The other one is the one we will use in this page.

The right USB port used on this page is the one at the opposite of the 2 Luos connectors.

![USB board]({{img_path}}/usb-1.jpg)

## How to use the USB board
Luos' USB board acts like a serial port on your system.
To control your device, you have to get and set Json data into the serial port opened by the USB board. In order do that with pyluos, you can use the following python code:

```python
from pyluos import Device
device = Device('COM13')
device.modules
```

### On Windows
On Windows, a *COM* port is usually used (like `COM1`, `COM2`, `COM3`, etc.). It can be found in *Device Manager* (right-click on *Start* button), after it’s plugged:

![Port COM]({{img_path}}/usb-2.png)

Once the port is known, the connexion can be set on pyluos with python:

```python
device = Device('COM27')
```

### On MacOS
To list the available serial ports, use the following command line:

```bash
ls /dev/cu.usbserial-*
```

Once the port is known, the connexion can be set on pyluos with python:

```python
device = Device('/dev/cu.usbserial-DN30VKKB')
```

### On Linux
To list the available serial ports, type the following in a terminal:

```bash
dmesg | grep tty
```

Once the port is known, the connexion can be set on pyluos with python:

```python
device = Device('/dev/ttyS0')
```

## USB board power delivery
The USB board power-delivery in the Luos network is limited to `500 mA`. This board can’t power too many boards and power-demanding ones like, for example, a DC-motor board with one or two motors connected. If you experiment power issues, feel free to add a power category board like a [Jack power input board]({{boards_path}}/jack-power-input.md).

<div class="cust_edit_page"><a href="https://{{gh_path}}{{boards_path}}/usb.md">Edit this page</a></div>
