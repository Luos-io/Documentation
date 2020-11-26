# Getting started

This page provides quick and easy tutorials to get started with Luos demonstration boards.

## Tutorial \#1
On the following steps, you will learn how to make a simple behavior with a [RGB LED](boards_list/rgbled.md) board and a [Button](boards_list/button.md) board step-by-step.

### What you will need
In the following example, we will make a LED turn on and off by pushing and releasing a button. You will need the following boards and accessories:

* 1x Luos [LED board](boards_list/rgbled.md)
* 1x Luos [Button board](boards_list/button.md)
* 1x Luos [USB board](boards_list/usb.md) and an USB cable
* 2x Luos [cables](boards_list/cables.md)

### STEPS

#### 1. Configure your computer
The default tool we use to control a Luos network is a board hosting a [Gate](../high/containers_list/gate.md) container, with a Python lib called [Pyluos](../high/pyluos.md).

To begin, you have to install Python and Pyluos library, following the [pyluos documentation page](../high/pyluos.html).

#### 2. Plug the boards together
Plug together all the boards with [cables](./boards_list/cables.md). You can plug them to any of the two connectors of each board, in any order.

> **Warning:** Don't close a loop with the boards at each extremity.


![Boards](../../_assets/img/quickstart-1.png)<br />
*From left to right: LED, Button, and USB . The plug order doesn’t matter.*

#### 3. Connect the device to a computer
Plug the USB board to a computer with micro-USB to USB cable.

*In this particular example there is no high consumption component so we can use the power given by USB.*

Your device is now powered and connected. All the low-level code and electronics is ready to use to help you program your own behaviors.

![USB board](../../_assets/img/quickstart-2.png)<br />

#### 4. Interact with the device
*The USB node handle a specific container called "[Gate](../high/containers_list/gate.md)". There are other boards hosting "Gate" container and using different connection than USB. These particular containers convert Luos containers data into something easier to understand and manage, using [JSON API](../high/json-api.md).*

Interacting with the Luos system and program behaviors will require to spot the USB connection on your computer. The following steps are explained on the [General board use page](./electronic-use.md) with more details. In the following example, the associated port is `COM13`.

Once you know the port, you can connect using:

```python
import pyluos
from pyluos import Device
device = Device('COM13')
```
When Pyluos establishes the connection with a Gate container, it asks to run a network detection. This detection allows to discover all boards wired together on the network.

To list the discovered boards you can run:

```python
print(device.containers)
```

In this tutorial, Python should find three boards, the Gate (USB), the LED, and the Button boards. You can check that all are detected:

```AsciiDoc
-------------------------------------------------
Type                Alias               ID
-------------------------------------------------
Gate                gate                1
Button              button_mod          2
Led                 rgb_led_mod         3
```

Knowing the alias of the boards, you can use them in your code.
To read values from a sensor, you just have to read a variable. For example, you can see the button state using this code:

```python
print(device.button_mod.pressed)
```

Python will answer `True` if you execute this line by pressing the button and `False` if you don't.

The same way, you can control a board by setting variables.
In the following example we can control the led color using RGB values. Type and execute the following line:

```python
device.rgb_led_mod.color = [50,80,5]
```

The LED turns on.

Changing to the value `[0, 0, 0]` will turn it off.

More details are provided on the page <a href="../../demo_boards/electronic-use.md">Luos boards general use</a>.

#### 5. Write a simple beahvior
You can now write a simple behavior that makes the LED to turn on when pushing the button and turn off when releasing it.

```python
# Import libraries
from pyluos import Device
# Establish connection with the luos network
device = Device('COM13')

# Use an infinite loop to put the behavior inside
while 1:
    if (device.button_mod.pressed == True): # if the button is pushed
        device.rgb_led_mod.color = [0,15,15] # Assigns a color to the LED
    else: # If the button is released or idle
        device.rgb_led_mod.color = [0,0,0] # Turns the LED off
```

Test your behavior by executing the code.

![LED board](../../_assets/img/quickstart-3.png)

---
## Tutorial \#2
The following video shows a basic tutorial explaining how to make a LED and a servomotor be responsive to a potentiometer.

<iframe width="800" height="450" src="https://www.youtube.com/embed/ula16zdZgDk?feature=oembed" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe><br /><br />


---
## What’s next?
These were simple tutorials to get you on tracks.

Now you just have to create awesome projects and share them with <a href="https://community.luos.io" target="_blank">the community</a>.
