---
custom_edit_url: null
---

import IconExternalLink from '@theme/IconExternalLink';

# Basic button-LED device tutorial

In the following steps, you will learn to make a simple behavior with a Luos RGB LED demo board and a Luos button demo board step-by-step.

## What you will need

In the following example, we will make a LED turn on and off by pushing and releasing a button. You will need the following boards and accessories:

- 1x Luos LED board
- 1x Luos Button board
- 1x Luos USB board and a USB cable
- 2x Luos cables

## STEPS

### 1. Configure your computer

The default tool we use to control a Luos network is a board hosting a [Gate](/docs/tools/gate) service with a Python lib called [Pyluos](/docs/tools/pyluos).

To begin, you have to install Python and Pyluos library, following the [Pyluos documentation page](/docs/tools/pyluos).

### 2. Plug the boards together

Plug together all the boards with cables. You can plug them into any of the two connectors of each board, in any order.

:::caution
Don't close a loop with the boards at each extremity.
:::

![Boards](/img/quickstart-1.png)

_From left to right: LED, Button, and USB. The plug order doesn't matter._

### 3. Connect the device to a computer

Plug the USB board to a computer with micro-USB to USB cable.

_In this particular example, there is no high consumption component, so we can use the power given by USB._

Your device is now powered and connected. All the low-level code and electronics are ready to use to help you program your behaviors.

![USB board](/img/quickstart-2.png)

### 4. Interact with the device

_The USB node handles a specific service called "[gate](/docs/tools/gate)". Other boards are hosting a "gate" service and using a different connection than USB. These particular services convert Luos services data into something easier to understand and manage, using [JSON API](/docs/api/api-json)._

Interacting with the Luos system and program behaviors will require spotting the USB connection on your computer. In the following example, the associated port is `COM13`.

Once you know the port, you can connect using:

```python
import pyluos
from pyluos import Device
device = Device('COM13')
```

When Pyluos establishes the connection with a gate service, it asks to run a network detection. This detection allows discovering all boards wired together in the network.

To list the discovered boards, you can run:

```python
print(device.services)
```

In this tutorial, Python should find three boards, the gate (USB), the LED, and the Button boards. You can check that all are detected:

```AsciiDoc
-------------------------------------------------
Type                Alias               ID
-------------------------------------------------
Gate                gate                1
State               button              2
Color               rgb_led             3
```

Knowing the alias of the boards, you can use them in your code.
To read values from a sensor, you just have to read a variable. For example, you can see the button state using this code:

```python
print(device.button_mod.pressed)
```

Python will answer `True` if you execute this line by pressing the button and `False` if you don't.

In the same way, you can control a board by setting variables.
In the following example, we can control the LED color using RGB values. Type and execute the following line:

```python
device.rgb_led_mod.color = [50,80,5]
```

The LED turns on.

Changing to the value `[0, 0, 0]` will turn it off.

### 5. Write a simple beahvior

You can now write a simple behavior that makes the LED turn on when pushing the button and turn off when releasing it.

```python
# Import libraries
from pyluos import Device
# Establish connection with the Luos network
device = Device('COM13')

# Use an infinite loop to put the behavior inside
while 1:
    if (device.button.state == True): # if the button is pushed
        device.rgb_led.color = [0,15,15] # Assigns a color to the LED
    else: # If the button is released or idle
        device.rgb_led.color = [0,0,0] # Turns the LED off
```

Test your behavior by executing the code.

![LED board](/img/quickstart-3.png)

---

Don't hesitate to share your projects with the <a href="https://www.reddit.com/r/Luos/" target="_blank">Luos community<IconExternalLink width="10" /></a>.
