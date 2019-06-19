---
layout: default
title: "Quick start"
---
{% include var.md %}

# Getting started

Quick tutorials to get started with Luos products.

## Tutorial \#1
The following video shows a basic tutorial explaining how to make a LED and a servomotor be responsive to a potentiometer.

<iframe width="800" height="450" src="https://www.youtube.com/embed/ula16zdZgDk?feature=oembed" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe><br /><br />

---
## Tutorial \#2

On the following steps, you will learn how to make a simple behavior with a LED board and a Button board step by step.

### What you will need
 In the following example, we will make a LED turn on and off by pushing and releasing a button. You will need the following boards and accessories:

* 1x Luos LED board
* 1x Luos Button board
* 1x Luos USB board and an USB cable
* 2x Luos cables


### STEPS


#### 1. Configure your computer
The default tools we use to control a Luos network are a board hosting a gate module, with a Python lib called Pyluos.

To begin, you have to install Python and Pyluos library, following the [pyluos documentation page]({{ "/pyluos.html" | absolute_url }}).

#### 2. Plug the boards together
Plug together all the boards with cables. You can plug them to any of the two connectors of each board, in any order. 

<blockquote class="warning"><strong>Warning:</strong> Don't close a loop with the boards at each extremity.</blockquote><br />


![Boards](/assets/img/quickstart-1.png)<br />*From left to right: LED, Button, and USB . The plug order doesn’t matter.*

#### 3. Connect the device to a computer
Plug the USB board to a computer with micro-USB to USB cable.

*In this particular example there is no high consumption component so we can use the power given by USB.*

Your device is now powered and connected. All the low-level code and electronics is ready to use to help you to program your own behaviors.

![USB board](/assets/img/quickstart-2.png)<br />*The blue light from the L0 on the USB module turns on when plugged to a computer.*

#### 4. Interact with the device
*The USB node handle a specific module called "Gate". There is other board hosting "Gate" module and using different connection than USB. This particular modules convert Luos modules data into something easier to understand and manage using Json protocol.*

Interacting with the Luos system and program behaviors will require to spot the USB connection on your computer. The following steps are explained on the [General board use page]({{ "/" | absolute_url }}electronic-use.html) with more details. In the example, the associated port is COM13.

Once you know the port, you can connect using:

```python
import pyluos
from pyluos import Robot
robot = Robot('COM13')
```
When Pyluos establish the connection with a gaate module it ask to run a network detection. This detection allow to discover all modules wired together on the network.

To list the discovered modules you can run:

```python
print(robot.modules)
```

In this tutorial, Python should find three modules, the Gate (USB), the LED, and the Button modules. You can check that all are detected:

`[<Gate alias="gate" id=1>, <Button alias="button_mod" id=2>, <LED alias="rgb_led_mod" id=3>]`

Knowing the alias of the modules, you can use them.
To read something from a sensor you just have to read a variable.
For example, you can see the button state using this code:

```python
print(robot.button_mod.pressed)
```

Python will answer `True` if you execute this line by pressing the button and `False` if you don't.

The same way you can control a module by setting variables.
In this example we can control the led color using RGB values. If you type and execute the following line:

```python
robot.rgb_led_mod.color = [50,80,5]
```

The LED turns on.<br />
Changing to the value `[0, 0, 0]` will turn it off.

More details are provided on the page <a href="{{ "/" | absolute_url }}general-use.html">General use</a>.

#### 5. Write a simple beahvior
You can now write a simple behavior that makes the LED to turn on when pushing the button and turn off when releasing it.

```python
# Import libraries
from pyluos import Robot
# Establish connection with the luos network
robot = Robot('COM13')

# Use an infinite loop to put the behavior inside
while 1:
    if (robot.button_mod.pressed == True): # if the button is pushed
        robot.rgb_led_mod.color = [0,15,15] # Assigns a color to the LED
    else: # If the button is released or idle
        robot.rgb_led_mod.color = [0,0,0] # Turns the LED off
```

Test your behavior by executing the code.

![LED module](/assets/img/quickstart-3.png)


## What’s next?


This was a really simple tutorial to get you on tracks.

Now you just have to create awesome projects and share them with [the community](https://forum.luos.io).
