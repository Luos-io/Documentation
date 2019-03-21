---
layout: default
---
{% include var.md %}

# Getting started

Quick tutorials to get started with Luos products.

## Tutorial \#1
The following video shows a basic tutorial explaining how to make a LED and a servomotor be responsive to a potentiometer.

<iframe width="800" height="450" src="https://www.youtube.com/embed/ula16zdZgDk?feature=oembed" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe><br /><br />

---
## Tutorial \#2

On the following steps, you will learn how to make a simple behavior with a LED module and a Button modules

### What you will need
 In the following example, we will make a LED turn on and off by pushing and releasing a button. You will need the following modules and accessories:

* 1x LED module
* 1x Button module
* 1x USB module and an USB cable
* 2x Luos cables
 

### STEPS
 

#### 1. Configure your computer
See the general use guide to install Python, pyluos library, and other useful tools.

#### 2. Plug the modules together
Plug together all the modules with the cables. You can plug them to any of the two connectors of each module.

![Modules](/assets/img/quickstart-1.png)<br />*From left to right: LED module, Button module and USB module. The plug order doesn’t matter.*

#### 3. Connect the device to a computer
Plug the USB module to a computer with micro-USB to USB cable.

Your device is now powered and connected. All the low-level code is automatically created to help you to program the behaviors.  

![USB module](/assets/img/quickstart-2.png)<br />*The blue light from the L0 on the USB module turns on when plugged to a computer.*

#### 4. Interact with the device
Interacting with the device and program behaviors will require to detect it on your computer. The following steps are explained on the General use page with more details.

First, determine the associated serial port (in the example, the associated port is COM13).

Once you know the port, you can begin the connexion:

```python
import pyluos
from pyluos import Robot
robot = Robot('COM13')
```
 
The next line will list the modules detected in the device:

```python
robot.modules
```

In this tutorial, Python should find two modules, the LED and the Button modules. You can check that both are detected:

`[<Button alias="button_mod" id=2 state=OFF>,`
`<LED alias="rgb_led_mod" id=3 state=[0, 0, 0]>]`

Knowing the alias of the modules, you can now change their status. For example, if you push the button without releasing it and execute this code:

```python
robot.button_mod.pressed
```
 
Python will answer `True`

If you type and execute the following line:

```python
robot.rgb_led_mod.color = [50,80,5]
```
 
The LED turns on.<br />
Changing to the value `[0, 0, 0]` will turn it off.

More details are provided on the page <a href="{{ "/" | absolute_url }}general-use.html">General use</a>.

#### 5. Write a simple beahvior
You can now write a simple behavior that makes the LED to turn on when pushing the button and turn off when releasing it.

The following code is commented to explain

```python
# Import libraries
import pyluos
from pyluos import Robot
 
# Associate with the device
robot = Robot('COM13')
 
# define a function push_led
def push_led():
 
# Use an infinite loop to put the behavior inside
while 1:
 
if (robot.button_mod.pressed == True): # if the button is pushed
 
robot.rgb_led_mod.color = [0,15,15] # Assigns a color to the LED
 
else: # If the button is released or idle
 
robot.rgb_led_mod.color = [0,0,0] # Turns the LED off
 
# Call the function
push_led()
```
 
Test your behavior by executing the code.

![LED module](/assets/img/quickstart-3.png)


## What’s next?
 

This was a really simple tutorial to get you on tracks.

With the other modules of the collection, you can connect with Wifi and store code (Raspberry Pi module), plug several types of motors (Servo, DC motor and DXL modules), or add sensors (IMU and lidar module).

Possibilities are infinite.