---
layout: default
title: "Pyluos"
---
{% include var.md %}

# A Pyluos guide

The python library to manage a Luos systems

## Installing Pyluos library

In a console, the following code will install the *pyluos* library using the *pip packet manager*:

```bash
pip install pyluos
```
 
## Installing Jupyter Notebook
*Jupyter Notebook* can be used to program behaviors easily. We will use it to share our examples.

```bash
pip install jupyter
```
 
Once installed, it can be launched through a console:

```bash
jupyter notebook
```
 
In the browser page that opened, the `New` button creates a new Python file:

![python](/assets/img/pyluos-1.png)

> **Note:** In the previous picture, *jupyter* use *Python 3* but you also can use *Python 2.7* depending on your computer configuration.

![Jupyter](/assets/img/pyluos-2.png)

The Jupyter work-space looks like the following image. On the keyboard,  `Maj+Enter` executes any selected part of code.

Now you are ready to code using python.

## Import Pyluos
The first thing to do is to call the pyluos library along with the Robot tool inside that library:

```python
from pyluos import Robot
```
 
This line is always used while programming behaviors and should be called before any connection with the device is made.

## Robot connection
Now you should be ready to use the Pyluos library and connect to your robot. To do that you have to create a robot object with your robot address as argument.

Your robot address could be an IP address (192.168.0.6 or my_robot.local for example) or a serial port (COM13 on windows or /dev/cu.usbserial-DN2YEFLN on mac).

```python
robot = Robot('address to the device')
```
 
This line makes the connexion between the computer and the device. Python should answer with this kind of message:

`Connected to "address to the device".`<br />
`Sending detection signal.`<br />
`Waiting for first state...`<br />
`Robot setup.`

Only once the connection is set it is possible to start programming behaviors.

## List available modules of your robot
The following line of code refers to the listing of all the modules available in your robot.

```python
robot.modules
```
 
For example if we have an RGB led and a button ou our system we should have:

`[<Button alias="button_mod" id=2 state=OFF>,`<br />
`<LED alias="rgb_led_mod" id=3 state=[0, 0, 0]>]`

We can get the module type, the alias (name), the id and the state of each module.

> **Note:** Communication modules are not shown in the module listing and you should never have to access it.


## Module type

Each module has a type (eg. `Button`, `LED`, ...). 
You can retrieve the type of a module from the previous code, or with the following Python code:

```python
robot.module_name.type
```
`module_name` being the alias you got from the listing 


## Get and set modules informations
Once you have detected your modules, you can use these information like variables.

To access values you have to address them in the robot object following this rules :

```python
robot.module_name.information_name
```
 
For example :

```python
robot.rgb_led_mod.color = [50,80,5] # Change the color of the LED in "rgb_led_mod" module
 
robot.button_mod.pressed # Returns the status of the push button
 
robot.button_mod.type # Returns the type of the module "button_mod"
```
 
If you use *ipython* or *jupyter notebook*, you can use auto-completion using the `Tab` key to find every available objects.

![Auto-completion](/assets/img/pyluos-3.png)

## Change a module name
The name of any module can be changed following this code. To list each module and its associated alias, refer to Listing the modules in a device section.

```python
robot.actual_module_name.rename("new_name")
```
 
For example:

```python
robot.rgb_led_mod.rename("myLED")
```
 
> **Note:** You should restart your robot  and reconnect to it after this operation.

> **Note:** To get back to the module default name set the `""` name.