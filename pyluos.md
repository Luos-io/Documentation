---
layout: default
title: "Pyluos"
---
{% include var.md %}

# A Pyluos guide

The python library to manage a Luos systems.

## Installing Python and Pip

<blockquote class="warning"><strong>Warning:</strong> In order to use pyluos library, Python and the pip packet manager must be installed on your computer.</blockquote><br />

To install Python, download the last release according to your computer's OS: [https://www.python.org/downloads/]{https://www.python.org/downloads/}.

To install Pip, type the following commands in a console:

```bash
curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py
sudo python get-pip.py
```

> **Note:** If you are using Windows, the second command line must be typed without `sudo`:

```bash
python get-pip.py
```

## Installing or updating Pyluos library

In a console, the following code will install the *pyluos* library using the *pip packet manager*:

```bash
pip install pyluos
```

If pyluos is already installed, it may need to be updated:

```bash
pip install --upgrade pyluos
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

## List available boards of your robot
The following line of code refers to the listing of all the boards available in your robot.

```python
robot.modules
```
 
For example if we have an RGB led and a button ou our system we should have:

`[<Button alias="button_mod" id=2 state=OFF>,`<br />
`<LED alias="rgb_led_mod" id=3 state=[0, 0, 0]>]`

We can get the *module type*, the *alias* (name), *id* and *state* of each board.
From the first line of the previous example: `Button` is the module type, `button_mod` is the board's alias, `2` is the id and `OFF` is the state.

> **Note:** Communication boards are not shown in the boards listing and you should never have to access it.


## Module type

Each board has a module type (eg. `Button`, `LED`, ...). 
You can either retrieve your board's module type from the previous code, or with the following Python code:

```python
robot.board_name.type
```
`board_name` being the alias you got from the listing.


## Get and set boards informations
Once you have detected your boards, you can use these information like variables.

To access values you have to address them in the robot object following this rules :

```python
robot.board_name.information_name
```
 
For example :

```python
robot.rgb_led_mod.color = [50,80,5] # Change the color of the LED in "rgb_led_mod" board
 
robot.button_mod.pressed # Returns the status of the push button
 
robot.button_mod.type # Returns the module type of the board "button_mod"
```
 
If you use *ipython* or *jupyter notebook*, you can use auto-completion using the `Tab` key to find every available objects.

![Auto-completion](/assets/img/pyluos-3.png)

## Change a board name
The name of any board can be changed following this code. To list each board and its associated alias, refer to Listing the boards in a device section.

```python
robot.actual_board_name.rename("new_name")
```
 
For example:

```python
robot.rgb_led_mod.rename("myLED")
```

> **Note:** You should restart your robot and reconnect to it after this operation.

> **Note:** To get back to the board default name, set the `""` name.

