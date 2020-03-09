---
layout: default
title: "Pyluos"
---
{% include var.md %}

# A Pyluos guide

Pyluos is the standard Python library to manage a Luos system. In this tutorial, you will learn how to install Pyluos in order to use Luos with Python on a computer, through a module _gate_.

## Installation
### Required: Installing Python and Pip

<blockquote class="warning"><strong>Warning:</strong> In order to use Pyluos library, Python and the Pip packet manager must be installed on your computer.</blockquote>

_« Python is a programming language that lets you work more quickly and integrate your systems more effectively. » ([Source](https://python.org))_

_« [Pip](https://pip.pypa.io/en/stable/) is the standard package manager for Python. It allows you to install and manage additional packages that are not part of the Python standard library. » ([Source](https://realpython.com/what-is-pip/#getting-started-with-pip))_

If Python is not installed on you computer, download and run the last release according to your computer's OS: [https://www.python.org/downloads/](https://www.python.org/downloads/).

To install Pip, type the following commands in a console:

```bash
curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py
python get-pip.py
```

### Installing Jupyter Notebook

The tool _Jupyter Notebook_ is needed for this tutorial. Jupyter Notebook will allow you to type Python commands in an internet browser to communicate with a Luos system, via Pyluos.

_« The Jupyter Notebook App is a server-client application that allows editing and running notebook documents via a web browser. The Jupyter Notebook App can be executed on a local desktop requiring no internet access [...] or can be installed on a remote server and accessed through the internet. » ([Source](https://jupyter-notebook-beginner-guide.readthedocs.io/en/latest/what_is_jupyter.html))_

Type the following command in the console to install Jupyter:

```bash
pip install jupyter
```

> **Note:** You can consult _Jupyter Notebook_'s help at this address: [https://jupyter.readthedocs.io/en/latest/content-quickstart.html](https://jupyter.readthedocs.io/en/latest/content-quickstart.html)
 
### Installing or updating Pyluos library

You are now ready to install Pyluos. **The last Pyluos version is `{{ last_version_pyluos }}`.**

In a console, the following command will install the *Pyluos* library using the *Pip packet manager*:

```bash
pip install pyluos
```

If Pyluos is already installed, it may only need to be updated:

```bash
pip install --upgrade pyluos
```


## Start using Jupyter Notebook and Pyluos

_Jupyter Notebook_ can be launched through a console:

```bash
jupyter notebook
```

In the browser page that opened, the `New` button creates a new Python file:

![python](/assets/img/pyluos-1.png)

> **Note:** In the previous picture, *Jupyter* use *Python 3* but you also can use *Python 2.7* depending on your computer configuration.

![Jupyter](/assets/img/pyluos-2.png)

The Jupyter work-space looks like the following image. On the keyboard,  `Maj+Enter` executes any selected part of code.

Now you are ready to code using Python.

### Import Pyluos
The first thing to do is to call the Pyluos library along with the Robot tool inside that library:

```python
from pyluos import Robot
```
 
This line is always used while programming behaviors and should be called before any connection with the device is made.

### Robot connection
Now you should be ready to use the Pyluos library and connect to your robot. To do that you have to create a robot object with your robot address as argument.

Your robot address could be an IP address (192.168.0.6 or my_robot.local for example) or a serial port (COM13 on windows or /dev/cu.usbserial-DN2YEFLN on mac).

```python
robot = Robot('address of the device')
```
 
This line makes the connexion between the computer and the device. Python should answer with this kind of message:

`Connected to "address to the device".`<br />
`Sending detection signal.`<br />
`Waiting for first state...`<br />
`Robot setup.`

Only once the connection is set it is possible to start programming behaviors.

### List available boards of your robot
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


### Module type

Each board has a module type (eg. `Button`, `LED`, ...). 
You can either retrieve your board's module type from the previous code, or with the following Python code:

```python
robot.board_name.type
```
`board_name` being the alias you got from the listing.


### Get and set boards informations
Once you have detected your boards, you can use these information like variables.

To access values you have to address them in the robot object following this rules :

```python
robot.board_name.information_name
```
 
For example :

```python
robot.rgb_led_mod.color = [50,80,5] # Change the color of the LED in "rgb_led_mod" board
 
robot.button_mod.state # Returns the status of the push button
 
robot.button_mod.type # Returns the module type of the board "button_mod"
```
 
If you use *ipython* or *Jupyter Notebook*, you can use auto-completion using the `Tab` key to find every available objects.

![Auto-completion](/assets/img/pyluos-3.png)

### Change a board name
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

### Full script

```python
from pyluos import Robot
robot = Robot('address of the device')

robot.modules

robot.rgb_led_mod.color = [50,80,5] 
robot.button_mod.state 
robot.button_mod.type

robot.rgb_led_mod.rename("myLED")
```
