# A Pyluos guide
Pyluos is the standard Python library to manage a Luos system with a computer. In this tutorial, you will learn how to install Pyluos in order to use Luos with Python on a computer, through a  [_gate_](/pages/high/modules_list/gate.md) module.

## Installation

### Required: Installing Python and Pip
> **Warning:** In order to use Pyluos library, Python and the Pip packet manager must be installed on your computer.

_« Python is a programming language that lets you work more quickly and integrate your systems more effectively. » (<small><a href="https://python.org" target="_blank">Source</a></small>)_

_« <a href="https://pip.pypa.io/en/stable/" target="_blank">Pip</a> is the standard package manager for Python. It allows you to install and manage additional packages that are not part of the Python standard library. » (<small><a href="https://realpython.com/what-is-pip/#getting-started-with-pip" target="_blank">Source</a></small>)_

If Python is not installed on you computer, download and run the last release according to your computer's OS: <a href="https://www.python.org/downloads/" target="_blank">https://www.python.org/downloads/</a>.

To install Pip, type the following commands in a console:

```bash
curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py
python get-pip.py
```

### Installing Jupyter Notebook
The tool _Jupyter Notebook_ is needed for this tutorial. Jupyter Notebook will allow you to type Python commands in an internet browser to communicate with a Luos system, via Pyluos.

_« The Jupyter Notebook App is a server-client application that allows editing and running notebook documents via a web browser. The Jupyter Notebook App can be executed on a local desktop requiring no internet access (...) or can be installed on a remote server and accessed through the internet. » (<small><a href="https://jupyter-notebook-beginner-guide.readthedocs.io/en/latest/what_is_jupyter.html" target="_blank">Source</a></small>)_

Type the following command in the console to install Jupyter:

```bash
pip install jupyter
```

> **Note:** Feel free to consult <a href="https://jupyter.readthedocs.io/en/latest/content-quickstart.html" target="_blank">_Jupyter Notebook_'s</a>.

### Installing or updating Pyluos library
You are now ready to install Pyluos. **The last Pyluos version is `{{last_version_pyluos}}`.**

In a console, the following command will install the *Pyluos* library using the *Pip packet manager*:

```bash
pip install pyluos
```

If Pyluos is already installed, it may only need to be updated:

```bash
pip install --upgrade pyluos
```

Pyluos also provides auto generated pre-releases for advanced developers user. You can get it using:
```bash
pip install --pre pyluos
```

## Start using Jupyter Notebook and Pyluos

_Jupyter Notebook_ can be launched through a console:

```bash
jupyter notebook
```

In the browser page that opened, the `New` button creates a new Python file:

![python](/_assets/img/pyluos-1.png)

> **Note:** In the previous picture, *Jupyter* use *Python 3* but you also can use *Python 2.7* depending on your computer configuration.

![Jupyter](/_assets/img/pyluos-2.png)

The Jupyter work-space looks like the following image. On the keyboard,  `Maj+Enter` executes any selected part of code.

Now you are ready to code using Python.

### Import Pyluos
The first thing to do is to call the Pyluos library along with the `Device` tool inside that library:

```python
from pyluos import Device
```

This line is always used while programming behaviors and should be called before any connection with the device is made.

### Device connection
Now you should be ready to use the Pyluos library and connect to your device. To do that you have to create a device object with your device address as argument.

Your device address could be an IP address (192.168.0.6 or my_device.local for example) or a serial port (COM13 on windows or /dev/cu.usbserial-DN2YEFLN on mac).

```python
device = Device('address of the device')
```

This line makes the connexion between the computer and the device. Python should answer with this kind of message:

`Connected to "address to the device".`<br />
`Sending detection signal.`<br />
`Waiting for first state...`<br />
`Device setup.`

Only once the connection is set it is possible to start programming behaviors.


### Routing table display

[Routing table](/pages/low/modules/routing-table.md) can be easily displayed using Pyluos.

Pyluos can displays a list of all the modules by filtering the route table, and their associated characteristics (type, alias and ID).
To display it, use the following command:
```python
device.modules
```

> **Note:** `device` is the name of the network.

Pyluos will give you a list of all modules without any topological informations :
```AsciiDoc
-------------------------------------------------
Type                Alias               ID
-------------------------------------------------
Gate                gate                1
Voltage             analog_read_P1      2
Voltage             analog_read_P7      3
Voltage             analog_read_P8      4
Voltage             analog_read_P9      5
State               digit_read_P5       6
State               digit_read_P6       7
State               digit_write_P2      8
State               digit_write_P3      9
State               digit_write_P4      10
Angle               potentiometer_m     11
```

Pyluos also can interpreate route_table and transform it into a tree. This way we can display a lot more complete information usinig the following command :
```python
device.nodes
```

> **Note:** `device` is the name of the network.

Based on the previous example Pyluos will give you all informations about modules and topological informations :
```AsciiDoc
 root : [4653093, 1194612501, 540554032]
        |  Type                Alias               ID
        └> Gate                gate                1
└── 1<=>0 : [4456498, 1347571976, 540555569]
            |  Type                Alias               ID
            └> Voltage             analog_read_P1      2
            └> Voltage             analog_read_P7      3
            └> Voltage             analog_read_P8      4
            └> Voltage             analog_read_P9      5
            └> State               digit_read_P5       6
            └> State               digit_read_P6       7
            └> State               digit_write_P2      8
            └> State               digit_write_P3      9
            └> State               digit_write_P4      10
    └── 1<=>0 : [4653107, 1347571976, 540555569]
                |  Type                Alias               ID
                └> Angle               potentiometer_m     11
```
In this example, 3 nodes (MCU) and their associated UUID are listed, along with their modules and associated characteristics (type, alias and ID).
The characters after each set of node's modules and before the UUID's next node specify which connector is used. For example, `1<=>0` means the first node is connected from it's second connector (1) to the first connector (0) of the next node.

### Module type

Each module has a type (eg. `Button`, `Led`, ...).
You can either retrieve your module's type from the previous code, or with the following line:

```python
device.module_alias.type
```
`module_alias` being the alias you got from the previous listing.

> **Note:** *Unknown* module types are defaulty set for custom module types such as some [Luos apps](/pages/low/modules/create-modules.md).

### Get and set modules informations
Once you have detected your modules, you can use these information like variables.

To access values you have to address them in the device object following this rules :

```python
device.module_alias.variable
```

For example :

```python
device.rgb_led_mod.color = [50,80,5] # Change the color of the LED in "rgb_led_mod" module

device.button_mod.state # Returns the status of the push button

device.button_mod.type # Returns the module type of the module "button_mod"

device.button_mod.luos_revision # Returns the version of luos

device.button_mod.robus_revision # Returns the version of robus
```

If you use *ipython* or *Jupyter Notebook*, you can use auto-completion using the `Tab` key to find every available objects and variables.

![Auto-completion](/_assets/img/pyluos-3.png)

### Change a module name
The name of any module can be changed following this code. To list each module and its associated alias, refer to [List available modules of your device](#list-available-modules-of-your-device) section.

```python
device.module_alias.rename("new_name")
```

For example:

```python
device.rgb_led_mod.rename("myLED")
```

> **Note:** You should restart your device and reconnect to it after this operation.

> **Note:** To get back to the module default name, set a void name (`""`).

### Full script

```python
from pyluos import Device
device = Device('address of the device')

device.modules

device.rgb_led_mod.color = [50,80,5]
device.button_mod.state
device.button_mod.type

device.rgb_led_mod.rename("myLED")
```

<div class="cust_edit_page"><a href="https://{{gh_path}}/pages/first_steps/pyluos.md">Edit this page</a></div>
