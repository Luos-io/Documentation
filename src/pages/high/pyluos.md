<img class="print-break" src="../../_assets/img/python-logo.png" height="100px">

<h1 class="no-break"><a href="#pyluos" class="header" id="pyluos">A Pyluos guide</a></h1>

Pyluos is the standard Python library to manage a Luos system with a computer. In this tutorial, you will learn how to install Pyluos in order to use Luos with Python on a computer, through a  [_gate_](./containers_list/gate.md) container.

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

> **Note:** Feel free to consult <a href="https://jupyter.readthedocs.io/en/latest/content-quickstart.html" target="_blank">_Jupyter Notebook_'s</a> documentation.

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

![python](../../_assets/img/pyluos-1.png)

> **Note:** In the previous picture, *Jupyter* use *Python 3* but you also can use *Python 2.7* depending on your computer configuration.

![Jupyter](../../_assets/img/pyluos-2.png)

The Jupyter work-space looks like the following image. On the keyboard,  `Maj+Enter` executes any selected part of code.

Now you are ready to code using Python.

### Import Pyluos
The first thing to do is to call the Pyluos library along with the `Device` tool inside that library:

```python
from pyluos import Device
```

This line is always used while programming behaviors and should be called before any connection with the device is made.

### Device connection

Connect your device to your computer through a [Gate](../demo_boards/boards_list/usb.html) with a USB cable.

#### Configuring USB transfer sizes and latency timer

Some devices may not work properly with the default USB transfer sizes and latency timer for COM ports on Windows. These parameters can be set to lower values in order to use your device properly while connected to your computer from a [Gate](../demo_boards/boards_list/usb.html).

**USB Transfer Sizes**: Default value is 4096 Bytes, however if you have issues to use your connected device, you should try the minimum possible values both for `Receive` and `Transmit`.

**Latency Timer**: Default value is 16 msec, but you can rise lower it to the minimal value of 1 msc.

To access to these parameters, open the Device Manager in Windows, and right-click on the *USB Serial Port (COMX)* where your device is connected, then click on *Properties*.

![](../../_assets/img/device-manager.png)

Click on *Port Settings* tab and click on *Advanced...* button.

![](../../_assets/img/serial-properties.png)

Change the desired values.

![](../../_assets/img/com-port-adv-settings.png)

These values can give you better results, for example if your device has motors to control.

#### Connection to the device

Now you should be ready to use the Pyluos library and connect to your device. To do that, you have to create a device object with your device address as an argument.

Your device address can be an IP address (`192.168.0.6` or `my_device.local` for example) or a serial port (`COM13` on Windows or `/dev/cu.usbserial-DN2YEFLN` on Mac).

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

[Routing table](../low/containers/routing-table.md) can be easily displayed using Pyluos.

Pyluos can displays a list of all the containers by filtering the routing table, and their associated characteristics (type, alias and ID).
To display it, use the following command:
```python
device.containers
```

> **Note:** `device` is the name of the network.

Pyluos will give you a list of all containers without any topological informations :
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

Pyluos also can interpreate routing_table and transform it into a tree. This way we can display a lot more complete information usinig the following command :
```python
device.nodes
```

> **Note:** `device` is the name of the network.

Based on the previous example Pyluos will give you all informations about containers and topological informations :
```AsciiDoc
  ┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓
  ┃  ╭node 1                Certified            ┃
  ┃  │  Type                Alias           ID   ┃
  ┃  ╰> Gate                gate            1    ┃
╔>┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛
║     ┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓
╚══ 0>┃1 ╭node 2                Certified            ┃
      ┃  │  Type                Alias           ID   ┃
      ┃  ├> Voltage             analog_read_P1  2    ┃
      ┃  ├> Voltage             analog_read_P7  3    ┃
      ┃  ├> Voltage             analog_read_P8  4    ┃
      ┃  ├> Voltage             analog_read_P9  5    ┃
      ┃  ├> State               digit_read_P5   6    ┃
      ┃  ├> State               digit_read_P6   7    ┃
      ┃  ├> State               digit_write_P2  8    ┃
      ┃  ├> State               digit_write_P3  9    ┃
      ┃  ╰> State               digit_write_P4  10   ┃
    ╔>┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛
    ║     ┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓
    ╚══ 0>┃1 ╭node 3                Certified            ┃
          ┃  │  Type                Alias           ID   ┃
          ┃  ╰> Angle               potentiometer_m 11   ┃
         >┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛
```
In this example, 3 nodes (MCU) and their associated UUID are listed, along with their containers and associated characteristics (type, alias and ID).
The characters after each set of node's containers and before the UUID's next node specify which connector is used. For example, `1<=>0` means the first node is connected from its second connector (1) to the first connector (0) of the next node.

### Container type

Each container has a type (eg. `Button`, `Led`, ...).
You can either retrieve your container's type from the previous code, or with the following line:

```python
device.container_alias.type
```
`container_alias` being the alias you got from the previous listing.

> **Note:** *Unknown* container types are defaulty set for custom container types such as some [Luos apps](../low/containers/create-containers.md).

### Get and set containers informations
Once you have detected your containers, you can use these information like variables.

To access values you have to address them in the device object following this rules :

```python
device.container_alias.variable
```

For example :

```python
device.rgb_led_mod.color = [50,80,5] # Change the color of the LED in "rgb_led_mod" container

device.button_mod.state # Returns the status of the push button

device.button_mod.type # Returns the container type of the container "button_mod"

device.button_mod.luos_revision # Returns the version of luos

device.button_mod.robus_revision # Returns the version of robus
```

If you use *ipython* or *Jupyter Notebook*, you can use auto-completion using the `Tab` key to find every available objects and variables.

![Auto-completion](../../_assets/img/pyluos-3.png)

### Change a container name
The name of any container can be changed following this code. 

```python
device.container_alias.rename("new_name")
```

For example:

```python
device.rgb_led_mod.rename("myLED")
```

> **Note:** You should restart your device and reconnect to it after this operation.

> **Note:** To get back to the container default name, set a void name (`""`).

### Get a node statistics
Nodes are able to send back some values representing the sanity of a node. You can use it to evaluate the Luos needs depending on your particular configuration.
The RAM usage of Luos depends on the number of messages the node has to treat and the max Luos loop delay.

```python
device.container_alias.luos_statistics
```

For example:

```python
device.gate.luos_statistics
```
```AsciiDoc
gate statistics :
.luos allocated RAM occupation  = 53%
  .Message stack                = 50%
  .Luos stack                   = 53%
.Dropped messages number        = 0
.Max luos loop delay            = 16ms
.Msg fail ratio                 = 0%
.Nak msg max number             = 1
.Collision msg max number       = 5
```
 - **luos allocated RAM occupation** represents the global Luos RAM usage based on **Message stack** and **Luos stack**. You can use this value to know if you need to expand or reduce the amount of RAM dedicated to Luos through the `MAX_MSG_NB` configuration flag (equals to `2 * MAX_CONTAINER_NUMBER` where MAX_CONTAINER_NUMBER = 5 by default ).

 - **Dropped messages number** represents the number of messages dropped by Luos. Luos is able to drop messages if they are too old and consume too much memory. If you experience message drops, you should increase the `MSG_BUFFER_SIZE` configuration flag (equals to `3 * sizeof(msg_t)` by default. sizeof(msg_t) -> 7 bytes Header + 128 bytes data).

- Contrary to **Message stack**,  **Luos stack**, **Max luos loop delay** which are Node relatif statistics, **Msg fail ratio** and **NAK msg max number** are container's statistic. **Msg fail ratio** give a ratio of msg send fail base a all the msg that the container has sent. **NAK msg max number** give the max number of NAK receive when a message has been sent.

 - The RAM occupation and message drop number is also related to **Max luos loop delay**. If **Max luos loop delay** is too big, Luos has to buffer more messages between loop executions and consumes more RAM. So you can reduce the RAM consumption and messages dropping by reducing the **Max luos loop delay**. To do that, you have to call the `Luos_Loop()` function more frequently.

### Full script

```python
from pyluos import Device
device = Device('address of the device')

device.containers

device.rgb_led_mod.color = [50,80,5]
device.button_mod.state
device.button_mod.type

device.rgb_led_mod.rename("myLED")
```
