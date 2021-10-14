# Get started

This page allows you to build, flash, run, and control your very first Luos code.

This Get started is separated in 2 parts:
 1. **The embedded part**: By following this part you will have all the tools you need to easily develop using Luos in your embedded target.
 2. **The remote control part**: By following this part you will have all the tools you need to take control and easily test any Luos device.

 In order to complete this tutorial you will need to install dependancies such as <a href="https://www.python.org" target="_blank">Python</a>, <a href="https://git-scm.com/" target="_blank">GIT</a>, the USB driver compatible with your board and <a href="https://ipython.org/" target="_blank">Ipython</a>. How to install and use those dependancies is not our purpose here, however we will provide you with links to help you along the process.

## 1. The embedded part: Run your first embedded app!

This tutorial shows you how to quickly upload a Luos application on a Microcontroller Unit (MCU) development kit.

Supported boards are listed below:
- <a href="https://www.arduino.cc/en/Main/ArduinoBoardZero&" target="_blank">Arduino zero</a>, <a href="https://store.arduino.cc/products/arduino-mkr-zero-i2s-bus-sd-for-sound-music-digital-audio-data" target="_blank">MKRzero</a>, <a href="https://store.arduino.cc/collections/boards/products/arduino-mkr1000-wifi" target="_blank">MKR1000</a>, or any <a href="https://en.wikipedia.org/wiki/List_of_Arduino_boards_and_compatible_systems" target="_blank">SAMD21-based</a> Arduino board
- <a href="https://www.st.com/en/evaluation-tools/nucleo-l432kc.html" target="_blank">STM32L432KC Nucleo</a>

> **Note:** This list will grow longer with time.

### Setup development environment

We will use <a href="https://platformio.org/platformio-ide" target="_blank">PlatformIO &#8599;</a> as development environment.

First, download and install the free coding editor <a href="https://code.visualstudio.com/" target="_blank">Microsoft Visual Studio Code &#8599;</a> (VSCode). PlatformIO's IDE is built on top of it.

Then, in VSCode:

<p align="center">
  <img src="../../_assets/img/get-started/install_VSCODE.png" />
</p>

 1. Open VSCode Extension Manager.
 2. Search for the official [PlatformIO IDE](https://platformio.org/install/ide?install=vscode) extension.
 3. Install PlatformIO IDE.

### Clone the project

There is two ways to clone the *getting started* repository on your computer: 

- via terminal, using this command line (in this case you will need to have <a href="https://git-scm.com/downloads" target="_blank">GIT</a> previewsly installed on your computer)
```bash
git clone https://github.com/Luos-io/getting_started.git
```

If you are not familiar with Git, you can consult <a href="https://git-scm.com/doc" target="_blank">their documentation &#8599;</a>.

### Flash your board depending on which one you have

Open VSCode and click on **Open Folder** in the project explorer on the left, then select a project depending on the board you have chosen. For example, for the STM32L432KC Nucleo, open **L432KC_Nucleo** in the folder explorer then click on **ADD**:

<p align="center">
  <img src="../../_assets/img/get-started/Open_project2.png" />
</p>

The project folder is opened in the explorer.

> **Note:** Arduino users can select their board on the `platformio.ini` file by modifying the `board = zero` line.

You can now flash your board: make sure it's connected to your PC with a USB cable and click on **Upload** on the bottom left of the VSCode window:

<p align="center">
  <img src="../../_assets/img/get-started/Flash_board2.png" />
</p>

PlatformIO will build the firmware and flash it. Take a look at the terminal to watch each step platformIO follows and a **Success** message at the end. If you have any trouble with your USB driver, you can consult [this FAQ page](../faq/002.dfu.md). Once the board is programmed, you should see the **LED** blinking on your board.

Congratulations, your first Luos app is running!

### What is going on

There are two [**services**](../luos-technology/services/services.md) loaded in your board allowing to blink the LED.

- **Blinker** sends a message at a fixed duration</br> ╰ located at the root of the *getting_started* repository (because the same app can run on any board)
- **Led** receives this message and makes the LED blink</br> ╰ located on the *lib* folder of your project (because it is a driver which is specific to your board)

On top of it, we also added two other services allowing you to take control of your board:

- **Pipe**, managing a serial interface</br> ╰ located on the *lib* folder of your project (because it is a driver which is specific to your board)
- **Gate**, an app that translates Luos to JSON and sends it through *Pipe*</br> ╰ located in the cloud, (because it is a common cross-platform's Luos app) PlatformIO just downloaded it for you.

## 2. The remote control part: 💊 You can control the Matrix.

The gate running on your board allows you to take control of any service loaded on your device.

### Setup development environment

We will use Python with the default library of Luos called pyluos.
To install it, run:

```bash
pip install pyluos
```

### Connect and control your device

Pyluos provides a set of tools. To control you device, run:

```bash
pyluos-shell
```

This command will find the serial port of your device and mount it into a "device" object.

For example:

```bash
$ pyluos-shell
Searching for a gate available
Testing /dev/cu.usbserial-D308N885
Testing /dev/cu.usbmodem13102
Connected to "/dev/cu.usbmodem13102".
Sending detection signal.
Waiting for routing table...
Device setup.

 Hit Ctrl-D to exit this interpreter.

Your luos device have been successfully mounted into a "device" object:
  ┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓
  ┃  ╭node 1            /!\ Not certified            ┃
  ┃  │  Type                Alias               ID   ┃
  ┃  ├> State               led                 2    ┃
  ┃  ├> Pipe                Pipe                3    ┃
  ┃  ├> Gate                gate                1    ┃
  ┃  ╰> Unknown             blinker             4    ┃
╔>┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛

```

Now that you are on an IPython command line, you can run Python scripts in it.
The `device` object is your real device and you can interact with it. For example, try to execute these lines one by one:

In \[1\]: `device.blinker.time=0.25`  
In \[2\]: `device.blinker.pause()`  
In \[3\]: `device.led.state=True`  
In \[4\]: `device.led.state=False`  
In \[5\]: `device.blinker.play()`  

## Next steps

Your development environment is now installed and you have a Luos app running on your MCU. Check the [tutorials](../tutorials/tutorials.md) to learn how to use each feature of Luos technology. You can also read the [Luos documentation](../luos-technology/luos_tech.md) to learn more about the core technology.
