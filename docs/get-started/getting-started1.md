# Part 1: Your first Luos service

This page allows you to build, flash, run, and control your very first Luos code and Luos network.

This Getting started is separated in 3 parts:
 - **The embedded part**: By following this part you will have all the tools you need to easily develop using Luos in your embedded target.
 - **The remote control part**: By following this part you will have all the tools you need to take control and easily test any Luos device.
 - **The network part**: By following this part you will have your very first Luos network.

## The embedded part: Run your first embedded app!

This tutorial shows you how to quickly upload a Luos application on an MCU development kit.

Supported boards are listed below:
- Arduino zero, MKRzero, MKR1000, or any SAMD21-based Arduino board
- STM32L432KC Nucleo

> **Note:** This list will grow longer with time.

### Setup development environment

We will use <a href="https://platformio.org/platformio-ide" target="_blank">PlatformIO &#8599;</a> as development environment.

First, install the free coding editor <a href="https://code.visualstudio.com/" target="_blank">Microsoft Visual Studio Code &#8599;</a> (VSCode). PlatformIO's IDE is built on top of it.

Then, in PlatformIO:
 1. Open VSCode Extension Manager.
 2. Search for the official [PlatformIO IDE](https://marketplace.visualstudio.com/items?itemName=platformio.platformio-ide) extension.
 3. Install PlatformIO IDE.

<p align="center">
  <img src="/img/get-started/install_VSCODE.png" />
</p>

### Clone the project

Clone the *getting started* repository on your computer: 

```bash
git clone https://github.com/Luos-io/getting_started.git
```

If you are not familiar with Git, you can consult <a href="https://git-scm.com/doc" target="_blank">their documentation &#8599;</a>.

### Flash your board depending on which one you have

Open VSCode and click on **Open Folder** in the project explorer on the left, then select a project depending on the board you have chosen. For example, for the STM32L432KC Nucleo, open **L432KC_Nucleo** in the folder explorer then click on **ADD**:

<p align="center">
  <img src="/img/get-started/Open_project2.png" />
</p>

The project folder is opened in the explorer.

> **Note:** Arduino users can select their board on the `platformio.ini` file by modifying the `board = zero` line.

You can now flash your board: make sure it's connected to your PC with a USB cable and click on **Upload** on the bottom left of the VSCode window:

<p align="center">
  <img src="/img/get-started/Flash_board2.png" />
</p>

PlatformIO will build the firmware and flash it. Take a look at the terminal to watch each step platformIO follows and a **Success** message at the end. If you have any trouble with your USB driver, you can consult [this FAQ page](/faq/dfu). Once the board is programmed, you should see the **LED** blinking on your board.

Congratulations, your first Luos app is running!

### What is going on

There are two [**services**](/luos-technology/services/services.md) loaded in your board allowing to blink the LED.

- **Blinker** sends a message at a fixed duration<br/> ╰ located at the root of the *getting_started* repository (because the same app can run on any board)
- **Led** receives this message and makes the LED blink<br/> ╰ located on the *lib* folder of your project (because it is a driver which is specific to your board)

On top of it, we also added two other services allowing you to take control of your board:

- **Pipe**, managing a serial interface<br/>╰ located on the *lib* folder of your project (because it is a driver which is specific to your board)
- **Gate**, an app that translates Luos to JSON and sends it through *Pipe*<br/> ╰ located in the cloud, (because it is a common cross-platform's Luos app) PlatformIO just downloaded it for you.

Your embedded development environment is now completely installed and you can run Luos services on your MCU.
