---
custom_edit_url: null
---

import Image from '/src/components/Images.js';
import IconExternalLink from '@theme/IconExternalLink';

# Part 1: Your first Luos service

<div align="center"><iframe className="player_iframe" src="https://www.youtube.com/embed/VcK-LJ-gnDo?feature=oembed" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture; fullscreen" ></iframe></div>

## Summary

1. Introduction
2. Set up development environment
3. Clone the project
4. Build your project
5. Upload to the board
6. Luos Package

## 1. Introduction

This part will guide you through the basic essential tools’ installation and how to use them.

## 2. Set up your development environment

We will use [PlatformIO](https://platformio.org/platformio-ide) as a development environment.

1. First, download and install it following the [PlatformIO installation page](https://platformio.org/install/ide?install=vscode).
2. When it is done, open the VS Code editor.
3. Throught this _Get started_, you will need to enter some command lines in a terminal. You can either use the terminal from VS Code, or use your favorite one.

<div align="center">
  <Image src="/img/get-started/get-started-1.png" darkSrc="/img/get-started/get-started-1.png"/>
</div>
<div align="center">
  <Image src="/img/get-started/get-started-1-1.png" darkSrc="/img/get-started/get-started-1-1.png" />
</div>

## 3. Download the base code

Download and unzip [from this link](https://github.com/Luos-io/Get_started/archive/refs/heads/master.zip) the _Get started_ base code.

:::tip
Alternatively, you can clone this code through **Git** via the terminal, using the following command line:

```
git clone https://github.com/Luos-io/Get_started.git
```

You will need to have [GIT](https://git-scm.com/downloads) installed on your computer to do that. If you are not familiar with Git, you can consult [their documentation](https://git-scm.com/doc).

:::

## 4. Build your project

1. On VS Code, click on **File → Open Folder**. Locate the _Get_started_ folder.
2. Select a project in the folder you just downloaded or cloned from GitHub, depending on the board you use. (For example, for the STM32L432KC Nucleo, select the folder **NUCLEO_L432KC**)
3. Click on **Select Folder**.

<div align="center">
  <Image src="/img/get-started/get-started-1-2.png" darkSrc="/img/get-started/get-started-1-2.png"/>
</div>

:::caution
If VS Code displays the message “Do you trust the authors of the files in the folder?”, you can trust us 😉: check the option “Trust the authors of all files in the parent folder” so it will not pop up anymore, then click the “Yes, I trust the authors” button.

:::

The project folder should now be opened in the PlatformIO explorer. 👍

4. Select the environment corresponding to your demo board:

All the project from the _Get_started_ folder are already configured in the file *plateformio.ini*. If you use an Arduino board, you will have to specify the model of your board in this file.

On the left panel in VS Code, locate and click on the *platformio.ini* file, and type the right board in the line `board = ` (or uncomment the name corresponding to the board you are currently using).

:::info
Arduino users can select their board on the *platformio.ini* file by modifying the `board = zero` line.

<div align="center">
  <Image src="/img/get-started/get-started-1-3.png" darkSrc="/img/get-started/get-started-1-3.png"/>
</div>

:::

5. Build the project

Click on the tick button on the bottom. Building the project creates a binary file of your code that you will upload in the next step. 

<div align="center">
  <Image src="/img/get-started/get-started-1-4.png" darkSrc="/img/get-started/get-started-1-4.png"/>
</div>

:::caution
The build may sometimes output an error message. In that case, you need to update the PlatformIO libraries (click on the PlatformIO ant logo on the left, then click on *Update All* on the left panel as shown on the image below), and build again.

<div align="center">
  <Image src="/img/get-started/get-started-1-5.png" darkSrc="/img/get-started/get-started-1-5.png" />
</div>

:::

## 5. Upload to the board

You can now flash your board: connect it to your PC with a USB cable and upload the code by clicking on the right arrow in the bottom left of the VS Code window, situated next to the build tick button (see image below).

PlatformIO will build the firmware and flash the board. Take a look at the terminal to observe each step thet PlatformIO follows. A success message should appear at the end.

Once the board is flashed, it means it is properly programmed, and you should see the LED blinking on your board.

<div align="center">
  <Image src="/img/get-started/get-started-1-6.png" darkSrc="/img/get-started/get-started-1-6.png"/>
</div>

:::tip
In order to make this step to work, you may need to install the USB driver related to your board on your computer.
Note: Linux users should give USB access to their boards by modifying the `udev rules` access rights. To give such rights to PlatformIO, please follow this [tutorial](https://docs.platformio.org/en/latest/faq.html#platformio-udev-rules) on PlatformIO FAQ section.

:::

:::warning
If you have any trouble with your board’s USB driver, you can consult **[our related troubleshooting page](/faq/dfu)**. Also, Make sure your USB cable is not faulty (it happens... 🤗).

:::

Congratulations, your first Luos app is running!

## 6. Explanation: What has been going on?

Two [services](/docs/luos-technology/services/services) have been loaded in your board on step 5, allowing you to make the LED blink.

- **Blinker** sends a message at a fixed duration.<br /> ╰ located at the root of the _get_started_ repository (because the same app can run on any board)
- **Led** receives this message and makes the LED blink.<br /> ╰ located on the _lib_ folder of your project (because it is a driver which is specific to your board)

On top of it, we also added two other services allowing you to take control of your board:

- **Pipe**, managing a serial interface.<br /> ╰ located on the _lib_ folder of your project (because it is a driver which is specific to your board)
- **Gate**, an app that translates Luos to JSON and sends it through _Pipe_.<br /> ╰ located in the cloud (because it is a common cross-platform’s Luos app), PlatformIO just downloaded it for you.

## Next step

Your development environment is now configured, and the _Get started_ project is installed in a local folder of your choice. The [next part](/get-started/get-started2) of this section will teach you how to run Luos on your MCU and take the control with high level API!
