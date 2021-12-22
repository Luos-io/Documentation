---
custom_edit_url: null
---

import Image from '/src/components/Images.js';
import IconExternalLink from '@theme/IconExternalLink';

# Part 1: Your first Luos service

<div align="center"><iframe className="player_iframe" src="https://www.youtube.com/embed/VcK-LJ-gnDo?feature=oembed" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture; fullscreen" ></iframe></div>

## The embedded part: Run your first embedded app!

This tutorial shows you how to quickly upload a Luos application on a Microcontroller Unit (MCU) development kit.

Supported boards are listed below:

- <a href="https://www.arduino.cc/en/Main/ArduinoBoardZero&" target="_blank">Arduino zero<IconExternalLink width="10" /></a>, <a href="https://store.arduino.cc/products/arduino-mkr-zero-i2s-bus-sd-for-sound-music-digital-audio-data" target="_blank">MKRzero<IconExternalLink width="10" /></a>, <a href="https://store.arduino.cc/collections/boards/products/arduino-mkr1000-wifi" target="_blank">MKR1000<IconExternalLink width="10" /></a>, or any <a href="https://en.wikipedia.org/wiki/List_of_Arduino_boards_and_compatible_systems" target="_blank">SAMD21-based<IconExternalLink width="10" /></a> Arduino board
- <a href="https://www.st.com/en/evaluation-tools/nucleo-l432kc.html" target="_blank">STM32L432KC Nucleo<IconExternalLink width="10" /></a>
- <a href="https://www.st.com/en/evaluation-tools/nucleo-f072rb.html" target="_blank">STM32F072RB Nucleo<IconExternalLink width="10" /></a>
- <a href="https://www.st.com/en/evaluation-tools/nucleo-f401re.html" target="_blank">STM32F401RE Nucleo<IconExternalLink width="10" /></a>
- <a href="https://www.st.com/en/evaluation-tools/nucleo-f410rb.html" target="_blank">STM32F410RB Nucleo<IconExternalLink width="10" /></a>
- <a href="https://www.st.com/en/evaluation-tools/nucleo-g431kb.html" target="_blank">STM32G431KB Nucleo<IconExternalLink width="10" /></a>

:::info
This list will grow longer with time.
:::

### Setup development environment

We will use <a href="https://platformio.org/platformio-ide" target="_blank">PlatformIO<IconExternalLink width="10" /></a> as development environment.

First, download and install the free coding editor <a href="https://code.visualstudio.com/" target="_blank">Microsoft Visual Studio Code<IconExternalLink width="10" /></a> (VSCode). PlatformIO's IDE is built on top of it.

Then, in VSCode:

1.  Open VSCode Extension Manager.
2.  Search for the official <a href="https://platformio.org/install/ide?install=vscode" target="_blank">PlatformIO IDE<IconExternalLink width="10" /></a> extension.
3.  Install PlatformIO IDE.

<div align="center">
  <Image src="/img/get-started/install_VSCODE.png" darkSrc=''/>
</div>

### Clone the project

There are two ways to clone the _get started_ repository on your computer:

- via the terminal, using the following command line (in this case you will need to have <a href="https://git-scm.com/downloads" target="_blank">GIT<IconExternalLink width="10" /></a> previously installed on your computer)

```bash
git clone https://github.com/Luos-io/Get_started.git
```

- by downloading and unziping the <a href="https://github.com/Luos-io/Get_started/archive/refs/heads/master.zip" target="_blank">Get Started repository<IconExternalLink width="10" /></a> in your chosen directory

If you are not familiar with Git, you can consult <a href="https://git-scm.com/doc" target="_blank">their documentation<IconExternalLink width="10" /></a>.

### Flash your board depending on which one you have

Open VSCode and click on **Open Folder** in the project explorer on the left, then select a project depending on the board you have chosen. For example, for the STM32L432KC Nucleo, open **L432KC_Nucleo** in the folder explorer then click on **ADD**.

If VSCode outputs the following message "Do you trust the authors of the files in the folder?": you can trust us 😉 and check the option "Trust the authors of all files in the parent folder" so it won't popup anymore. Then click the "Yes, I trust the authors" button.

<div align="center">
  <Image src="/img/get-started/Open_project2.png" darkSrc=''/>
</div>

The project folder is opened in the explorer.

:::note Note
Arduino users can select their board on the `platformio.ini` file by modifying the `board = zero` line.
:::

:::note Note
Linux users should give USB access to their boards by modifying the `udev rules` access rights. To give such rights to platformIO, please follow this <a href="https://docs.platformio.org/en/latest//faq.html#platformio-udev-rules" target="_blank">tutorial<IconExternalLink width="10" /></a> on platformio FAQ section.
:::


You can now flash your board: make sure it is connected to your PC with a USB cable, and click on **Upload** on the bottom left in the VSCode window. In order for this step to work, you will need the USB driver related to your board. As an example, here is the <a href="https://www.st.com/en/development-tools/stsw-link009.html" target="_blank">STM32L432K drivers<IconExternalLink width="10" /></a> on the constructor website. If you have any trouble with your USB driver, you can also consult [our troubleshooting page](/faq/dfu) on this topic.

<div align="center">
  <Image src="/img/get-started/Flash_board2.png" darkSrc=''/>
</div>

PlatformIO will build the firmware and flash it. Take a look at the terminal to watch each step platformIO follows and a **Success** message at the end. Once the board is programmed, you should see the **LED** blinking on your board.

Congratulations, your first Luos app is running!

### What is going on

There are two [services](/docs/luos-technology/services/services) loaded in your board allowing to blink the LED.

- **Blinker** sends a message at a fixed duration<br /> ╰ located at the root of the _get_started_ repository (because the same app can run on any board)
- **Led** receives this message and makes the LED blink<br /> ╰ located on the _lib_ folder of your project (because it is a driver which is specific to your board)

On top of it, we also added two other services allowing you to take control of your board:

- **Pipe**, managing a serial interface<br /> ╰ located on the _lib_ folder of your project (because it is a driver which is specific to your board)
- **Gate**, an app that translates Luos to JSON and sends it through _Pipe_<br /> ╰ located in the cloud (because it is a common cross-platform's Luos app), PlatformIO just downloaded it for you.

## Next steps

Your development environment is set up and the project is installed in a local folder. The [next part](/get-started/get-started2) of this section will teach you how to run Luos on your MCU.
