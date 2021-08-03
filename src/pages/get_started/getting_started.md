# Run your first App

This tutorial shows you how to quicly upload a luos app on a MCU developpement kit.

Supported boards are listed below:
- STM32F072 Discovery
- STM32F401
- STM32L432
- STM32G4

## Setup development environment

Install <a href="https://code.visualstudio.com/" target="_blank">VSCode</a>.

Install PlatformIO plugin <a href="https://platformio.org/platformio-ide" target="_blank">PlatformIO</a>.

## Update your USB driver

You may need to upload you USB driver. The process depends on your OS:

### Windows
You need to install and run *Zadig* by following <a href="https://github.com/profezzorn/ProffieOS/wiki/zadig" target="blank_">this tutorial</a> on github.

### Linux
You need permissions to access the device for non-root users:
```bash
echo 'SUBSYSTEMS=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="df11", GROUP="plugdev", MODE="0666"' > /etc/udev/rules.d/60-luos.rules
```

## Clone project depending on your board


## Flash your board

## Going further

Now your developpement environment is installed and you have a luos app running on your MCU. Check [tutorials](../tutorials/tutorials.md) to learn how to use each feature of luos technology. You can also read the [luos documentation](../luos-technology/luos_tech.md) to learn more about the core technology.