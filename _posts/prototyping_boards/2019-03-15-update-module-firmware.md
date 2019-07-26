---
layout: post
title: "Update firmware"
categories: 3_prototyping_boards
desc: How to update Luos bords' firmware.
order: 2
wip: 0
---
{% include var.md %}

# A guide to upgrade your boards

At Luos Robotics, we work every day on new versions of the boards’ internal software (the modules), in order to improve your experience with robotics. This guide explains step-by-step how to upgrade any board with an up-to-date software provided by Luos.

<blockquote class="warning"><strong>Warning:</strong> Upgrading a module is a critical task that should be attempted only if you know what you are doing.</blockquote><br />

<p class="bigger" markdown="1">Last firmware version: {{ last_version_fw }}</p>

**[\>\> Get the firmware files on *GitHub*.](https://github.com/Luos-Robotics/module_binaries/releases)**

### How to check my module firmware revision
If you are using [pyluos](https://www.luos-robotics.com/en/documentation/pyluos) you can check the firmware revision of modules in your network using the `robot.module.firmware_revision` command.

For example : 

```python
robot.servo1_mod.firmware_revision
```
 
### What you need

#### For Windows:

* A computer with Windows.
* The DFU file manager software from ST
* The [binary file](https://github.com/Luos-Robotics/module_binaries/releases) provided by Luos Robotics
* A micro-USB to USB cable
* The board that needs to be upgraded

**[\>\> Windows steps](#windows)**

#### For Mac and Linux:

* A computer with one of these OS
* The [binary file](https://github.com/Luos-Robotics/module_binaries/releases) provided by Luos Robotics
* A micro-USB to USB cable
* The board that needs to be upgraded

**[\>\> Mac / Linux steps](#unix)**

----

## Windows steps 
{: #windows }

These steps are only for Windows. For Mac or Linux, ignore them and go directly to [Mac / Linux steps](#unix).

### Step 1: Download and install the software
The software can be downloaded **[here](https://www.st.com/en/development-tools/stsw-stm32080.html#getsoftware-scroll)**. You will have to create an account in order to download it.

After the download, install the software on your computer. The files you will need usually figure here on your computer:

`[Windows Installation Disk]:\Program Files (x86)\STMicroelectronics\Software\DfuSe v3.0.6\Bin`

### Step 2: Convert the binary file
Download the binary file at **[this address](https://github.com/Luos-Robotics/module_binaries/releases)**.
Execute the program *DfuFileMgr.exe*.

![DfuFileMgr1](/assets/img/firmwareupdate-1.png)

Click `OK`, and fill the Product ID and Version values with the respective addresses `0xDF11` and `0x2200`, as shown on the next image. Then click on the `Multi BIN…` button.

![DfuFileMgr2](/assets/img/firmwareupdate-2.png)

![DfuFileMgr3](/assets/img/firmwareupdate-3.png)

The Address must be `0x08000000`.

Load the binary file provided by Luos Robotics (button `…`), and click `Add to list >>`, then click `OK`.<br />
Click the `Generate…` button, choose the DFU file’s name and location and save it.

![DfuFileMgr4](/assets/img/firmwareupdate-4.png)

You can close the program.

### Step 3: Connect the board to the computer
Each Luos board has a micro-USB port. Plug the USB cable to this port and then to your computer.

> **Note:** The USB board has two USB ports, you should always plug the port situated on the L0, under the connectors. You should not see any light from the board when you plug it.

### Step 4: Upload the new file into the board
Execute the program *DfuSeDemo.exe*.<br />
Click on `Choose…` in the section `Upgrade or Verify Action` and load the DFU file you just converted from binary.

![Dfuse1](/assets/img/firmwareupdate-5.png)

Click on `Upgrade`. A message may appear. If it does, click `Yes`.

![Dfuse2](/assets/img/firmwareupdate-6.png)

After uploading, you should see a new message in the bottom.<br />
Your board is now ready, you can close the program and unplug the USB form the computer.

![Dfuse3](/assets/img/firmwareupdate-7.png)

----

## Mac / Linux steps 
{: #unix }

These steps are only for Mac or Linux. If you're using Windows, ignore them and go directly to [Windows steps](#windows).

### Step 1: Install the software
Install the program *dfu-util*.

### Step 2: Connect the board to the computer
Each Luyos board has a micro-USB port. Plug the USB cable to this port and then to your computer.

> **Note:** The USB board has two USB ports, you should always plug the port situated on the L0, under the connectors. You should not see any light from the board when you plug it.

### Step 3: Upload the file into the board
Download the binary file at **[this address](https://github.com/Luos-Robotics/module_binaries/releases)**.

Type the following line and replace `module.bin` by the filename of the binary file provided by Luos Robotics:

`dfu-util -d 0483:df11 -a 0 -s 0x08000000 -D module.bin`
 
After executed, your board is ready to be used, you can unplug the USB from the computer.