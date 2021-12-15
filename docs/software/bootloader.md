---
custom_edit_url: null
---

import Image from '/src/components/Images.js';

# A Luos bootloader guide

The bootloader feature allows updating the firmware of any node in a Luos network. It's useful for quickly upgrading your application software without using specific programming tools and without physically accessing your boards.

This page first explains how the bootloader works from a high-level perspective, then how to use it on already supported targets (STM32 L4/F4/G4/F0, ATSAMD21).

A third section details the requested changes to make a Luos application compatible with this bootloader.

## How It Works

The bootloader feature consists of three elements:

- A CLI on your computer, used to monitor the network and manage the update
- A gate service connected to your computer that is able to convey commands from the CLI to the Luos network
- A bootloader code, which is flashed in each node in the network

<div align="center">
  <Image src="/img/bootloader_archi.png"/>
</div>

When you want to update the firmware of node 2 (for example), the CLI tool sends commands through JSON files to the gate, which converts them into Luos commands. During the update, if the node needs to send information to the CLI tool, it sends information to the gate, converting it into JSON files.

The feature has been designed to be as simple as possible, and the process can be described by only a few steps:

1. The CLI reboots the entire Luos network (except the gate) on bootloader mode, so all nodes are ready to receive and save a new firmware
2. The CLI sends binary files to the nodes you want to update
3. The CLI checks the CRC of the binary files saved by the nodes
4. The CLI reboots the entire network in application mode.

From the user's point of view, the CLI only passes through these four steps.

## How To Use It

Let's assume that you already have a network with a bootloader flashed in each node, an application running on top of the bootloader, and a gate allowing you to connect the Luos network to a computer. How can you send a new binary file to node 2 (for example)? All you need to know is how to use the CLI tool. First of all, you can use a command to detect each node in the network:

```bash
pyluos-bootloader detect <SERIAL_PORT>
```

In your case, you use COM3:

```c
pyluos-bootloader detect COM3
```

This leads to the following result:

<div align="center">
  <Image src="/img/boot_detect.png"/>
</div>

We can see two nodes in our network:

- Node 1 containing the gate app
- Node 2 containing one service

Your goal is to update the **button_old** service. To do that, you need to flash a new firmware in **node 2** (we suppose it is called **firmware_new.bin** and that it has been saved in a known location on the user computer). You can then call the following command:

```c
pyluos-bootloader flash <SERIAL_PORT> -t <target1 target2 ...> -b <file.bin>
```

We use the port COM3, our target has the ID n°2, and our binary file is called firmware_new.bin:

```bash
pyluos-bootloader flash COM3 -t 2 -b firmware_new.bin
```

Let's type this command, then you should see the following text on your screen:

<div align="center">
  <Image src="/img/flash_new.png"/>
</div>
You can see the four steps described in the previous section, plus a few log information. First, the CLI prints the parameters used to program the network:

- The gate id used to access the network ( this option is not active yet )
- The list of target nodes to program
- The binary file used to program nodes
- The port on which the gate is connected to the user computer

> **Note**: there is a default parameter for the target list: if nothing is set, the node with the id n°2 (first node after the gate) is flashed.

> **Note**: If you need help to use the tool, you can type the following command:

```bash
pyluos-bootloader flash --help
```

After the CLI launches the programming process, you can notice that the CLI checks if the node is ready (or alive) before programming it. Once the process is finished, you can re-run the network detection and see the following:

<div align="center">
  <Image src="/img/detect_new.png"/>
</div>

You can program more than one node by giving an ID list with the option -t :

```c
pyluos-bootloader flash COM3 -t 2 3 4 -b firmware_new.bin
```

Here we will program nodes with ID n°2, 3, 4.

## Troubleshooting

If, for some reason, you lost the connection with the network or a node during the update, the bootloader allows you to re-run the process without the need to use specific programming tools (such as a JTAG debugger). Let's see what happens if you lost the connection during the update:

<div align="center">
  <Image src="/img/flash_error.png"/>
</div>

The CLI tells you that you have lost the connection. Now by powering off and on your network and re-running a detection, you should see the following:

<div align="center">
  <Image src="/img/detect_boot_service.png"/>
</div>

The **boot_service** tells the node is in bootloader mode. You just have to re-run the flashing process with the CLI:

```bash
pyluos-bootloader flash COM3 -b firmware_new.bin
```

> **Note**: no matter what problem you encounter during the loading process, you have to power-off / power-on your network to see all the nodes running in bootloader mode. Once you get there, you have to use **pyluos-bootloader detect** / **flash** tools to load applications and make it work fine.

## How to add the bootloader feature in your project

### CLI tool

The **pyluos-bootloader** CLI is included in pyluos since v2.0.0. So it's packed with the package when you install it with `pip`.

### Gate

Since Luos v2.0.0, the gate application handles bootloader commands.

### Bootloader binary

The bootloader runs in its own flash partition, which is completely isolated from the application. You have to flash it on your target before you can use the feature. Depending on your hardware, you can use an already available project or make your own bootloader:

#### Luos already supports your target:

The Luos bootloader is available for the following targets:

- STM32 F0 / F4 / G4 / L4 families
- Microchip ATSAMD21J18

Projects for each of these targets can be found in this repo: [https://github.com/ncros3/Luos_bootloader.git](https://github.com/ncros3/Luos_bootloader.git). You can clone this repo and use projects for your application, or use them as examples to build your own bootloader for your specific target.

> **Note**: Examples are available for several IDEs: L4 / F4 / F0 uses platformIO, G4 uses SW4STM32 (an Eclipse-based IDE) and SAMD21 uses MPLAB.

#### Luos does not yet support your target:

First of all, you have to **enable the bootloader feature** in the Luos library. To do so, you have to add the **-D BOOTLOADER_CONFIG** parameter when you invoke your compiler. Then you have to run the library in your main() function as you would do for any project:

<div align="center">
  <Image src="/img/main_bootloader.png"/>
</div>

> **Warning**: Luos will now run the bootloader application. Be careful not to initialize any package with the **ADD_PACKAGE()** macro. Luos can only run the bootloader app in bootloader mode, and your package will not be executed.

Now you have to adjust your linker settings: your bootloader has to reserve a portion of the flash, and the remaining memory will be dedicated to the Luos application. You can find the memory layout of the flash resumed in the following picture:

<div align="center">
  <Image src="/img/memory_layout.png"/>
</div>

This figure shows a third section called **shared_flash**, which exchanges information between the bootloader and the application. When you want to port the bootloader on a specific target, you have to specify this layout (e.g. the amount of flash you reserve for the bootloader, the shared section, and the application). Here is an example for the STM32L432: We choose to dedicate 48 kB for the bootloader, 2 kB (one flash page) for the shared section, and all remaining memory for the application. The translation in the linker file can be seen here:

<div align="center">
  <Image src="/img/linker_bootloader.png"/>
</div>

### Applications

As for the bootloader, you have to modify your linker file in the application if you want to make it compatible with this feature. Now that we defined the memory layout, the modification is straightforward:

<div align="center">
  <Image src="/img/linker_app.png"/>
</div>

You also have to set up the **VTOR** register to the APP_ADDRESS. This feature exists in most modern ARM CPUs and allows to jump to applications saved at any address in flash. On STM32L4, this register is set in **SystemInit()** function:

```c
#define VECT_TAB_OFFSET  0xC800

void SystemInit(void)
{
    ...
    SCB->VTOR = FLASH_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal FLASH */
  ...
}
```

> **Note**: You can find application examples in [https://github.com/ncros3/luos_bootloader_app.git](https://github.com/ncros3/luos_bootloader_app.git).

### How to deal with no VTOR

Some CPUs don't have a VTOR register (such as all CPUs based on cortex-m0), so you have to emulate one. The more convenient solution consists of using a section in RAM to save the application vector table. Then we have to find a way for the CPU to look to this section when it has to check the vector table (by default, it will always look at the first address of the flash memory).

To do so, we modify the application linker to add a dedicated section in RAM:

<div align="center">
  <Image src="/img/linker_ram1.png"/>
</div>

<div align="center">
  <Image src="/img/linker_ram2.png"/>
</div>

Then, we initialize an empty vector table in this section:

```c
#define RSVD_SECTION ".rsvd.data,\"aw\",%nobits//"
#define _RSVD __attribute__((used, section(RSVD_SECTION)))

static volatile _RSVD uint32_t VectorTable[48];
```

Now we copy the application vector table in RAM in the **main()** function:

```c
for (i = 0; i < 48; i++)
{
    VectorTable[i] = *(__IO uint32_t *)(0x0800C800 + (i << 2));
}

/* Enable the SYSCFG peripheral clock*/
__HAL_RCC_SYSCFG_CLK_ENABLE();
/* Remap SRAM at 0x00000000 */
__HAL_SYSCFG_REMAPMEMORY_SRAM();
```

The last line remaps the CPU to the RAM, allowing the CPU to check the vector table at the first address of RAM instead of FLASH.
