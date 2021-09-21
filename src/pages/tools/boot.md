# Bootloader

The bootloader feature allows updating the firmware of any node in a Luos network. It's useful for quickly upgrading your application software without using specific programming tools or physically accessing your boards.

This page first explains how the bootloader works from a high-level perspective, and how to use it on already supported targets (STM32 L4/F4/G4/F0, ATSAMD21).

A third section details the requested changes to make a Luos application compatible with this bootloader.

## How it works

The bootloader feature consists of three elements:

- A CLI on your computer, used to monitor the network and manage the update
- A gate service connected to your computer that is able to convey commands from the CLI to the Luos network
- A bootloader code, which is flashed in each node in the network

<p align="center">
    <img src="../../_assets/img/bootloader_archi.png" />
</p>

When you want to update the firmware of *node 2* (for example), the CLI tool sends commands through JSON files to the gate, which converts them into Luos commands. During the update, if the node needs to send information to the CLI tool, it sends information to the gate, converting it into JSON files.

The feature has been designed to be as simple as possible, and only four steps can describe the process:

1. The CLI reboots the entire Luos network (except the gate) on bootloader mode, so that all nodes are ready to receive and save a new firmware.
2. The CLI sends binary files to the nodes you want to update.
3. The CLI checks the CRC of the binary files saved by the nodes.
4. The CLI reboots the entire network in application mode.

From the user's point of view, the CLI only passes through these four steps.

## How to use it

In a network with a bootloader flashed in each node, an application running on top of the bootloader, and a gate allowing you to connect the Luos network to a computer, you can send a new binary file to *node 2* (for example) by using the CLI tool. First, you can use a command to detect each node in the network:

```bash
pyluos-bootloader detect <SERIAL_PORT>
```

In this example, you will use COM3:

```c
pyluos-bootloader detect COM3
```

This leads to the following result:

<p align="center">
    <img src="../../_assets/img/boot_detect.png" />
</p>

We can see two nodes in the network:

- *node 1* containing the gate app
- *node 2* containing one service

Your goal is to update the **button_old** service. To do that, you need to flash a new firmware in ***node 2*** (we suppose it is called **firmware_new.bin** and has been saved in a known location on the user's computer). You can then call the following command:

```c
pyluos-bootloader flash <SERIAL_PORT> -t <target1 target2 ...> -b <file.bin>
```

We use the port COM3, our target has the ID's number 2, and our binary file is called *firmware_new.bin*:

```bash
pyluos-bootloader flash COM3 -t 2 -b firmware_new.bin
```

Type this command to see the following text on your screen:

<p align="center">
    <img src="../../_assets/img/flash_new.png" />
</p>

You can see the four steps described in the previous section, plus a few log information. First, the CLI prints the parameters used to program the network:

- The gate id used to access the network (this option is not active yet).
- The list of target nodes to program.
- The binary file used to program nodes.
- The port on which the gate is connected to the user's computer.

> **Note:** There is a default parameter for the target list: if nothing is set, the node with the ID's number 2 (first node after the gate) is flashed.

> **Note:** If you need help to use the tool, you can type the following command:
>
>```bash
>pyluos-bootloader flash --help
>```

After the CLI launches the programming process, it checks if the node is ready (or alive) before programming it. Once the process is finished, you can re-run the network detection and see the following:

<p align="center">
    <img src="../../_assets/img/detect_new.png" />
</p>

You can program more than one node by giving an ID list with the option -t:

```c
pyluos-bootloader flash COM3 -t 2 3 4 -b firmware_new.bin
```

In this example, we programmed nodes with ID numbers 2, 3, and 4.

## Troubleshooting

If the connection with the network or with a node is lost during the update, the bootloader allows you to re-run the process without the need to use specific programming tools (such as a JTAG debugger). The following image shows what happens in case of a loss of connection during the update:

<p align="center">
    <img src="../../_assets/img/flash_error.png" />
</p>

The CLI tells you that you have lost the connection. Now by powering off and on your network and re-running a detection, you should see the following:

<p align="center">
    <img src="../../_assets/img/detect_boot_service.png" />
</p>

The **boot_service** tells the node is in bootloader mode. You have to re-run the flashing process with the CLI:

```bash
pyluos-bootloader flash COM3 -b firmware_new.bin
```

> **Note:** No matter what problem you encounter during the loading process, you must power off and power back on your network to see all the nodes running in bootloader mode. Once it is done, you have to use **pyluos-bootloader detect** / **flash** tools to load the applications and make it work again.

## How to add the bootloader feature in your project

### CLI tool

The **pyluos-bootloader** CLI is included in pyluos since v2.0.0, packed with the package when installed with `pip`.

### Gate

Since Luos v2.0.0, the gate application handles bootloader commands.

### Bootloader binary

The bootloader runs in its own flash partition, which is totally isolated from the application. You have to flash it on your target before you can use the feature. Depending on your hardware, you can use an already available project or make your own bootloader:

#### Luos already supports your target:

The Luos bootloader is available for the following targets:

- STM32 F0 / F4 / G4 / L4 families
- Microchip ATSAMD21J18

Several projects for each of these targets can be found in this repository: [https://github.com/ncros3/Luos_bootloader.git](https://github.com/ncros3/Luos_bootloader.git). You can clone this repository and use the projects for your application or use them as examples to build your own bootloader for your specific target.

> **Note:** Examples are available for several IDEs: L4 / F4 / F0 use platformIO, G4 uses SW4STM32 (an Eclipse-based IDE) and SAMD21 uses MPLAB.

#### Luos does not support your target yet:

First, you have to **enable the bootloader feature** in the Luos library. To do so, you must add the **-D BOOTLOADER_CONFIG** parameter when you invoke your compiler. Then you have to run the library in your **main()** function as you would do for any project:

<p align="center">
    <img src="../../_assets/img/main_bootloader.png" />
</p>

> **Warning:** Luos will now run the bootloader application. Be careful not to initialize any package with the **ADD_PACKAGE()** macro. Luos can only run the bootloader app in bootloader mode, and your package will not be executed.

Now you have to adjust the linker settings: your bootloader has to reserve a portion of the flash, and the remaining memory will be dedicated to the Luos application. You can find the memory layout of the flash summarized in the following picture:

<p align="center">
    <img src="../../_assets/img/memory_layout.png" />
</p>

This figure shows a third section called **shared_flash**, which exchanges information between the bootloader and the application. When you want to port the bootloader on a specific target, you have to specify this layout (e.g. the amount of flash you save for the bootloader, the shared section, and the application). Here is an example for the STM32L432: We choose to dedicate 48 kB for the bootloader, 2 kB (one flash page) for the shared section, and all remaining memory for the application. The translation in the linker file can be seen here:

<p align="center">
    <img src="../../_assets/img/linker_bootloader.png" />
</p>

### Applications

As for the bootloader, you have to modify the linker file in the application to make it compatible with this feature. Now that we defined the memory layout, the modification is straightforward:

<p align="center">
    <img src="../../_assets/img/linker_app.png" />
</p>

You also have to set up the **VTOR** register to the APP_ADDRESS. This feature exists in most modern ARM CPUs and allows to jump to applications that are saved at any address in flash. On STM32L4, this register is set in the **SystemInit()** function:

```c
#define VECT_TAB_OFFSET  0xC800

void SystemInit(void)
{
    ...
    SCB->VTOR = FLASH_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal FLASH */
  ...
}
```

> **Note:** You can find application examples in [https://github.com/ncros3/luos_bootloader_app.git](https://github.com/ncros3/luos_bootloader_app.git).

### How to deal with no VTOR

Some CPUs don't have a VTOR register (such as all the CPUs based on cortex-m0), so you will have to emulate one. The most convenient solution consists in using a section in RAM to save the application's vectors table. Then you have to find a way for the CPU to look to this section when it has to check the vectors table (by default, it will always look at the first address of the flash memory).

To do so, we modify the application linker to add a dedicated section in RAM:

<p align="center">
    <img src="../../_assets/img/linker_ram1.png" />
</p>

<p align="center">
    <img src="../../_assets/img/linker_ram2.png" />
</p>

Then, we initialize an empty vector table in this section:

```c
#define RSVD_SECTION ".rsvd.data,\"aw\",%nobits//"
#define _RSVD __attribute__((used, section(RSVD_SECTION)))

static volatile _RSVD uint32_t VectorTable[48];
```

Now we can copy the application vector table in RAM in the **main()** function:

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
