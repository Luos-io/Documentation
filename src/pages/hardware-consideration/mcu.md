# MCU with Luos

## Compatible MCUs
Luos can manage any type of microcontrollers as long as they are first added to the library manually. If your microcontroller is not supported yet, please contact us:

 - by mail: <a href="mailto:hello@luos.io">hello@luos.io</a>
 - on <a href="https://github.com/Luos-io/Luos/issues/new?assignees=nicolas-rabault&labels=porting&template=porting-request.md&title=%5BMCU+PORTING%5D+" target="_blank">GitHub</a>

Check the list of MCU family Luos cover :<a href="https://github.com/Luos-io/LuosHAL" target="_blank">Hardware Abstraction Layers for MCU Families</a>,


## Default Configuration

Luos libraries is made to run on MCU and use hardware peripheral of the MCU to complete communication between services. In order to configure this low-level part, Luos company provides, for many MCU family, a default configuration that can lead to plug and play Luos library with the chosen MCU family. The peripheral configuration is described in files luos_hal_config.h in the repository <a href="https://github.com/Luos-io/LuosHAL" target="_blank">LuosHAL</a>, and can be redefined in the node_config.h file to fit with your design.

## Luos HAL configuration

To match the pinout and functionality with your design, you can create or use the file `node_config.h` (see Luos example)
As you can see on the default configuration, defined in luos_hal_config.h, you are able to define in the file `node_config.h`, in the section "LUOS HAL LIBRARY DEFINITION", parameters like pinout, usart, timer, etc.

In this way you will be able to change the default hardware configuration, and use it by calling it in the preprossessor variable section of your IDE, so as to be taken into consideration for your project.

> **FYI:** Every example provided by Luos has a node_config.h file, which is included by the platformio project's initialization file, called platformio.ini.

Example of PTPA redefinition:

In `luos_hal_config.h` this is defined as followed:
```C
#ifndef PTPA_PIN
#define PTPA_PIN                    GPIO_PIN_8
#endif
```

In `node_config.h` this should be redefined as followed:
```C
#define PTPA_PIN                    GPIO_PIN_11
```

There are many possible configurations that you can change, but it is possible that not all of them are necessary for your design:

### Pinout
| Function | Description | Comments |
| :---: | :---: | :---: |
| PORT_CLOCK_ENABLE | Activates clock for GPIO | Depends on port |
| RX_EN_PIN/TX_EN_PIN | Chooses pinout to activate Rx/Tx comms | Necessary for special comms |
| COM_TX_PIN/COM_RX_PIN | Chooses pinout for Rx/Tx comms | Adapts to the chosen serial bus |
| PTPX_PIN/PTPX_IRQ/PINOUT_IRQHANDLER | Chooses pinout, IRQ and callback for the PTP line | Necessary for topology detection |

### Communication
| Function | Description | Comments |
| :---: | :---: | :---: |
| LUOS_COM_CLOCK_ENABLE | Activates clock for serial | Depends on serial bus |
| LUOS_COM/LUOS_COM_IRQ/LUOS_COM_IRQHANDLER | Chooses serial bus, IRQ and callback | Adapts to the serial bus chosen |
| LUOS_DMA_CLOCK_ENABLE | Activates clock for DMA | Necessary for for Tx |
| LUOS_DMA_CHANNEL | Chooses Channel | Send Tx|

### Timer
| Function | Description | Comments |
| :---: | :---: | :---: |
| LUOS_TIMER_CLOCK_ENABLE | Activates clock for Timeout| Necessary for Timeout |
| LUOS_TIMER/LUOS_TIMER_IRQ/LUOS_TIMER_IRQHANDLER | Chooses Timer, IRQ and callback| Necessary for Timeout |

### Flash
| Function | Description | Comments |
| :---: | :---: | :---: |
| PAGE_SIZE/ADRESS_LAST_PAGE | Defines space in flash for alias | Necessary to rename service alias |
