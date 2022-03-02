---
custom_edit_url: null
---

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

# Luos configuration

## Compatible MCUs

Luos can manage any type of microcontrollers as long as they are first added to the library manually. If your microcontroller is not supported yet, please contact us:

- by mail <a href="mailto:hello@luos.io">hello&#x40;luos.io</a>
- on <a href="https://github.com/Luos-io/Luos/issues/new?assignees=nicolas-rabault&labels=porting&template=porting-request&title=%5BMCU+PORTING%5D+" target="_blank">GitHub</a>

Check the list of MCU family Luos cover :<a href="https://github.com/Luos-io/LuosHAL" target="_blank">Hardware Abstraction Layers for MCU Families</a>,

## Default Configuration

Luos libraries is made to run on MCU and use hardware peripheral of the MCU to complete communication between services. In order to configure this low level part, Luos compagny provide, for many MCU family, a default configuration that can be follow to plug and play luos library with the chosen MCU family. The peripheral configuration is describe in files luos_hal_config.h in the repository <a href="https://github.com/Luos-io/LuosHAL" target="_blank">LuosHAL</a>, and can be redefined in the node_config.h file to fit with your design.

## Luos HAL configuration

To match pinout and fonctionality with your design, you can create or use the file `node_config.h` (see Luos example)
Base on the default configuration define in luos_hal_config.h, you can define in the file `node_config.h`, in the section "LUOS HAL LIBRARY DEFINITION", pinout usart timer etc.

This way you are able to change default hardware configuration so it need to be call in the preprossessor variable section of your IDE to be taken into consideration for your project

> **FYI:** Every example provide by luos has a node_config.h files and includes by the file platformio.ini

For example, redefine PTPA to fit with your design

in`luos_hal_config.h` this is defined as followed:

```c
#ifndef PTPA_PIN
#define PTPA_PIN                    GPIO_PIN_8
#endif
```

in`node_config.h` this should be redefined as followed:

```c
#define PTPA_PIN                    GPIO_PIN_11
```

There are many possible configurations that you can change, not all of them being necessary for your design:

<Tabs>
    <TabItem value="Pinout" label="Pinout">

|              Function               |                    Description                    |             Comments             |
| :---------------------------------: | :-----------------------------------------------: | :------------------------------: |
|          PORT_CLOCK_ENABLE          |             Activates clock for GPIO              |         Depends on port          |
|         RX_EN_PIN/TX_EN_PIN         |      Chooses pinout to activate Rx/Tx comms       |   Necessary for special comms    |
|        COM_TX_PIN/COM_RX_PIN        |          Chooses pinout for Rx/Tx comms           | Adapts to the chosen serial bus  |
| PTPX_PIN/PTPX_IRQ/PINOUT_IRQHANDLER | Chooses pinout, IRQ and callback for the PTP line | Necessary for topology detection |

</TabItem>
<TabItem value="Communication" label="Communication">

|                 Function                  |             Description              |            Comments             |
| :---------------------------------------: | :----------------------------------: | :-----------------------------: |
|           LUOS_COM_CLOCK_ENABLE           |      Activates clock for serial      |      Depends on serial bus      |
| LUOS_COM/LUOS_COM_IRQ/LUOS_COM_IRQHANDLER | Chooses serial bus, IRQ and callback | Adapts to the serial bus chosen |
|           LUOS_DMA_CLOCK_ENABLE           |       Activates clock for DMA        |      Necessary for for Tx       |
|             LUOS_DMA_CHANNEL              |           Chooses Channel            |             Send Tx             |

</TabItem>
<TabItem value="Timer" label="Timer">

|                    Function                     |           Description           |       Comments        |
| :---------------------------------------------: | :-----------------------------: | :-------------------: |
|             LUOS_TIMER_CLOCK_ENABLE             |   Activates clock for Timeout   | Necessary for Timeout |
| LUOS_TIMER/LUOS_TIMER_IRQ/LUOS_TIMER_IRQHANDLER | Chooses Timer, IRQ and callback | Necessary for Timeout |

</TabItem>
<TabItem value="Flash" label="Flash">

|          Function          |           Description            |             Comments              |
| :------------------------: | :------------------------------: | :-------------------------------: |
| PAGE_SIZE/ADRESS_LAST_PAGE | Defines space in flash for alias | Necessary to rename service alias |

</TabItem>

</Tabs>
