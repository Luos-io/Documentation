
# Luos Hardware Abstraction Layer

## Luos Library, Robus communication protocol, physical bus

Luos can work on a single node or creates a network for the communication between containers located on different nodes. This communication should be define and hardware configured to fit with the MCU chosen. luos_hal.c et luos_hal.h define all function need by luos library to send message through the bus and create the initialization of all hardware MCU composant.
luos_hal_config.h files is a default config for a MCU family and can be redefine in your project to fit with your design.

## Luos hal
List all the function using Hardware composant relative to protocol communication and physical bus

- **CRC**: Validate messages. CRC can be generate by hardware or software
- **PORT**: Define necessary pin, PTP lines, Rx/Tx, enable/disable 
- **FLASH**: Storing alias of container in the system
- **TIMER**: Define timeout of the communication
- **COM**: serial bus

## Luos hal config
Define a default configuration pinout, serial bus for a MCU Family.

Luos cover a lot of <a href="https://github.com/Luos-io/LuosHAL" target="_blank">Hardware Abstraction Layer for MCU Family</a>, choose the right one for your MCU.
To match pinout and fonctionality with you design you can create a files "hardware_config.h" in main.c for example and include the file in your project before including luos.h
```C
#include "hardware_config.h"
#include "luos.h"

int main(void)
{
    Luos_Init();
    Container_Init();
    while(1)
    {
        Luos_Loop();
        Container_Loop();
    }
    return 0;
}

```
This way your can predefine hardware configuration for Luos in the file hardware_config.h.
For Example using USART3 in your design instead of USART1 define by luos_hal_config.h: 
```C
#define COM_TX_PIN                  10
#define COM_TX_PORT                 GPIOA
#define COM_TX_AF                   GPIO_AF1_USART3

#define COM_RX_PIN                  11
#define COM_RX_PORT                 GPIOA
#define COM_RX_AF                   GPIO_AF1_USART3


#define LUOS_COM_CLOCK_ENABLE()     __HAL_RCC_USART3_CLK_ENABLE()
#define LUOS_COM                    USART3
#define LUOS_COM_IRQ                USART3_IRQn
#define LUOS_COM_IRQHANDLER()       USART3_IRQHandler()

```

There is many configuration possible and config that you can change not all of them are necessary for your design

| Function | Description | Relative
| :---: | :---: | :---: |
| PORT_CLOCK_ENABLE | Activate clock for GPIO | Depends on port |
| RX_EN_PIN/TX_EN_PIN | Choose pinout to activate Rx/Tx com | Necessary for special com |
| COM_LVL_DOWN_PIN/COM_LVL_UP_PIN | Choose pinout to pull com | Necessary for special com |
| COM_TX_PIN/COM_RX_PIN | Choose pinout for Rx/Tx com | Adapt to the serial bus chosen |
| PTPX_PIN/PTPX_IRQ/PINOUT_IRQHANDLER | Choose pinout, IRQ and callback for PTP line | Necessary for topology detection |
| LUOS_COM_CLOCK_ENABLE | Activate clock for serial | Depends on serial bus |
| LUOS_COM/LUOS_COM_IRQ/LUOS_COM_IRQHANDLER | Choose serial bus, IRQ and callback| Adapt to the serial bus chosen |
| LUOS_TIMER_LOCK_ENABLE| Activate clock for Timeout| Necessary for Timeout |
| LUOS_TIMER/LUOS_TIMER_IRQ/LUOS_TIMER_IRQHANDLER | Choose Timer, IRQ and callback| Necessary for Timeout |
| PAGE_SIZE/ADRESS_LAST_PAGE | Define space in flash for alias| Necessary to rename container alias |

