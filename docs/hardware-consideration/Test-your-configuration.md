---
custom_edit_url: null
---

import Image from '/src/components/Images.js';

# Test Your configuration

You can use Luos tools call selftest in Luos Library to perform a check your configuration. check <a href="https://github.com/Luos-io/Luos" target="_blank">Luos<IconExternalLink width="10" /></a> to find the selftest utils folder.
Base on Luos default configuration you can change define value of peripheral and pinout to match you design with Luos Library and test your hardware configuration with
selftest_run fonction.


## Set Hardware

Tx and Rx pin must be connect together that is already the case in One Wire configuration and RS485 configuration
You must connect on your board or on the connector (only for selftest) PTP pin together

<div align="center">
    <Image src="/img/selftest_connection.png" />
</div>


## Set software

To use the selftest tool you have to define a pre-compiler definition SELFTEST. if you use platformio IDE to manage your project you must add in the definition file platformio.ini -D SELFTEST

In your main file

```AsciiDoc
#include "luos.h"
#include "selftest.h"

void selftest_ok(void)
{
    //selftest pass! your configuration is OK
}

void selftest_nok(void)
{
    //selftest don't pass! your configuration is NOK
}


int main(void)
{
    selftest_run(selftest_ok, selftest_nok);
}
```

Selftest.c file show you wich part of the hardware communication is tested.

If NOK :
That can be a communication problem :
1. Check your MCU frequency definition to match what is in LuosHAL_config and your setup
2. Check USART configuration (Pinout RX/Tx, USART number, IRQ, Handler)
3. Check the TIMER configuration (Channel, IRQ, Handler)
4. Check the DMA configuration (Channel, IRQ, Handler)

That can be a PTP problem :
1. Check your Pinout definition and IRQ for you PTP Pin
2. Check in dedicated files the handler for IRQ
