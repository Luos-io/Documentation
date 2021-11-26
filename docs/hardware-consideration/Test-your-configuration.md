---
custom_edit_url: null
---

import Image from '/src/components/Images.js';

# Test Your configuration

You can use a Luos tool called selftest in Luos Library to check your hardware configuration. Check <a href="https://github.com/Luos-io/Luos" target="_blank">Luos<IconExternalLink width="10" /></a> to find the selftest util folder.
Based on Luos default configuration, you can change #define values of peripherals and pinout to match your design.


## Set Hardware

Tx and Rx pin must be connected together. This is already the case in One Wire configuration and RS485 configuration.
You must connect PTP pin together too.

<div align="center">
    <Image src="/img/selftest_connection.png" />
</div>


## Set software

To use the selftest tool, you have to define the pre-processor definition SELFTEST. If you use platformio IDE to manage your project you must add a "-D SELFTEST" directive in build_flags setting (this can be found in platformio.ini file).

In your main file :

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

Selftest.c file shows you which part of the hardware communication is tested.

If NOK :
It can be a communication problem :
1. Check your MCU frequency definition in LuosHAL_config and your setup (those must be equals)
2. Check USART configuration (Pinout RX/Tx, USART number, IRQ, Handler)
3. Check the TIMER configuration (Channel, IRQ, Handler)
4. Check the DMA configuration (Channel, IRQ, Handler)

It can be a PTP problem :
1. Check your Pinout definition and IRQ for you PTP Pin
2. Check in dedicated files the handler for IRQ
