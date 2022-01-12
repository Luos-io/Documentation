# Test your hardware configuration

To adapt Luos to your board specificities, you need to configure it. This subject is covered in the [Luos configuration](/hardware-consideration/mcu.md).
To validate your network hardware configuration, you can use a Luos tool called selftest available in the <a href="https://github.com/Luos-io/Luos" target="_blank">Luos Library<IconExternalLink width="10" /></a>.

## Hardware test conditions

In order to allow selftest to check everything, you need to have specific conditions for your board.
To make this test, you need to have only one board isolated (unconnected) from the network.
Tx and Rx pins must be connected together, or you have to be able to receive what you send. This is already the case in One Wire and RS485 configurations.
You must also connect the PTP pins together.

<div align="center">
    <Image src="/img/selftest_connection.svg" darkSrc="/img/selftest_connection-dark.svg" />
</div>

## Software conditions

To check the Luos access to the physical network, you need to create a specific binary file only containing the selftest program. This program will perform some checkups allowing you to validate your hardware configuration.
To use the selftest tool, you have to define SELFTEST using "-D SELFTEST" GCC build flag.

:::info
If you use platformio IDE, you must add a "-D SELFTEST" directive in the build_flags setting (this can be found in the file platformio.ini).
:::

To finish, you need to add this in your main file:

```AsciiDoc
#include "luos.h"
#include "selftest.h"

void selftest_ok(void)
{
    //selftest pass! your configuration is OK
}

void selftest_nok(void)
{
    //selftest doesn't pass! your configuration is NOK
}


int main(void)
{
    selftest_run(selftest_ok, selftest_nok);
}
```

## Execution and validation

To know if your configuration is OK or not, you need to be able to know in which function (`selftest_ok` or `selftest_nok`) you will finish.
If you have debug capabilities, you can add breakpoints on these functions allowing you to analyse the call stack and precisely diagnose your test.
If not, you need to find a way to retrieve feedback such as a characteristic LED blink, for example.

The Selftest.c file shows you which part of the hardware communication is tested.

:::info
If NOK:

It can be a communication problem:

1. Check your MCU frequency definition in LuosHAL_config and your setup (those must be equals)
2. Check USART configuration (Pinout Rx/Tx, USART number, IRQ, Handler)
3. Check the TIMER configuration (Channel, IRQ, Handler)
4. Check the DMA configuration (Channel, IRQ, Handler)

It can be a PTP problem:

1. Check your Pinout definition and IRQ for you PTP pin
2. Check in dedicated files the handler for IRQ
:::
