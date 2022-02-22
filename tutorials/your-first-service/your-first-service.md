---
hide_table_of_contents: true
custom_edit_url: null
---

# Your First Service

### Estimated difficulty level

Medium

### Total estimated time

1 hour

## 1. Description

You just finished the _Get started_ and you want to go further with Luos? This tutorial is made for you!

In this tutorial, you will learn:

- How to create your first service with Luos Engine step by step
- How to make your project structure ready for a modular architecture

## 2. Level guidelines

### Pre-requisite:

- [Luos Get started](/get-started/get-started)
- [Training repository](https://github.com/Luos-io/Training)

### Equipment you will need

Choose your development board from the list in the _Get started_ tutorial:

- **[Arduino zero](https://www.arduino.cc/en/Main/ArduinoBoardZero&)**, **[MKRzero](https://store.arduino.cc/products/arduino-mkr-zero-i2s-bus-sd-for-sound-music-digital-audio-data)**, **[MKR1000](https://store.arduino.cc/collections/boards/products/arduino-mkr1000-wifi)**, or any **[SAMD21-based](https://en.wikipedia.org/wiki/List_of_Arduino_boards_and_compatible_systems)** Arduino board
- **[STM32L432KC Nucleo](https://www.st.com/en/evaluation-tools/nucleo-l432kc.html)**
- **[STM32F072RB Nucleo](https://www.st.com/en/evaluation-tools/nucleo-f072rb.html)**
- **[STM32F401RE Nucleo](https://www.st.com/en/evaluation-tools/nucleo-f401re.html)**
- **[STM32F410RB Nucleo](https://www.st.com/en/evaluation-tools/nucleo-f410rb.html)**
- **[STM32G431KB Nucleo](https://www.st.com/en/evaluation-tools/nucleo-g431kb.html)**

:::caution
If you are using an Arduino board other than MKRzero, you will have to modify the line `board=` in the file _plateformio.ini_ from the folder `1_First_Service/Work_base/Arduino` to match your board reference.

:::

Additionally, you will need for this tutorial a coding environment with the last versions of [Python](https://www.python.org/), [GIT](https://git-scm.com/), [Ipython](https://ipython.org/), pyluos, and of a USB driver compatible with your board.

## 3. Summary

[Part 1: Luos service](/tutorials/your-first-service/luos-service)

[Part 2: Create a package](/tutorials/your-first-service/create-a-package)

## 4. Keywords

- <span className="cust_tooltip">Service<span className="cust_tooltiptext">Software element run by Luos that can communicate with other services. It can be a driver or an app.</span></span>
- Package
- Handler
- Gate
- Pipe

## 5. Resources

- [GitHub](https://github.com/Luos-io/)
