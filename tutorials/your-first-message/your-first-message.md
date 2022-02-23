---
hide_table_of_contents: true
custom_edit_url: null
---

# Your First Message

### Estimated difficulty level

Medium

### Total estimated time

1 hour

## 1. Description

Luos Engine aims to exchange information between services by sending messages. In this tutorial, you will learn messages' structure and how to send one to another service.

## 2. Level guidelines

### Pre-requisite:

- [Luos _Get started_](/get-started/get-started)
- [Training repository](https://github.com/Luos-io/Training)
- [Luos Basics](/docs/luos-technology/basics/basics)
- [Your first service tutorial](/tutorials/your-first-service/your-first-service)

### Equipment you will need:

Choose your development board from the list in the _Get started_ tutorial:

- **[Arduino zero](https://www.arduino.cc/en/Main/ArduinoBoardZero&)**, **[MKRzero](https://store.arduino.cc/products/arduino-mkr-zero-i2s-bus-sd-for-sound-music-digital-audio-data)**, **[MKR1000](https://store.arduino.cc/collections/boards/products/arduino-mkr1000-wifi)**, or any **[SAMD21-based](https://en.wikipedia.org/wiki/List_of_Arduino_boards_and_compatible_systems)** Arduino board
- **[STM32L432KC Nucleo](https://www.st.com/en/evaluation-tools/nucleo-l432kc.html)**
- **[STM32F072RB Nucleo](https://www.st.com/en/evaluation-tools/nucleo-f072rb.html)**
- **[STM32F401RE Nucleo](https://www.st.com/en/evaluation-tools/nucleo-f401re.html)**
- **[STM32F410RB Nucleo](https://www.st.com/en/evaluation-tools/nucleo-f410rb.html)**
- **[STM32G431KB Nucleo](https://www.st.com/en/evaluation-tools/nucleo-g431kb.html)**

:::caution
If you are using an Arduino board other than MKRzero, you will have to modify the line board= in the file plateformio.ini from the folder 1_First_Service/Work_base/Arduino to match your board reference.
:::

## 3. Summary

[Part 1: Receiving a message](/your-first-message/receiving-message)

[Part 2: Send a message from a button service](/your-first-message/sned-message)

## 4. Keywords

- <span className="cust_tooltip">Service<span className="cust_tooltiptext">Software element run by Luos that can communicate with other services. It can be a driver or an app.</span></span>
- Luos message
- Polling
- Message handler

## 5. Resources

- [GitHub](https://github.com/Luos-io/)
