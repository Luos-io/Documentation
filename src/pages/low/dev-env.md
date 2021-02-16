# Setup your development environment
Before starting developing with Luos, you need to have an operational development environment.
At Luos, we use PlatformIO to share all our examples and to make our lib integration easy, but of course you can use your favorite IDE and integrate our libs by yourself.

## General integration consideration

Luos works as a code library running on nodes. To match Luos library with your hardware, Luos offers a *Hardware Abstraction Layer* for a lot of devices in <span class="cust_tooltip">LuosHAL<span class="cust_tooltiptext">{{luoshal_def}}</span></span>.  

 - <a href="https://github.com/Luos-io/LuosHAL" target="_blank">LuosHAL</a>: This repository gives you a list of family device covers to match Luos library with your hardware.
 - <a href="https://github.com/Luos-io/Luos/tree/master/luos" target="_blank">Luos</a>: This is the main library you will be working with.

To make it work on your environment, you have to:

 - Include Luos lib folders in your project compilation;
 - Select the right LuosHAL from your family device in LuosHAL folder, and include `luos_hal.c`, `luos_hal.h` and `luos_hal_config.h` in your project;
 - Change, if necessary, `luos_hal_config.h` in you project, the default configuration created by Luos (before including `luos.h`) in order to match LuosHAL with your hardware (eg: match pins with your design);
 - Include `luos.h` on your source file.

## Setup a Luos PlatformIO project
<a href="https://platformio.org/" target="_blank">PlatformIO</a> is a cross-platform, cross-architecture, multiple framework, professional tool for embedded systems engineers and for software developers who write applications for embedded products. You can put it as a plug-in in a lot of <a href="https://docs.platformio.org/en/latest/integration/ide/index.html#desktop-ide" target="_blank">different editors</a>.

### Getting Started
 1. Install Platform IO on VSCode by following the instructions on <a href="https://platformio.org/platformio-ide" target="_blank">this page</a>.
 2. Create a new projet on PlatformIO
 3. Add Luos as dependancy and select HAL on your `platformio.ini` file:

```Json
lib_deps = Luos
board = <board name>
```
Replace `<board name>` with the name of the board you're using, eg. `board = l0` for the L0 board.

> *Note:* More information about how Luos libs are managed into PlatformIO is available by <a href="https://community.luos.io/t/how-to-link-luos-with-platformio/303" target="\_blank">following this post on our forum</a>.

### Project examples
Luos shares a lot of <a href="https://github.com/Luos-io/Examples" target="_blank">code examples</a>, feel free to use and modify them as you want.

### Demonstration boards
Luos created [a sets of boards](../demo_boards/boards-list.md) allowing to easily test the technology.

## Setup a Luos Arduino project
<a href="https://www.arduino.cc/" target="_blank">Arduino</a> is open-source electronics platform based on easy-to-use hardware and software. Luos library can be use in Arduino IDE and examples are provides to quickly test on Arduino hardware. For now, only Arduino SAM Board 32-bits ARM cortex-M0+ are compatible with Luos Library (Arduino Zero, Arduino MKR Wifi, Arduino MKR FOX, Arduino MKR WAN, Arduino MKR GSM, Arduino NB, etc).

Create a Luos Networks with arduino board is very easy. Use D0 and D1 for Tx and Rx, D2 and D3 for Rx_En an Tx_En (RS485 configuaration), D5 and D6 for PTPA and PTPB. See [physical bus](./physical-bus.md) page for more informations.

### Getting Started
 1. Install Ardiuno IDE from <a href="https://www.arduino.cc/" target="_blank">Arduino</a> web site
 2. Copy and paste the following URL into the File > Preferences > "Additional Boards Manager" textbox.
 ```Json
https://raw.githubusercontent.com/Luos-io/Arduino_core/main/package_luos_index.json
```
 ![](../../../_assets/img/arduino_board_luos_preferences.png)

 3. Install the Luos adapted Arduino SAMD Library in Boards > "Add board definition" > Search for "Luos" > Install "Luos adapted Arduino SAMD (32-bits ARM Cortex-M0+) Boards" 

 Install the Arduino SAMD Library in Boards > "Add board definition" > Search for "SAMD" > Install Arduino SAMD Library

 ![](../../../_assets/img/arduino_Luos_board.png)

 4. Download Luos Library <a href="https://github.com/Luos-io/Luos/releases" target="_blank">on GitHub</a>. 

 5.	Include Luos Library to your IDE

 ![](../../../_assets/img/arduino_include_library.png)

 6. Use provided Luos example, Compil and upload to your Arduino board

 ![](../../../_assets/img/arduino_Luos_example.png)
