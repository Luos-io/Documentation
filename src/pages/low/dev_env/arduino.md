# Setup a Luos Arduino project
<a href="https://www.arduino.cc/" target="_blank">Arduino</a> is an open-source electronic platform based on easy-to-use hardware and software. Luos library can be used in Arduino IDE and examples are provided to quickly test it on Arduino hardware. For now, only Arduino SAM Board 32-bits ARM cortex-M0+ are compatible with Luos Library (Arduino Zero, Arduino MKR Wifi, Arduino MKR FOX, Arduino MKR WAN, Arduino MKR GSM, Arduino NB, etc).

Create a Luos Networks with an Arduino board is very easy. Use D0 and D1 for Tx and Rx, D2 and D3 for Rx_En and Tx_En (RS485 configuration), D5 and D6 for PTPA and PTPB. See the [hardware topics](../hardware-topics.md) page for more information.

## Getting Started
 1. Install Ardiuno IDE from <a href="https://www.arduino.cc/" target="_blank">Arduino</a> website.
 2. Copy and paste the following URL into the *File > Preferences > "Additional Boards Manager"* textbox.
 ```Json
https://raw.githubusercontent.com/Luos-io/Arduino_core/main/package_luos_index.json
```
 ![](../../../_assets/img/arduino_board_luos_preferences.png)

 3. Install the Luos adapted Arduino SAMD Library in *Boards > "Add board definition" > Search for "Luos" > Install "Luos adapted Arduino SAMD (32-bits ARM Cortex-M0+) Boards"*

 4. Install the Arduino SAMD Library in *Boards > "Add board definition" > Search for "SAMD" > Install Arduino SAMD Library*:

 ![](../../../_assets/img/arduino_Luos_board.png)

 5. Download Luos Library <a href="https://github.com/Luos-io/Luos/releases" target="_blank">on GitHub</a>. 

 6.	Include Luos Library to your IDE:

 ![](../../../_assets/img/arduino_include_library.png)

 7. Use provided Luos example, then compile and upload it to your Arduino board:

 ![](../../../_assets/img/arduino_Luos_example.png)
