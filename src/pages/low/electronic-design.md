# Integrating Luos into an electronic board
Luos uses a communication protocol call <span class="cust_tooltip">Robus<span class="cust_tooltiptext">{{robus_def}}</span></span> to communicate with other containers. Luos creates a network for the communication between containers located on different nodes. This communication goes through a physical layer, which must be adapted to your own design (through the file `LuosHAL_Config.h`).

Electronic boards must respect some design rules in order to properly work in a Luos network.

## Electronic design
Board examples and electronic sources are available <a href="https://github.com/Luos-io/Examples/tree/master/Projects" target="_blank">on GitHub</a>. You are free to use them as you want.

The basic physical layer to create a Luos network is based on RS485, but you can use any half duplex support allowing to check transmitted data.

Here is the example of the schematic of L0 boards (available <a href="https://github.com/Luos-io/Examples/tree/master/Projects/0_electronics_basis/l0" target="_blank">on GitHub</a>).

![]({{img_path}}/L0_sch.png)

A Luos-friendly electronic board must contain *at least* the following elements:
 - **1** <a href="https://en.wikipedia.org/wiki/Microcontroller" target="_blank">**MCU**</a> (microcontroller unit): It hosts, as a node, the Luos firmware along with the different <span class="cust_tooltip">containers<span class="cust_tooltiptext">{{container_def}}</span></span> (drivers and apps).
 - **At least 2 connectors**: They allow to link boards together into a Luos network (Luos' official connector is: <a href="https://octopart.com/df11-8dp-2ds%2824%29-hirose-39521447" target="_blank">*DF11-8DP-2DS(24)*</a>).

 ## Compatible MCUs
 Luos manages any type of microcontrollers, but they need to be added manually to the library. If your microcontroller is not managed yet, please contact us:
  - by mail: contact@luos.io
  - on <a href="https://github.com/Luos-io/Luos/issues/new?assignees=nicolas-rabault&labels=porting&template=porting-request.md&title=%5BMCU+PORTING%5D+" target="_blank">GitHub</a>


