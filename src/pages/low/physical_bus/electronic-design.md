# Integrating Luos into an electronic board
To create and match with the default reference design, electronic boards must respect some design rules in order to properly work in a Luos network.

## Electronic design
Board examples and electronic sources are available <a href="https://github.com/Luos-io/Examples/tree/master/Projects" target="_blank">on GitHub</a>. You are free to use them as you want.

Here is the example of the schematic of L0 boards (available <a href="https://github.com/Luos-io/Examples/tree/master/Projects/0_electronics_basis/l0" target="_blank">on GitHub</a>).

![](../../../_assets/img/L0_sch.png)

A Luos-friendly electronic board must contain *at least* the following elements:
 - **1** <a href="https://en.wikipedia.org/wiki/Microcontroller" target="_blank">**MCU**</a> (microcontroller unit): It hosts, as a node, the Luos firmware along with the different <span class="cust_tooltip">containers<span class="cust_tooltiptext">{{container_def}}</span></span> (drivers and apps).
 - **At least 2 connectors**: They allow to link boards together into a Luos network (Luos' official connector is: <a href="https://octopart.com/df11-8dp-2ds%2824%29-hirose-39521447" target="_blank">*DF11-8DP-2DS(24)*</a>).

 ## Compatible MCUs
 Luos manages any type of microcontrollers, but they need to be added manually to the library. If your microcontroller is not yet supported, please contact us:
  - by mail: <a href="mailto:hello@luos.io">hello@luos.io</a>
  - on <a href="https://github.com/Luos-io/Luos/issues/new?assignees=nicolas-rabault&labels=porting&template=porting-request.md&title=%5BMCU+PORTING%5D+" target="_blank">GitHub</a>
