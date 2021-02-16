# Integrating Luos into an electronic board
To create and match with a default reference design, electronic boards must respect some design rules in order to properly work in a Luos network.

## Electronic design
Board examples and electronic sources are available <a href="https://github.com/Luos-io/Examples/tree/master/Projects" target="_blank">on GitHub</a>. You are free to use them as you want.

You can Find the schematic of a Luos ready board call L0 for a quick hardware example(available <a href="https://github.com/Luos-io/Examples/tree/master/Projects/0_electronics_basis/l0" target="_blank">on GitHub</a>).

A Luos-friendly electronic board must contain *at least* the following elements:
 - **1** <a href="https://en.wikipedia.org/wiki/Microcontroller" target="_blank">**MCU**</a> (microcontroller unit): It hosts, as a node, the Luos firmware along with the different <span class="cust_tooltip">containers<span class="cust_tooltiptext">{{container_def}}</span></span> (drivers and apps).
 - **At least 2 connectors**: They allow to link boards together into a Luos network 

## One wire reference design
![](../../../_assets/img/Luos_Network_Interface_OW.png)
(Luos' One wire official connector is: <a href="https://octopart.com/df11-4dp-2ds%2852%29-hirose-261749" target="_blank">*DF11-4DP-2DS*</a>).

## RS485 reference design
![](../../../_assets/img/Luos_Network_Interface_485.png)
(Luos' RS485 official connector is: <a href="https://octopart.com/df11-8dp-2ds%2824%29-hirose-39521447" target="_blank">*DF11-8DP-2DS*</a>).

 ## Compatible MCUs
 Luos manages any type of microcontrollers, but they need to be added manually to the library. If your microcontroller is not managed yet, please contact us:
  - by mail: <a href="mailto:hello@luos.io">hello@luos.io</a>
  - on <a href="https://github.com/Luos-io/Luos/issues/new?assignees=nicolas-rabault&labels=porting&template=porting-request.md&title=%5BMCU+PORTING%5D+" target="_blank">GitHub</a>
