# Integrating Luos into an electronic board
Luos uses <span class="cust_tooltip">Robus<span class="cust_tooltiptext">{{robus_def}}</span></span> to communicate with other boards. To design your board, you have to understand that Robus needs to adapt to your own design.
Electronic boards must respect some design rules in order to properly work in a Luos network.

# Electronic design
Board examples and electronic sources are available <a href="https://github.com/Luos-io/Electronics" target="_blank">on GitHub</a>. You are free to use them as you want.

The basic version of Robus uses RS485, but you can use any Half duplex support allowing to check transmitted data.

Here is the example of the schematic of L0 boards (available <a href="https://github.com/Luos-io/Electronics" target="_blank">on GitHub</a>).

![]({{img_path}}/L0_sch.png)

A Luos-friendly electronic board must contain *at least* the following elements:
 - **1** <a href="https://en.wikipedia.org/wiki/Microcontroller" target="_blank">**MCU**</a> (microcontroller unit): It hosts, as a node, the Luos firmware along with the different <span class="cust_tooltip">modules<span class="cust_tooltiptext">{{module_def}}</span></span> (drivers and apps).
 - **At least 2 connectors**: They allow to link boards together into a Luos network (Luos' official connector is: <a href="https://octopart.com/df11-8dp-2ds%2824%29-hirose-39521447" target="_blank">*DF11-8DP-2DS(24)*</a>).

 ## Compatible MCUs
 Luos manages any type of microcontrollers, but they need to be added manually to the library. If your microcontroller is not managed yet, please contact us:
  - by mail: contact@luos.io
  - on <a href="https://github.com/Luos-io/Pre_luos/issues/new/choose" target="_blank">GitHub</a>

 Here is a non-exhaustive list of Luos-compatible MCUs:

  - STM32F072
  - STM32L4

  **Want to see your favorite MCU on this list? <a href="https://www.luos.io/us/contact/" target="_blank">>> Ask-us to add it!</a>**

  > **Note:** If you are a developer, you can create an issue on 
  <a href="https://github.com/Luos-io/Luos/issues/new?assignees=nicolas-rabault&labels=porting&template=porting-request.md&title=%5BMCU+PORTING%5D+" target="_blank">Luos' GitHub repository</a> to ask for a porting and include the new MCU in this list.

<div class="cust_edit_page"><a href="https://{{gh_path}}/pages/low/electronic-design.md">Edit this page</a></div>
