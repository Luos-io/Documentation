# Cables
<div class="cust_sheet" markdown="1">
<p class="cust_sheet-title" markdown="1"><strong>Default Alias:</strong> N/A</p>
<p class="cust_sheet-title" markdown="1"><strong>Type:</strong> N/A</p>
<p class="cust_sheet-title" markdown="1"><strong>Number of container(s):</strong> N/A</p>
<p class="cust_sheet-title" markdown="1"><strong>Image</strong></p>
<p class="cust_indent" markdown="1"><img height="150" src="{{img_path}}/cable-10cm.png"><img height="150" src="{{img_path}}/cable-20cm.png"></p>
<p class="cust_sheet-title" markdown="1"><strong>Category(-ies)</strong></p>
<p class="cust_indent" markdown="1">N/A
</p>
<p class="cust_sheet-title" markdown="1"><strong>Project source </strong></p>
<a class="github-button" data-size="large" aria-label="Star Luos-io/Luos on GitHub" href="https://github.com/Luos-io/Examples/tree/master/Projects/0_electronics_basis/" target="_blank">Cables</a>
</div>

## How do the cables work?

The cables are used to link the Luos bords together. There are two lengths of cables : 10 cm (3.9 in) cables and 20 cm (7.9 in) cables.
However, it is possible to build a cable from any disired length (see the next section).

**Maximal current value:** The Luos cable can handle up to 7 A.

**Boards connection:** The connectors on the board side and on the cable side have a foolproof so that they can plug together in one way only. Fore more information about plugging boards together with cables, please follow [this link](/pages/demo_boards/electronic-use.md#plug).

## How to buid a Luos compatible-cable?

If you need a cable with a length not available, you can build one, provided you have electrical wire and two connectors:

- Electrical wire: `AWG 22`
- Connectors: `DF11-8DS-2C, 2mm`

<a href="https://datasheet.octopart.com/DF11-8DS-2C-Hirose-datasheet-15540170.pdf" target="_blank">The datasheet connector is available here</a>.

The board connector associated to the cables is `DF11-8DP-2DS`. this connector's pinout on Luos' boards is shown in the following picture:

![Luos board connector pinout]({{img_path}}/pinout-board-connector.png)

The routing of the board connector is as shown below:

![Luos board connector routing]({{img_path}}/board-large-view.png)

[![Luos board connector routing]({{img_path}}/routing-board-connector-small.png)]({{img_path}}/routing-board-connector.png)
<br />*Click on the image to display it in full size.*

On a Luos cable, the wires are organized in this order:

- Pin 1 (first connector end) on pin 1 (second connector end)
- Pin 2 (first connector end) on pin 2 (second connector end)
- Pin 3 (first connector end) on pin 3 (second connector end)
- ...
- Pin 8 (first connector end) on pin 8 (second connector end)

> **Note:** If you wish to use only 4 wires instead of 8 (data only, no power), this is possible with using the pins 4 (`B_RS_485_P`), 5 (`A_RS_485_N`), 3 or 6 (`PTP`), and `GND`. 


