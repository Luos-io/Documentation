# Hardware consideration

When creating a Luos network, if you have a product with several nodes, it is mandatory to configure your MCU, to create the physical layer. In <a href="https://github.com/Luos-io/Examples/" target="_blank">Luos's examples on GitHub &#8599;</a>, Luos uses RS485 with a driver or one-wire communication for this physical layer ([see Electronic design](./electronics.md) page), but other communication ways are possible.

Board examples and driver sources are available <a href="https://github.com/Luos-io/Examples/tree/master/Projects" target="_blank">on GitHub &#8599;</a>. We encourage you to use them in the way you want.

You can find the schematic of a Luos-ready board called L0 <a href="https://github.com/Luos-io/Examples/tree/master/Hardware/l0" target="_blank">on Github &#8599;</a> for a quick hardware example.
