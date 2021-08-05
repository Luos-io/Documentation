# General guide to Luos electronic boards

Luos library has been designed to run on low-cost hardware. It works with all Arm microcontrollers, starting with the smallest and cheapest one: the Cortex-M0.

The demonstration boards are a set of small electronic boards examples, each one hosting Luos and providing with an electronic function (motor, distance sensor, battery, LED, potentiometer, etc.). These boards can be used to test the technology, or to quickly develop an electronic device prototype in order to prove a concept without any knowledge in electronics: demonstration boards are connected together with cables, behaviors can be programmed through a gate board on a computer, and the device can be tested in a matter of minutes!

Luos provides simple electronic boards examples to build in order to demonstrate the Luos modular technology. These examples are available on Github and contain a schematic file and a Kicad file, so that they can be easily reproduced to test Luos.

## Boards general specifications
Almost every demonstration board in the provided examples is composed of a motherboard and a shield board. The motherboard, called L0, has a <span class="cust_tooltip">node<span class="cust_tooltiptext">{{node_def}}</span></span> that hosts Luos. The shield board is added to a L0 to type it with an electronic function.

<img src="../../../_assets/img/assembly.png" height="200px" />

> Note: Power category boards don't include L0 motherboard as they provide only with power functions and don't need communication. However. he communication data pass through their connectors to other communicating boards.

Here are the specifications of this motherboard:

{{ #include ./../../../_includes/specs.md }}


see <a href="https://github.com/Luos-io/Examples/tree/master/Hardware" target="_blank">Example demonstration board</a>.

## Plugging boards together
Luos boards have at least 2 connection ports in their design. All connectors are the same, so that any board can be connected to another one using any of these ports. Just avoid to make a loop circuit, otherwise you will inhibit communication between services.

There is a correct side to plug a cable’s connector to a board. The small tab on the connector must face upward to plug correctly, as shown on the following pictures:

|![Wrong side img](../../../_assets/img/plug-no.png)|![Right side img](../../../_assets/img/plug-yes.png)|
|:-|:-|
|Wrong side, the upper surface is flat|Right side, the tab is visible on the upper surface|

## Power management
Luos boards can share their power inputs through the network connection, allowing you to feed other boards. These boards belong to the **power category**.
All the Luos boards can manage a voltage between 5V and 24V, up to 7A.

In a Luos network, **you can have multiple power category boards**. In this case, **the power board with the highest voltage takes over** and shares its power with other boards.

For example, for a device using a 12V motor and an USB board: The USB board belongs to the power category, so it can share its 5V into the network's wires. But you need 12V for your motor, so you will have to add a 12V AC plug board in your network to supply the motor. In this case, the USB board doesn’t share its power, only the AC plug board does, because 5V < 12V.

Some components needs a specific voltage to work properly. For example, in order to use standard servomotor, you have to feed the Luos network with 5V or 7V. If you need to combine 7V and 12V motors in a system, for example, you can manage multiple voltages on the same network using a power isolator board

## External communication management
The boards from the Communication category allow you to easily control a Luos network. These boards host a service called "gate", they can communicate using different kinds of technologies and reach devices outside the device.<br/>To start using Luos technology, you have to use at least one of these gates to be able to program your machine's behaviors.

The "gate" service's task is to stream the Luos network activity into a standard JSON format file, and on the oposite to allow an external device to easily interact with any device in the network.

This way, it’s **easy to use** your favorite device and language to interact and control your device.

We created an open-source **Python library** managing this JSON API called Pyluos. Feel free to use it, copy it, and convert it into your favorite language. We are open to contribution for any programing languages. You can suggest any change or new API on the <a href="https://community.luos.io/" target="_blank">Luos' forum</a>.

Get <a href="https://github.com/Luos-io/Pyluos" target="_blank">Pyluos on github</a>.
