---
layout: post
title: "Boards general use"
categories: 3_prototyping_boards
desc: How to use Luos prototyping boards.
order: 0
wip: 0
---
{% include var.md %}

# General guide to Luos electronic boards

Luos provides simple electronic boards to start prototyping using Luos modular technology.

Luos library has been designed to run on low-cost hardware. It works with all Arm microcontrollers, starting with the smallest and cheapest one: the [Cortex-M0](https://developer.arm.com/ip-products/processors/cortex-m/cortex-m0){:target="_blank"}. 

The prototyping boards are a set or small electronic boards, each one hosting Luos and providing with an electronic function (motor, distance sensor, battery, LED, potentiometer, etc.). These boards can be used to quickly develop an electronic device prototype in order to prove a concept without any knowledge in electronics: prototype boards are connected together with cables, behaviors can be programmed through a [gate](/boards_list/usb) board on a computer, and the device can be tested in a matter of minutes!

## Boards general specifications

Almost every prototyping board is composed of a motherboard and a shield board. The motherboard, called L0, has a {{ node_def }} that hosts Luos. The shield board is added to a L0 to type it with an electronic function.

![](/assets/img/assembly.png){:style="height:200px;"}

> Note: Power category boards don't include L0 motherboard as they provide only with power functions and don't need communication. However. he communication data pass through their connectors to other communicating boards.

Here are the specifications of this motherboard:

 - Board name: L0
 - MCU: STM32f0
 - Dimensions: 20 x 26 mm
 - Supply Voltage: 5 V to 24 V
 - Output Voltage: 5 V
 - Connectors: 2x Robus connectors ([*DF11-8DP-2DS(24)*](https://octopart.com/df11-8dp-2ds%2824%29-hirose-39521447))
 - Sockets: 2x 6 connectors ([826926-3](https://octopart.com/826926-3-te+connectivity-40939547)) 
 - Other Output: 1x micro-USB
 - USB Serial Speed: 1 Mbaud/s

![L0 dimensions](/assets/img/l0-dimensions.png){:style="height:300px;"}

## Boards categories

Luos boards are organized in 6 categories. Each board belongs to at least one of these categories. Understanding every categories will help to understand how to connect the Luos boards together in order to achieve any robotic system you want.

On each board’s image in the website, one or several small colored symbols are displayed allowing you to easily see the different categories it belongs to:

![Luos boards examples](/assets/img/boards_example.png){: .right_img }

**Below is the list of the six categories:**

|<a href="{{ "/" | absolute_url }}tags.html">{{sen_title}}</a>|<a href="{{ "/" | absolute_url }}tags.html">{{act_title}}</a>|<a href="{{ "/" | absolute_url }}tags.html">{{com_title}}</a>|
|:-|:-|:-|
|![{{sen_title}}]({{ "/assets/img/sticker-sensor.png" | absolute_url }})|![{{act_title}}]({{ "/assets/img/sticker-actuation.png" | absolute_url }})|![{{com_title}}]({{ "/assets/img/sticker-communication.png" | absolute_url }})|
|{{sen_desc}}|{{act_desc}}|{{com_desc}}|

|<a href="{{ "/" | absolute_url }}tags.html">{{cog_title}}</a>|<a href="{{ "/" | absolute_url }}tags.html">{{int_title}}</a>|<a href="{{ "/" | absolute_url }}tags.html">{{pow_title}}</a>|
|:-|:-|:-|
|![{{cog_title}}]({{ "/assets/img/sticker-cognition.png" | absolute_url }})|![{{int_title}}]({{ "/assets/img/sticker-interface.png" | absolute_url }})|![{{pow_title}}]({{ "/assets/img/sticker-power.png" | absolute_url }})|
|{{cog_desc}}|{{int_desc}}|{{pow_desc}}|


## <a name="plug"></a>Plugging boards together

Luos boards have at least 2 connection ports. All connectors are the same, so that any board can be connected to another one using any of these ports. Just avoid to make a loop circuit, otherwise you will damage the communication between modules.

There is a correct side to plug a cable’s connector to a board. The small tab on the connector must face upward to plug correctly, as shown on the following pictures:

|![Wrong side img](/assets/img/plug-no.png)|![Right side img](/assets/img/plug-yes.png)|
|:-|:-|
|Wrong side, the upper surface is flat|Right side, the tab is visible on the upper surface|



## Power management

Luos boards can share their power inputs through the network connection, allowing you to feed other boards. These boards belong to the **power category**.
All the Luos boards can manage a voltage between 5V and 24V, up to 7A.

In a Luos network, **you can have multiple power category boards**. In this case, **the power board with the highest voltage takes over** and shares its power with other boards.

For example, for a robot using a 12V motor and an USB board: The USB board belongs to the power category, so it can share its 5V into the network's wires. But you need 12V for your motor, so you will have to add a 12V AC plug board in your network to supply the motor. In this case, the USB board doesn’t share its power, only the AC plug board does, because 5V < 12V.

As you probably know, some component needs specific voltage to work properly. For example, in order to use standard servomotor you have to feed the Luos network with 5V or 7V. If you need to combine 7V and 12V motors in a robotic system, for example, you can manage multiple voltages on the same network using a [power isolator board]({{ "/board/power-isolator" | absolute_url }}).


## External communication management

The boards from the Communication category allow you to easily control a Luos network. These boards host a module called "gate", they can communicate using different kinds of technologies and reach devices outside the robot.<br/>To start using Luos technology, you have to use at least one of these gates to be able to program your machine's behaviors.

The "gate" module's task is to stream the Luos network activity into a standard Json format file, and on the oposite to allow an external device to easily interact with any device in the network.

This way, it’s **easy to use** your favorite device and language to interact and control your device.

We created an open-source **Python library** managing this JSON API called [*Pyluos*]({{ "/pyluos.html" | absolute_url }}). Feel free to use it, copy it, and convert it into your favorite language. We are open to contribution for any programing languages. You can suggest any change or new API on the [Luos' forum](https://forum.luos.io/).

Get [pyluos on github](https://github.com/Luos-Robotics/pyluos).

<blockquote class="warning"><strong>Warning:</strong> All examples codes of this documentation use the pyluos python library and are adapted to be used with Jupyter notebook.</blockquote><br />
