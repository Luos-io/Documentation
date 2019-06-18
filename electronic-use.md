---
layout: default
title: "Boards general use"
---
{% include var.md %}

# General guide to Luos electronic boards

Luos Robotics provides simple electronic boards to start prototyping using Luos modular technology.
This guide contains all the basic notions you will need to use Luos electronic boards.


## Boards categories

Luos boards are classified in 6 categories. Each board belongs to at least one of these categories. Understanding your board categories will help to understand how to combine them in order to achieve what you want.

On each board’s image on the website, one or several small symbols are displayed allowing you to easily see the different categories it belongs to.


|<a href="{{ "/" | absolute_url }}tags.html">{{sen_title}}</a>|<a href="{{ "/" | absolute_url }}tags.html">{{act_title}}</a>|<a href="{{ "/" | absolute_url }}tags.html">{{com_title}}</a>|
|:-|:-|:-|
|![{{sen_title}}]({{sen_img}})|![{{act_title}}]({{act_img}})|![{{com_title}}]({{com_img}})|
|{{sen_desc}}|{{act_desc}}|{{com_desc}}|

|<a href="{{ "/" | absolute_url }}tags.html">{{cog_title}}</a>|<a href="{{ "/" | absolute_url }}tags.html">{{int_title}}</a>|<a href="{{ "/" | absolute_url }}tags.html">{{pow_title}}</a>|
|:-|:-|:-|
|![{{cog_title}}]({{cog_img}})|![{{int_title}}]({{int_img}})|![{{pow_title}}]({{pow_img}})|
|{{cog_desc}}|{{int_desc}}|{{pow_desc}}|


## Plugging boards together

Luos boards have at least 2 connection ports. All connectors are the same, so you can connect any board to any other one using any of those port. Just avoid to make a loop circuit otherwise you will degrade your communication between modules.

There is a correct side to plug a cable’s connector to a board. The small tab on the connector must face upward to plug correctly, as shown on the following pictures:

|![Wrong side img](/assets/img/plug-no.png)|![Right side img](/assets/img/plug-yes.png)|
|:-|:-|
|Wrong side, the surface is flat|Right side, the tab is visible on the surface|



## Power management

Luos boards can share their power inputs through the network connection, allowing you to feed other boards. These boards belong to the **power category**.
All the Luos boards can manage a voltage between 5V and 24V, up to 7A.

In your network, **you can have multiple power category boards**. In this case, **the power board with the highest voltage takes over** and shares its power with other boards.

For example, for a robot using a 12V motor and an USB board: The USB board belongs to the power category, so it can share its 5V into the network's wires. But you need 12V for your motor, so you will have to add a 12V AC plug board in your network to supply the motor. In this case, the USB board doesn’t share its power, only the AC plug board does, because 5V < 12V.

As you probably know, some component needs specific voltage to work properly. For example, in order to use standard servomotor you have to feed your network with 5v or 7V. If you need to combine 7V and 12V motors in a robotic system, for example, you can manage multiple voltages on the same network using a power isolator board.


## External communication management

Some specific boards allow you to easily control a Luos boards' network. These boards host a module called "gate", they can communicate using different kinds of technologies and reach devices outside of the robot.<br/>To start using Luos technology, you have to use at least one of this module to be able to program your machine's behaviors.

The "gate" module's task is to stream the Luos network activity into a standard Json format file, and on the oposite to allow an external device to easily interact with any device in the network.

This way, it’s **easy to use** your favorite device and language to interact and control your device.

We created an open-source **Python library** managing this JSON API called [*Pyluos*]({{ "/pyluos.html" | absolute_url }}). Feel free to use it, copy it, and convert it into your favorite language. We are open to contribution for any programing languages.

Get [pyluos on github](https://github.com/Luos-Robotics/pyluos).

<blockquote class="warning"><strong>Warning:</strong> All examples codes of this documentation use the pyluos python library and are adapted to be used with Jupyter notebook.</blockquote><br />
