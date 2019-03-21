---
layout: default
---
{% include var.md %}

# A general guide to Luos technology

This guide contains all basic notions you will need to use and understand Luos technology.


## Module categories
Luos modules are classified in 6 categories. Each module belongs to at least one of these categories. Understanding your modules categories will help to understand how to combine them in order to achieve what you want.

On each module’s image on the website one or several small symbols are displayed, allowing you to easily see the different categories it belongs to.

|Sensor|Actuator|Communication|
|:-|:-|:-|
|![Actuator](/assets/img/sticker-sensor.png)|![Actuator](/assets/img/sticker-actuator.png)|![Communication](/assets/img/sticker-gate.png)|
| Sensor modules are able to measure physical world environment.|Actuators modules are able to act on the physical world.|Communication modules (also called gates) are able to share your system’s inputs, outputs and configurations outside of your robot, using a JSON API. <br />You can use these modules to control or program your entire robot with any other device (computer, phone, another robot, etc.)|

|Cognition|User interface|Power|
|:-|:-|:-|
|![Cognition](/assets/img/sticker-cognition.png)|![User interface](/assets/img/sticker-interface.png)|![Power](/assets/img/sticker-power.png)|
|Cognition are modules dedicated to execute your code or host your AI.|These modules are built to interact with the user of the machine.|Power modules are able to share their input power source into the Robus wire to feed other modules.|


## Plugging modules together

Luos modules have at least 2 Robus connection port. All connectors are the same, so you can connect any module to any other one using any of those Robus port. Just avoid ta make a loop circuit otherwise you will degrade your communication between modules.

There is a correct side to plug a cable’s connector to a module. The small tab on the connector must face upward to plug correctly, as shown on the following pictures:

|![Wrong side img](/assets/img/plug-no.png)|![Right side img](/assets/img/plug-yes.png)|
|:-|:-|
|Wrong side, the surface is flat|Right side, the tab is visible on the surface|



## Power management

Luos modules can share their power inputs through the Robus connection, allowing you to feed other modules. These modules belong to the **power category**.

In your network, **you can have multiple power category modules**. In this case, **the power module with the highest voltage takes over** and shares its power with other modules.

For example, for a robot using a 12V motor and an USB module: The USB module belongs to the power category, so it can share is 5V into the Robus wires. But you will need 12V for your motor. So you will have to add a 12V AC plug module in your network to supply your motor. In this case, the USB module doesn’t share its power but the AC plug does because 5V < 12V.

Some modules need specific voltage to work. If the Robus network voltage doesn’t match with the modules voltage limit, the modules will switch in *over-voltage* or *under-voltage* mode and disable vulnerable components.


## External communication management

Luos modules can communicate using different kind of technologies and reach devices outside of the robot. These modules belong to the **communication category**.

All those communication modules use the same **JSON data format** to transmit and receive information about your Luos network.

This way, it’s **easy to use** your favorite device and language to *interact and control your device*.

We created an open-source **Python library** managing this JSON API called *Pyluos*. Feel free to use it, copying it, and convert it into your favorite language. We are open to contribution for any languages.


## Naming management

Each Luos module has a name also called alias. When you receive a new module, it will use its **default name**, looking like `type_mod` (eg. `lidar_mod`).

You can **change the name** of a module to something easily remembered or simply convenient for you. The module saves its new name even if you unplug it.

As we don’t want to have multiple modules with the same name, a duplicate name on your system will be automatically renamed with an incrementing number at the end, in the network.

You can go back to the default name by setting a void name (`""`) to a module.