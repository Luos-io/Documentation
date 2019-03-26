---
layout: post
title:  "Raspberry Pi module"
date:   2019-03-15 17:48:00 +0100
categories: modules
tags: [communication, cognition, power]
---
{% assign module = "Raspberry-Pi" %}
{% include var.md %}

# How to start with the {{ module }} module

This guide contains all the basic notions you will need to use the {{ module }} module.

<div class="sheet" markdown="1">
<p class="sheet-title" markdown="1">**Name:** {{module}}</p>
<p class="sheet-title" markdown="1">**Type:** {{type}}</p>
<p class="sheet-title" markdown="1">**Image**</p>
<p class="indent" markdown="1"><img height="150" src="/assets/img/{{ module }}-module.png" alt="{{ tag | Capitalize }}"></p>
<p class="sheet-title" markdown="1">**Categories**</p>
<p class="indent" markdown="1">
{% for tag in page.tags %}
  <a href="{{ "/" | absolute_url }}tags.html"><img height="50" src="/assets/img/sticker-{{ tag }}.png" alt="{{ tag | capitalize }}"></a>
{% endfor %}
</p>
</div>

## How to connect and start your {{ module }} motors to your modules

![Dynamixel](/assets/img/dxl-mod-1.jpg)

The Dynamixel module is somehow special because it has a dynamic number of visible modules, depending on the number of motors. If you don’t plug any motor in your module, it will be invisible. If you have 5 motors on your module, you will see 5 modules.

This board creates modules dynamically upon motor detection. So in order to create module, this board has to detect motors, and to detect them they need to have a proper power supply.

Indeed, if you power your Luos network with a 7v supply for 12V-Robotis-motors, the motors won’t reply to the module request and you won’t be able to see any Dynamixel module on your network.

> **Warning:** Always be sure to plug Dynamixel power supply before any other module and before connecting to the computer in order to have a proper startup.

Dynamixel modules don’t belong to the power category. Thus, if you power your motors on the Robotis side, you won’t be able to share this power with others modules.

On the contrary, if you power your Luos network using a power category module, then your modules and your Dynamixel motors will be able to use this power supply.

## How to control {{ module }} module with pyluos

To control the Robotis motor through a Luos Robotics network, use the following commands:

```python
# Makes the motor compliant (True or False):
robot.my_dxl_9.compliant = False
 
# Continuous rotation (True or False):
robot.my_dxl_9.wheel_mode = False
 
# Set the rotation speed:
robot.my_dxl_9.moving_speed = 1024
 
# Give a position:
robot.my_dxl_9.target_position = 150.2
 
# Motor control tool:
robot.my_dxl_9.control()
```
 
To get the measured position, use:

```python
robot.my_dxl_9.position
```
 
It returns an angle value in degrees.

----

## Functions
List of functions of {{module}} module:
 
| **-** | - | - | 

## Variables
List of variables of {{module}} module:

| **-** | - | - | 

## Events
List of events of {{module}} module:

| **-** | - | - | 
