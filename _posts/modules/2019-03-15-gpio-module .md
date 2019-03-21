---
layout: post
title:  "GPIO module"
date:   2019-03-15 17:54:00 +0100
categories: modules
tags: [Actuation, Sensor]
---
{% include var.md %}
{% assign module = "GPIO" %}

# How to start with the {{ module }} module

This guide contains all the basic notions you will need to use the {{ module }} module.

<div class="sheet" markdown="1">

<p class="sheet-title" markdown="1">**Name**</p>

<p class="indent" markdown="1">{{module}}</p>

<p class="sheet-title" markdown="1">**Type**</p>

<p class="indent" markdown="1">`GPIO`</p>

<p class="sheet-title" markdown="1">**Image**</p>

<p class="indent" markdown="1">![{{ module }} module](/assets/img/gpio-module.png)</p>

<p class="sheet-title" markdown="1">**Categories**</p>

<p class="indent" markdown="1">
{% for tag in page.tags %}
  <a href="{{ "/" | absolute_url }}tags.html">{{ tag }}</a>
{% endfor %}
</p>
</div>


## Module categories

|<a href="{{ "/" | absolute_url }}tags.html">{{act_title}}</a>|<a href="{{ "/" | absolute_url }}tags.html">{{sen_title}}</a>|
|:-|:-|
|![{{act_title}}]({{act_img}})|![{{sen_title}}]({{sen_img}})|
|{{act_desc}}|{{sen_desc}}|


## {{ module }} pinout

The {{ module }} module allows you to use the pins of the L0 board through the Luos system. You can use `Digital Write`, `Digital Read`, or `Analog Read` pins.

<blockquote class="warning"><strong>Warning:</strong> Warning: The pins only support 3.3V.</blockquote><br />

![GPIO pinout](/assets/img/GPIO_pinout.png)

## How to use the {{ module }} module using Pyluos

To control the {{ module }} module, you have to connect it in a Luos network and connect this network through a communication module to Pyluos.

Then you can access to your module and control it.

To get an input pin value, you can use:

```python
robot.gpio_mod.analog_1
```
 
You can read these outputs :

* `analog_1`
* `analog_7`
* `analog_8`
* `analog_9`
* `digital_5`
* `digital_6`

To set an output pin value you can use:

```python
robot.gpio_mod.digital_4.set_high()
robot.gpio_mod.digital_4.set_low()
robot.gpio_mod.digital_4.toggle()
```
 
You can write these inputs :

* `digital_4`
* `digital_3`
* `digital_2`

----

## Functions
List of functions of {{module}} module:

| **is_high(self)** | - | - | 
| **is_low(self)** | - | - | 
| **read(self)** | - | - | 
| **set_high(self)** | - | - | 
| **set_low(self)** | - | - | 
| **toggle(self)** | - | - | 
| **-** | - | - | 

## Variables
List of variables of {{module}} module:

| **duty_cycle(self, duty)** | - | - | 
| **-** | - | - | 

## Events
List of events of {{module}} module:

| **-** | - | - | 
