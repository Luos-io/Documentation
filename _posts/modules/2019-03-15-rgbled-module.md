---
layout: post
title:  "RGB LED module"
date:   2019-03-15 17:47:00 +0100
categories: modules
tags: [Actuation, Interface]
---
{% include var.md %}
{% assign module = "RGB LED" %}

# How to start with the {{ module }} module

This guide contains all the basic notions you will need to use the {{ module }} module.

<div class="sheet" markdown="1">

<p class="sheet-title" markdown="1">**Name**</p>

<p class="indent" markdown="1">{{module}}</p>

<p class="sheet-title" markdown="1">**Type**</p>

<p class="indent" markdown="1">`LED`</p>

<p class="sheet-title" markdown="1">**Image**</p>

<p class="indent" markdown="1">![{{ module }} module](/assets/img/led-module.png)</p>

<p class="sheet-title" markdown="1">**Categories**</p>

<p class="indent" markdown="1">
{% for tag in page.tags %}
  <a href="{{ "/" | absolute_url }}tags.html">{{ tag }}</a>
{% endfor %}
</p>
</div>


## Module categories

|<a href="{{ "/" | absolute_url }}tags.html">{{ act_title }}</a>|<a href="{{ "/" | absolute_url }}tags.html">{{ int_title }}</a>|
|:-|:-|
|![{{ act_title }}]({{ act_img }})|![{{ int_title }}]({{ int_img }})|
|{{ act_desc }}|{{ int_desc }}|


## How to use the {{ module }} module using Pyluos

To control the {{ module }} module you have to connect it in a Luos network and connect this network through a communication module to Pyluos. Then you can access to your module and control it.

To set your RGB led brightness and color using an RGB standard:

```python
robot.rgb_led_mod.color = [12, 128, 233]
robot.rgb_led_mod.control()
```
 
The values `[0, 0, 0]` turn the LED off.

{{ actuation_desc }}

----

## Functions
List of functions of {{module}} module:

| **change_color(red, green, blue)** | - | - | 
| **control(self)** | - | - | 
| **-** | - | - | 

## Variables
List of variables of {{module}} module:

| **color(self, new_color)** | - | - | 
| **-** | - | - | 

## Events
List of events of {{module}} module:
 
| **-** | - | - | 
