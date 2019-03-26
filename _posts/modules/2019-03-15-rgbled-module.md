---
layout: post
title:  "RGB LED module"
date:   2019-03-15 17:47:00 +0100
categories: modules
tags: [actuation, interface]
---

{% assign module = "RGB-LED" %}
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
