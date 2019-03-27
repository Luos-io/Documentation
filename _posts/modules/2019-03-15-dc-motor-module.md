---
layout: post
title:  "DC-motor module"
date:   2019-03-15 17:57:00 +0100
categories: [module]
tags: [actuation]
---
{% assign module = "DC-motor" %}
{% include var.md %}

# How to start with the {{ module }} module

This guide contains all the basic notions you will need to use the {{ module }} module.

<div class="sheet" markdown="1">
<p class="sheet-title" markdown="1">**Name:** {{module}}</p>
<p class="sheet-title" markdown="1">**Type:** {{type}}</p>
<p class="sheet-title" markdown="1">**Image**</p>
<p class="indent" markdown="1"><img height="150" src="/assets/img/{{ module | downcase }}-module.png" alt="{{ tag | Capitalize }}"></p>
<p class="sheet-title" markdown="1">**Categories**</p>
<p class="indent" markdown="1">
{% for tag in page.tags %}
  <a href="{{ "/" | absolute_url }}tags.html"><img height="50" src="/assets/img/sticker-{{ tag }}.png" title="{{ tag | capitalize }}" alt="{{ tag | capitalize }}"></a>
{% endfor %}
</p>
</div>


## How to control {{ module }} module with pyluos

To control the {{ module }} through a Luos Robotics module, you can use the power ratio value between `-100%` and `100%`:

```python
robot.DC_motor1_mod.power_ratio = -73.3
```

According to the sign of the value and to the side of each motor’s wire, the motor will turn in a direction or the other.

----

## Functions
List of functions of {{module}} module:

| **control(self)** |  |  | 
| **-** | - | - | 

## Variables
List of variables of {{module}} module:

| **power_ratio(self, s):** |  |  | 
| **-** | - | - | 

## Events
List of events of {{module}} module:

| **-** | - | - | 
