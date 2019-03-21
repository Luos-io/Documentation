---
layout: post
title:  "DC-motor module"
date:   2019-03-15 17:57:00 +0100
categories: modules
tags: [Actuation]
---
{% include var.md %}
{% assign module = "DC-motor" %}

# How to start with the {{ module }} module

This guide contains all the basic notions you will need to use the {{ module }} module.

<div class="sheet" markdown="1">

<p class="sheet-title" markdown="1">**Name**</p>

<p class="indent" markdown="1">{{module}}</p>

<p class="sheet-title" markdown="1">**Type**</p>

<p class="indent" markdown="1">`DCMotor`</p>

<p class="sheet-title" markdown="1">**Image**</p>

<p class="indent" markdown="1">![DC-motor module](/assets/img/dc-module.png)</p>

<p class="sheet-title" markdown="1">**Categories**</p>

<p class="indent" markdown="1">
{% for tag in page.tags %}
  <a href="{{ "/" | absolute_url }}tags.html">{{ tag }}</a>
{% endfor %}
</p>
</div>


## Module categories

|<a href="{{ "/" | absolute_url }}tags.html">{{act_title}}</a>|
|:-|
|![{{act_title}}]({{act_img}})|
|{{act_desc}}| 


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
