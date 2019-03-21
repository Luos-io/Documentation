---
layout: post
title:  "Light module"
date:   2019-03-15 17:52:00 +0100
categories: modules
tags: [Sensor]
---
{% include var.md %}
{% assign module = "Light" %}

# How to start with the {{ module }} module

This guide contains all the basic notions you will need to use the {{ module }} module.

<div class="sheet" markdown="1">

<p class="sheet-title" markdown="1">**Name**</p>

<p class="indent" markdown="1">{{module}}</p>

<p class="sheet-title" markdown="1">**Type**</p>

<p class="indent" markdown="1">`LightSensor`</p>

<p class="sheet-title" markdown="1">**Image**</p>

<p class="indent" markdown="1">![{{ module }} module](/assets/img/light-module.png)</p>

<p class="sheet-title" markdown="1">**Categories**</p>

<p class="indent" markdown="1">
{% for tag in page.tags %}
  <a href="{{ "/" | absolute_url }}tags.html">{{ tag }}</a>
{% endfor %}
</p>
</div>


## Module categories

|<a href="{{ "/" | absolute_url }}tags.html">{{sen_title}}</a>|
|:-|
|![{{sen_title}}]({{sen_img}})|
|{{sen_desc}}| 


## How to use the {{ module }} module using Pyluos

To control the Light module, you have to connect it to a Luos network and connect this network through a communication module to Pyluos.

Then you can access to your module and control it.

To get the measured light in `lux`, you can use this command:

```python
robot.lightSensor.lux
```


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
