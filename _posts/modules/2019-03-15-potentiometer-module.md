---
layout: post
title:  "Potentiometer module"
date:   2019-03-15 17:51:00 +0100
categories: modules
tags: [interface, sensor]
---
{% include var.md %}
{% assign module = "Potentiometer" %}
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
To control the potentiometer module, you have to connect it in a Luos network and connect this network through a communication module to Pyluos.

Then you can access to your module and control it.

To get the state of your potentiometer, you can use:

```python
robot.potentiometer_mod.position
```

It returns an angular value in `degrees` between `0` and `300`.

For an example, refer to the video tutorial (Tutorial 1) of the [Quick start page]({{ "/" | absolute_url }}quick-start.html)

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
