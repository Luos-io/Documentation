---
layout: post
title:  "Distance module"
date:   2019-03-15 17:56:00 +0100
categories: board
tags: [sensor]
---
{% assign module = "Distance" %}
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
  <a href="{{ "/" | absolute_url }}tags.html"><img height="50" src="/assets/img/sticker-{{ tag }}.png" alt="{{ tag | capitalize }}"></a>
{% endfor %}
</p>
</div>

## How to control the {{ module }} module with pyluos

To control the {{ module }} module, you have to connect it to a Luos network and connect this network through a communication module to Pyluos.
Then you can access to your module and control it.

To get the measured distance in millimeter you can use:

```python
robot.lidar_mod.distance
```

----

## Functions
List of functions of {{module}} module:

| **distance(self)** |  |  |
| **-** | - | - |

## Variables
List of variables of {{module}} module:

| **-** | - | - |

## Events
List of events of {{module}} module:

| **-** | - | - |
