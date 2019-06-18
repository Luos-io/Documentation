---
layout: post
title:  "Power-switch"
date:   2019-03-15 17:49:00 +0100
categories: board
tags: [actuation]
---
{% assign module = "Power-switch" %}
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

## How to use the {{ module }} module using Pyluos

To control the {{ module }} module, you have to connect it in a Luos network and connect this network through a communication module to Pyluos.
Then you can access to your module and control it.

This module can electrically open and close a circuit. You can get the state or set a state to the {{ module }}.

To get the state of the module, use the command:

```python
robot.powerSwitch.state
```

To set the state of the module, use the command:

```python
robot.powerSwitch.state(True/False)
```

`True` closes the circuit, while `False` opens it.

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
