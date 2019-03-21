---
layout: post
title:  "Button module"
date:   2019-03-15 17:59:00 +0100
categories: modules
tags: [Sensor, Interface]
---
{% include var.md %}
{% assign module = "Button" %}

# How to start with the {{ module }} module

This guide contains all the basic notions you will need to use the {{ module }} module.

<div class="sheet" markdown="1">

<p class="sheet-title" markdown="1">**Name**</p>

<p class="indent" markdown="1">{{module}}</p>

<p class="sheet-title" markdown="1">**Type**</p>

<p class="indent" markdown="1">`Button`</p>

<p class="sheet-title" markdown="1">**Image**</p>

<p class="indent" markdown="1">![{{ module }} module](/assets/img/button-module.png)</p>

<p class="sheet-title" markdown="1">**Categories**</p>

<p class="indent" markdown="1">
{% for tag in page.tags %}
  <a href="{{ "/" | absolute_url }}tags.html">{{ tag }}</a>
{% endfor %}
</p>
</div>

## Module categories

|<a href="{{ "/" | absolute_url }}tags.html">{{ sen_title }}</a>|<a href="{{ "/" | absolute_url }}tags.html">{{ int_title }}</a>|
|:-|:-|
|![{{ sen_title }}]({{ sen_img }})|![{{ int_title }}]({{ int_img }})|
|{{ sen_desc }}|{{ int_desc }}|


## How to use the {{ module }} module with Pyluos
To control the {{ module }} module, you have to connect it in a Luos network and connect this network through a communication module to Pyluos.

Then you can access to your module and control it.

To get the state of the button you can use:

```pyhton
robot.button_mod.pressed
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

| **possible_events = {'changed', 'pressed', 'released'}** | - | - | 

## Properties
List of properties of {{module}} module:

| **pressed(self)** | - | - | 
