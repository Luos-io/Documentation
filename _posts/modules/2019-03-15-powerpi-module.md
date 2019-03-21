---
layout: post
title:  "Power Pi module"
date:   2019-03-15 17:50:00 +0100
categories: modules
tags: [Communication, Cognition, Power]
---
{% include var.md %}
{% assign module = "Power Pi" %}

# How to start with the {{ module }} module

This guide contains all the basic notions you will need to use the {{ module }} module.

<div class="sheet" markdown="1">

<p class="sheet-title" markdown="1">**Name**</p>

<p class="indent" markdown="1">{{module}}</p>

<p class="sheet-title" markdown="1">**Type**</p>

<p class="indent" markdown="1">``</p>

<p class="sheet-title" markdown="1">**Image**</p>

<p class="indent" markdown="1">![{{ module }} module](/assets/img/powerpi-module.png)</p>

<p class="sheet-title" markdown="1">**Categories**</p>

<p class="indent" markdown="1">
{% for tag in page.tags %}
  <a href="{{ "/" | absolute_url }}tags.html">{{ tag }}</a>
{% endfor %}
</p>
</div>



## Module categories

|<a href="{{ "/" | absolute_url }}tags.html">{{com_title}}</a>|<a href="{{ "/" | absolute_url }}tags.html">{{cog_title}}</a>|<a href="{{ "/" | absolute_url }}tags.html">{{pow_title}}</a>|
|:-|:-|
|![{{ com_title }}]({{ com_img }})|![{{ cog_title }}]({{ cog_img }})|![{{ pow_title }}]({{ pow_img }})|
|{{com_desc}}|{{cog_desc}}|{{pow_desc}}|


## Connection of Power Pi module to a board
The connection of the Power Pi module to an ODrive board or to a Raspberry Pi board is made according to the following images.

<blockquote class="warning"><strong>Warning:</strong> Warning: Be sure to plug the module on the correct pins of the board, and facing the correct side. A bad connection may damage both the board and the module.</blockquote><br />

![Plug location](/assets/img/power-pi-1.png)<br />
*Red rectangles show where to plug the Power Pi module on an ODrive board (left) and on a Raspberry Pi board (right).*

![Preview](/assets/img/power-pi-2.png)<br />
*On the left, a module connected to an ODrive board; on the right, a module connected to a Raspberry Pi board.*

 

## Power Pi vs. Raspberry Pi modules
For the technical documentation of the Power Pi module, please refer to the page [Raspberry Pi module]({{ site.baseurl }}{% post_url modules/2019-03-15-rpi-module %}).

The difference between Power Pi and Raspberry Pi modules is that the second can only be plugged with one particular Raspberry Pi board (model ZeroW, which is included into the package), whereas the Power Pi module can be plugged to any version of Raspberry Pi boards, as well as ODroid boards. 

This Power Pi module doesn’t include a Raspberry Pi or ODroid board.

 

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
