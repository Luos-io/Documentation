---
layout: post
title:  "Wifi/BLE module"
date:   2019-03-15 17:43:00 +0100
categories: modules
tags: [communication]
---

{% assign module = "Wifi-BLE" %}
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



## How to configure the Wifi network

This module allows a communication of the Luos network where it is plugged to with Wifi. 

1. Plug the module to a powered Luos network or to a power source. It auomatically turns the Wifi on.
2. Connect your computer to the modules's Wifi. A page automatically opens in your default browser.
3. On the page, choose the name of the Wifi network or choose an existing networ to connect to.




## How to use the {{ module }} module with pyluos



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
