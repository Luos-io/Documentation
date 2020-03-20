---
layout: post
title:  "Power isolator"
date:   2019-03-15 17:57:00 +0100
categories: -_boards_list
tags: [power]
---
{% assign board = "Power isolator" %}
{% assign alias = "N/A" %}
{% assign type = "N/A" %}
{% include var.md %}

# How to start with the {{ board }} board
{% include card.md %}

## How to use {{ board }} board

The {{ board }} board allow to manage multiple voltage into the same Luos network. This board is not active, you can't detect it.
This board isolate voltage between each side. You can use it to link component with different voltage needs. For example to XL320 (7V) and MX28(12V) on the same network you can use a Dynamixel V2 board with a 7V power module for XL320 and a Dynamixel V1 board with a 12V power module to link this 2 different voltage you can use the power isolator. You can connect a gate in the side you want without trouble.

<img height="350" src="{{ "/" | absolute_url }}/assets/img/power_isolator_example.png" alt="Power isolator example">

