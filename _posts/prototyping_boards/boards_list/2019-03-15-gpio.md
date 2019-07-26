---
layout: post
title:  "GPIO"
date:   2019-03-15 17:59:00 +0100
categories: -_boards_list
tags: [sensor, interface]
---
{% assign board = "GPIO" %}
{% assign alias = "analog_read_P1, analog_read_P7, analog_read_P8, analog_read_P9, digit_read_P5, digit_read_P6, digit_write_P2, digit_write_P3, digit_write_P4"  %}
{% assign type = "[State](/../modules_list/state), [Voltage](/../modules_list/voltage)" %}
{% include var.md %}

<blockquote class="warning"><strong>Warning:</strong> This board and the associated page are in <strong>work in progress</strong></blockquote><br />


# How to start with the {{ board }} board
{% include card.md %}


## {{ board }} pinout and power consideration

The {{ board }} board allows you to use the pins of the L0 board through the Luos system. You can use `Digital Write`, `Digital Read`, or `Analog Read` pins.

The {{ board }} board supports 5V to 24V DC input.

<blockquote class="warning"><strong>Warning: </strong>The pins only support 3.3V.</blockquote><br />

![GPIO pinout]({{ "/" | absolute_url }}/assets/img/GPIO_pinout.png)

This board creates a module for each available pin.
