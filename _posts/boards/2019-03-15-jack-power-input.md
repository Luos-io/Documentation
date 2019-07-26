---
layout: post
title:  "Jack power input"
date:   2019-03-15 17:57:00 +0100
categories: board
tags: [power]
---
{% assign board = "Jack power input" %}
{% assign alias = "" %}
{% assign type = "" %}
{% include var.md %}

# How to start with the {{ board }} board
{% include card.md %}

## How to use {{ board }} board

The {{ board }} board allows to power your Luos Network using a power Jack.

 - plug inner diameter : 2 mm
 - plug outer diameter : 6.4 mm

This board is not active, you can't detect it.
The {{ board }} board can provide 5V to 24V DC.

You can manage multiple voltage in the same network following Luos power rules defined in [Luos boards general use]({{ "/electronic-use" | absolute_url }}) or by using a [power isolator board]({{ "/board/power-isolator" | absolute_url }}).
