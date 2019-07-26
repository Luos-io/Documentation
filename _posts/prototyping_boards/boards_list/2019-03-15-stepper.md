---
layout: post
title:  "Stepper"
date:   2019-03-15 17:45:00 +0100
categories: -_boards_list
tags: [actuation]
---
{% assign board = "Stepper" %}
{% assign alias = "stepper_mod" %}
{% assign type = "[Stepper](/../modules_list/stepper)" %}
{% include var.md %}

# How to start with the {{ board }} board
{% include card.md %}

## How to connect the stepper motor to the {{ board }} board
The {{ board }} board have one PH connector with 4 pins where a stepper motor can be plugged.

This board accepts supply voltage from `7V` to `24V`.

<blockquote class="warning"><strong>Warning:</strong> USB board provides too weak power to drive a motor-reducer with the {{ board }} board. A power board such as Battery board or Power plug board shall be used.</blockquote><br />
