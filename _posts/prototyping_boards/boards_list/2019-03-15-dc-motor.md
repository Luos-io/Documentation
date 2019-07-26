---
layout: post
title:  "DC-motor"
date:   2019-03-15 17:57:00 +0100
categories: -_boards_list
tags: [actuation]
---
{% assign board = "DC-motor" %}
{% assign alias = "DC_motor1_mod, DC_motor2_mod" %}
{% assign type = "[DC-motor](/../modules_list/dc-motor)" %}
{% include var.md %}

# How to start with the {{ board }} board
{% include card.md %}

## How to use {{ board }} board

The {{ board }} board allows to drive up to 2 low-power DC motors. This board is useful to easily drive a small and simple rover.
The {{ board }} board supports 5V to 12V DC input to drive 5V to 12V DC motors up to 2 x 1.5 A (2 A peak).
